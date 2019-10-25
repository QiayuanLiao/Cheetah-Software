#include <SimUtilities/IMUTypes.h>
#include <errno.h>   /* Error number definitions */
#include <fcntl.h>   /* File control definitions */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <lcm/lcm-cpp.hpp>

#include "Utilities/utilities.h"
#include "rt/rt_hi220.h"
#include "vectornav_lcmt.hpp"

#define HI220_SERIAL "/dev/ttyUSB0"
//#define PRINT_IMU_DEBUG

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#ifndef CH_OK
#define CH_OK (0)
#endif

#ifndef CH_ERR
#define CH_ERR (1)
#endif

static Packet_t RxPkt;
static int16_t acc[3];
static int16_t gyo[3];
static int16_t mag[3];
static float eular[3];
static float quat[4]={0,0,0,1};
static uint8_t id;

/*  callback function of  when recv a data frame successfully */
static void OnDataReceived(Packet_t *pkt) {
  if (pkt->type != 0xA5) {
    return;
  }

  int offset = 0;
  uint8_t *p = pkt->buf;
  while (offset < pkt->payload_len) {
    switch (p[offset]) {
      case kItemID:
        id = p[1];
        offset += 2;
        break;
      case kItemAccRaw:
      case kItemAccCalibrated:
      case kItemAccFiltered:
      case kItemAccLinear:
        memcpy(acc, p + offset + 1, sizeof(acc));
        offset += 7;
        break;
      case kItemGyoRaw:
      case kItemGyoCalibrated:
      case kItemGyoFiltered:
        memcpy(gyo, p + offset + 1, sizeof(gyo));
        offset += 7;
        break;
      case kItemMagRaw:
      case kItemMagCalibrated:
      case kItemMagFiltered:
        memcpy(mag, p + offset + 1, sizeof(mag));
        offset += 7;
        break;
      case kItemRotationEular:
        eular[0] =
            ((float)(int16_t)(p[offset + 1] + (p[offset + 2] << 8))) / 100;
        eular[1] =
            ((float)(int16_t)(p[offset + 3] + (p[offset + 4] << 8))) / 100;
        eular[2] =
            ((float)(int16_t)(p[offset + 5] + (p[offset + 6] << 8))) / 10;
        offset += 7;
        break;
      case kItemRotationEular2:
        memcpy(eular, p + offset + 1, sizeof(eular));
        offset += 13;
        break;
      case kItemRotationQuat:
        memcpy(quat, p + offset + 1, sizeof(quat));
        offset += 17;
        break;
      case kItemPressure:
        offset += 5;
        break;
      case kItemTemperature:
        offset += 5;
        break;
      default:
        printf("data decode wrong\r\n");
        return;
        break;
    }
  }
}

static void crc16_update(uint16_t *currectCrc, const uint8_t *src,
                         uint32_t lengthInBytes) {
  uint32_t crc = *currectCrc;
  uint32_t j;
  for (j = 0; j < lengthInBytes; ++j) {
    uint32_t i;
    uint32_t byte = src[j];
    crc ^= byte << 8;
    for (i = 0; i < 8; ++i) {
      uint32_t temp = crc << 1;
      if (crc & 0x8000) {
        temp ^= 0x1021;
      }
      crc = temp;
    }
  }
  *currectCrc = crc;
}

uint32_t Packet_CreatePing(Packet_t *pkt) {
  pkt->buf[0] = 0x5A;
  pkt->buf[1] = 0xA6;
  pkt->payload_len = 0;
  pkt->len = 2;
  return CH_OK;
}

uint32_t Packet_CreatePingAck(Packet_t *pkt, uint8_t major, uint8_t minor,
                              uint8_t bugfix, uint16_t option) {
  pkt->buf[0] = 0x5A;
  pkt->buf[1] = 0xA7;

  /* protocol bug fix */
  pkt->buf[2] = bugfix;
  /* protocol minor */
  pkt->buf[3] = minor;
  /* protocol major */
  pkt->buf[4] = major;
  pkt->buf[5] = 'P';

  /* option low: sender's address low */
  pkt->buf[6] = (option & 0x00FF) >> 0;

  /* option high: sender's address high */
  pkt->buf[7] = (option & 0xFF00) >> 8;

  /* crc */
  uint16_t crc;
  crc = 0;
  crc16_update(&crc, &pkt->buf[0], 8);
  pkt->buf[8] = (crc & 0x00FF) >> 0;
  pkt->buf[9] = (crc & 0xFF00) >> 8;

  pkt->payload_len = 0;
  pkt->type = 0xA7;
  pkt->len = 10;
  return CH_OK;
}

uint32_t Packet_Begin(Packet_t *pkt) {
  pkt->ofs = 6; /* sof(2) len(2) + crc(2) */
  memset(&pkt->buf[0], 0, sizeof(pkt->buf));
  pkt->buf[0] = 0x5A; /* header */
  pkt->buf[1] = 0xA5; /* data packet */
  return CH_OK;
}

uint32_t Packet_AddData(Packet_t *pkt, uint8_t *buf, uint16_t len) {
  /* add item content into buffer */
  memcpy((pkt->buf + pkt->ofs), buf, len);
  pkt->ofs += len;
  return CH_OK;
}

uint32_t Packet_Final(Packet_t *pkt) {
  pkt->payload_len = pkt->ofs - 6;
  pkt->len = pkt->ofs;

  pkt->buf[2] = (pkt->payload_len & 0x00FF) >> 0;
  pkt->buf[3] = (pkt->payload_len & 0xFF00) >> 8;

  /* crc */
  uint16_t crc;
  crc = 0;
  crc16_update(&crc, &pkt->buf[0], 4);
  crc16_update(&crc, &pkt->buf[6], pkt->payload_len);
  pkt->buf[4] = (crc & 0x00FF) >> 0;
  pkt->buf[5] = (crc & 0xFF00) >> 8;

  return CH_OK;
}

enum status {
  kStatus_Idle,
  kStatus_Cmd,
  kStatus_LenLow,
  kStatus_LenHigh,
  kStatus_CRCLow,
  kStatus_CRCHigh,
  kStatus_Data,
};

uint32_t Packet_Decode(uint8_t c) {
  static uint16_t CRCReceived = 0;      /* CRC value received from a frame */
  static uint16_t CRCCalculated = 0;    /* CRC value caluated from a frame */
  static uint8_t status = kStatus_Idle; /* state machine */
  static uint8_t crc_header[4] = {0x5A, 0xA5, 0x00, 0x00};

  switch (status) {
    case kStatus_Idle:
      if (c == 0x5A) status = kStatus_Cmd;
      break;
    case kStatus_Cmd:
      RxPkt.type = c;
      switch (RxPkt.type) {
        case 0xA5: /* Data */
          status = kStatus_LenLow;
          break;
        case 0xA6: /* Ping */
          OnDataReceived(&RxPkt);
          status = kStatus_Idle;
          break;
        case 0xA7: /* Ping Respond */
          RxPkt.ofs = 0;
          status = kStatus_Data;
          break;
      }
      break;
    case kStatus_LenLow:
      RxPkt.payload_len = c;
      crc_header[2] = c;
      status = kStatus_LenHigh;
      break;
    case kStatus_LenHigh:
      RxPkt.payload_len |= (c << 8);
      crc_header[3] = c;
      status = kStatus_CRCLow;
      break;
    case kStatus_CRCLow:
      CRCReceived = c;
      status = kStatus_CRCHigh;
      break;
    case kStatus_CRCHigh:
      CRCReceived |= (c << 8);
      RxPkt.ofs = 0;
      CRCCalculated = 0;
      status = kStatus_Data;
      break;
    case kStatus_Data:
      RxPkt.buf[RxPkt.ofs++] = c;
      if (RxPkt.type == 0xA7 && RxPkt.ofs >= 8) {
        RxPkt.payload_len = 8;
        OnDataReceived(&RxPkt);
        status = kStatus_Idle;
      }
      if (RxPkt.ofs >= MAX_PACKET_LEN) {
        status = kStatus_Idle;
        return CH_ERR;
      }

      if (RxPkt.ofs >= RxPkt.payload_len && RxPkt.type == 0xA5) {
        /* calculate CRC */
        crc16_update(&CRCCalculated, crc_header, 4);
        crc16_update(&CRCCalculated, RxPkt.buf, RxPkt.ofs);

        /* CRC match */
        if (CRCCalculated == CRCReceived) {
          OnDataReceived(&RxPkt);
        }
        status = kStatus_Idle;
      }
      break;
    default:
      status = kStatus_Idle;
      break;
  }
  return CH_OK;
}

int get_raw_acc(int16_t *a) {
  memcpy(a, acc, sizeof(acc));
  return 0;
}

int get_raw_gyo(int16_t *g) {
  memcpy(g, gyo, sizeof(gyo));
  return 0;
}

int get_raw_mag(int16_t *m) {
  memcpy(m, mag, sizeof(mag));
  return 0;
}

int get_eular(float *e) {
  memcpy(e, eular, sizeof(eular));
  return 0;
}

int get_quat(float *q) {
  memcpy(q, quat, sizeof(quat));
  return 0;
}
int get_omega(float *o) {
  int16_t gyo_raw[3];
  memcpy(gyo_raw, gyo, sizeof(gyo));
  o[0] = (float)gyo_raw[0];
  o[1] = (float)gyo_raw[1];
  o[2] = (float)gyo_raw[2];

  o[0] = o[0] * 0.001745f;
  o[1] = o[1] * 0.001745f;
  o[2] = o[2] * 0.001745f;

  return 0;
}
int get_acc(float *a) {
  int16_t acc_raw[3];
  memcpy(acc_raw, acc, sizeof(acc));

  a[0] = (float)acc_raw[0];
  a[1] = (float)acc_raw[1];
  a[2] = (float)acc_raw[2];

  a[0] = a[0] * 0.01f;
  a[1] = a[1] * 0.01f;
  a[2] = a[2] * 0.01f;

  return 0;
}

int get_id(uint8_t *user_id) {
  *user_id = id;
  return 0;
}

static lcm::LCM *vectornav_lcm;
vectornav_lcmt vectornav_lcm_data;

static VectorNavData *g_vn_data = nullptr;

int init_hi220(VectorNavData *vn_data) {
  g_vn_data = vn_data;
  printf("[Simulation] Setup LCM...\n");
  vectornav_lcm = new lcm::LCM(getLcmUrl(255));
  if (!vectornav_lcm->good()) {
    printf("[ERROR] Failed to set up LCM\n");
  }

  struct termios options;

  int fd = open(HI220_SERIAL, O_RDWR | O_NOCTTY | O_NONBLOCK);

  tcgetattr(fd, &options);

  if (fd == -1) {
    printf("[rt_HI220] Unable to open HI220\n");
    return (-1);
  }

  if (fcntl(fd, F_SETFL, 0) < 0) {
    printf("[rt_HI220] Fcntl failed\n");
  } else {
    fcntl(fd, F_SETFL, 0);
  }

  if (isatty(STDIN_FILENO) == 0) {
    printf("[rt_HI220] Standard input is not a terminal device\n");
  } else {
    printf("[rt_HI220] Isatty success!\n");
  }

  bzero(&options, sizeof(options));

  options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  options.c_cc[VTIME] = 0;
  options.c_cc[VMIN] = 0;
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &options);
  return fd;
}

void receive_hi220(int port) {
  float Eular[3] = {0};
  float Quat[4] = {0,0,0,1};
  float Omega[3] = {0};
  float a[3] = {0};
  uint8_t buf[1024];

  ssize_t n = read(port, buf, sizeof(buf));

  for (int i = 0; i < n; i++) {
    Packet_Decode(buf[i]);
  }
  get_acc(a);
  get_eular(Eular);
  get_quat(Quat);
  get_omega(Omega);

    vectornav_lcm_data.q[0] = Quat[1];
    g_vn_data->quat[0] = Quat[1];
    vectornav_lcm_data.q[1] = Quat[2];
    g_vn_data->quat[1] = Quat[2];
    vectornav_lcm_data.q[2] = Quat[3];
    g_vn_data->quat[2] = Quat[3];
    vectornav_lcm_data.q[3] = Quat[0];
    g_vn_data->quat[3] = Quat[0];
  for (int i = 0; i < 3; i++) {
    vectornav_lcm_data.w[i] = Omega[i];
    vectornav_lcm_data.a[i] = a[i];
    g_vn_data->gyro[i] = Omega[i];
    g_vn_data->accelerometer[i] = a[i];
  }

#ifdef PRINT_IMU_DEBUG

  printf("Eular(P R Y):%0.2f %0.2f %0.2f\r\n", Eular[0], Eular[1], Eular[2]);
  printf("Quat(W X Y Z):%0.3f %0.3f %0.3f %0.3f\r\n", Quat[0], Quat[1], Quat[2],
         Quat[3]);
  printf("Acc(X Y Z):%0.3f %0.3f %0.3f\r\n", a[0], a[1], a[2]);
  printf("Omega(P R Y):%0.3f %0.3f %0.3f\r\n", Omega[0], Omega[1], Omega[2]);

#endif
  vectornav_lcm->publish("hw_vectornav", &vectornav_lcm_data);
}
