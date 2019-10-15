#include "TeleOperat_Controller.hpp"

void TeleOperat_Controller::runController() {
  Mat3<float> kpMat;
  Mat3<float> kdMat;
  // kpMat << 10, 0, 0, 0, 10, 0, 0, 0, 10;
  // kdMat << 2.1, 0, 0, 0, 2.1, 0, 0, 0, 2.1;
  kpMat << 20., 0, 0, 0, 20., 0, 0, 0, 10.;
  kdMat << 0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.03;

  for (int jidx(0); jidx < 3; ++jidx) {
    _legController->commands[2].qDes[jidx] = _legController->datas[3].q[jidx];
    _legController->commands[2].qdDes[jidx] = 0.;
    _legController->commands[2].tauFeedForward[jidx] = 0.;
    _legController->commands[3].qDes[jidx] = _legController->datas[2].q[jidx];
    _legController->commands[3].qdDes[jidx] = 0.;
    _legController->commands[3].tauFeedForward[jidx] = 0.;

    _legController->commands[2].kpJoint = kpMat;
    _legController->commands[2].kdJoint = kdMat;
    _legController->commands[3].kpJoint = kpMat;
    _legController->commands[3].kdJoint = kdMat;
  }
}
