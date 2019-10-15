#ifndef TELEOPERAT_CONTROLLER
#define TELEOPERAT_CONTROLLER

#include <RobotController.h>
#include "TeleOperatUserParameters.h"

class TeleOperat_Controller : public RobotController {
 public:
  TeleOperat_Controller()
      : RobotController(), _jpos_ini(cheetah::num_act_joint) {
    _jpos_ini.setZero();
  }
  virtual ~TeleOperat_Controller() {}

  virtual void initializeController() {}
  virtual void runController();
  virtual void updateVisualization() {}
  virtual ControlParameters* getUserControlParameters() {
    return &userParameters;
  }

 protected:
  DVec<float> _jpos_ini;
  JPosUserParameters userParameters;
};

#endif
