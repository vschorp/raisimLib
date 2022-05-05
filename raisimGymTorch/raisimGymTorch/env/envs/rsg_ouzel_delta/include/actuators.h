/*
 *  Weixuan Zhang 21.07.2021
 *  Eugenio Cuniato 21.07.2021
 */

//#include "raisim/World.hpp"
//#include "mav_msgs/Actuators.h"
//#include "ros_raisim_interface/parser.h"
#include "pid_controller.h"
#include <cmath>
//#include <tinyxml.h>


#ifndef RAISIM_SIMULATOR_ACTUATORS_
#define RAISIM_SIMULATOR_ACTUATORS_

namespace raisim_actuators {
  class delta_joint {
    public:
      delta_joint(double sampleTime);

      bool send_cmd(double cmd);

      const double getPos() {
        return x_;
      }
      const double getVel() {
        return xd_;
      }
      const double getAcc() {
        return xdd_;
      }

    public: PID pid_;

    private:
      double sampleTime_;
      double x_,xd_,xdd_;
  };
}


#endif // !RAISIM_SIMULATOR_ACTUATORS_