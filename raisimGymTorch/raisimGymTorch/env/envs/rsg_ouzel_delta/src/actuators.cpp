/*
 *  Weixuan Zhang 21.07.2021
 *  Eugenio Cuniato 21.07.2021
 */

#include "include/actuators.h"

namespace raisim_actuators {
  delta_joint::delta_joint(double sampleTime) {
        sampleTime_ = sampleTime;
        double scaling = 10;
        pid_.SetPGain(scaling*50.0);
        pid_.SetIGain(scaling*1.0);
        pid_.SetDGain(scaling*5.0);
        pid_.SetIMin(-20);
        pid_.SetIMax(20);
        pid_.SetCmdMin(-100);
        pid_.SetCmdMax(100);
        x_ = xd_ = xdd_ = 0.0;
      }

  bool delta_joint::send_cmd(double cmd) { //Position command
        double error = (cmd - x_);
        // ROS_INFO_STREAM(xdd_<<" "<<xd_<<" "<<x_<<" "<<error<<" "<<sampleTime_);
        double u = pid_.Update(-error, sampleTime_);
        if(std::isnan(u))
          return false;
        // xdd_ = 1*error - 0.4*xd_ ;
        xdd_ = u;
        xd_ += xdd_*sampleTime_;
        x_ += xd_*sampleTime_;
        return true;
        // this->setEffort(cmd);
      }

  void delta_joint::set_angle(double angle) {
    x_ = angle;
    xd_ = 0.0;
    xdd_ = 0.0;
  }
}