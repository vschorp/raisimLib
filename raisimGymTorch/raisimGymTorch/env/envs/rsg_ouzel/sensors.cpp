/*
 *  Weixuan Zhang 21.07.2021
 *  Eugenio Cuniato 21.07.2021
 */

#include "sensors.h"

namespace raisim_sensors {

  void accelerometer::update() {
    raisim::Vec<3> robot_linear_velocity_W, imu_linear_velocity_W;
    raisim::Mat<3,3> robot_orientation_rotmat;

    //  emulate the imu signals
    robot_->getVelocity(baseLink_, robot_linear_velocity_W);
    robot_->getVelocity(baseLink_, imu_wrt_link_offset_ , imu_linear_velocity_W);
    robot_->getBaseOrientation(robot_orientation_rotmat);

    measureGT_ = robot_orientation_rotmat.e().transpose()*((imu_linear_velocity_W.e() - prev_imu_linear_velocity_W_.e())/sampleTime_ - gravity_);

    prev_robot_linear_velocity_W_ = robot_linear_velocity_W;
    prev_imu_linear_velocity_W_ = imu_linear_velocity_W;

    if(noiseSource_!=NULL) {
      measure_ = measureGT_ + noiseSource_->getNoise();
    } else {
      measure_ = measureGT_;
    }
  }

  void gyroscope::update() {
    raisim::Vec<3> robot_angular_velocity_W;
    raisim::Mat<3,3> robot_orientation_rotmat;

    //  emulate the gyro signals
    robot_->getBaseOrientation(robot_orientation_rotmat);
    robot_->getAngularVelocity(baseLink_, robot_angular_velocity_W);

    measureGT_ = robot_orientation_rotmat.e().transpose()*robot_angular_velocity_W.e();

    if(noiseSource_!=NULL) {
      measure_ = measureGT_ + noiseSource_->getNoise();
    } else {
      measure_ = measureGT_;
    }
  }

  imuNoise::imuNoise(double sampleTime, imu_param parameters) : noise(sampleTime), params_(parameters) {
    for (int i = 0; i < 3; ++i) {
      turnOnBias_[i] =
        params_.turnOnBias_sigma * standardNormalDistribution_(randomGenerator_);
    };
    bias_.setZero();
  }

  Eigen::Vector3d imuNoise::getNoise() {
    double tau_a = params_.bias_correlation_time;
    // Discrete-time standard deviation equivalent to an "integrating" sampler
    // with integration time sampleTime_.
    double sigma_a_d = 1 / sqrt(sampleTime_) * params_.noise_density;
    double sigma_b_a = params_.random_walk;
    // Compute exact covariance of the process after raisimDt_ [Maybeck 4-114].
    double sigma_b_a_d = sqrt(-sigma_b_a * sigma_b_a * tau_a / 2.0 *
                              (exp(-2.0 * sampleTime_ / tau_a) - 1.0));
    // Compute state-transition.
    double phi_a_d = exp(-1.0 / tau_a * sampleTime_);
    // Simulate accelerometer noise processes 
    for (int i = 0; i < 3; ++i) {
        bias_[i] =
                phi_a_d * bias_[i] +
                sigma_b_a_d * standardNormalDistribution_(randomGenerator_);
        noise_(i) = bias_[i] +
                sigma_a_d * standardNormalDistribution_(randomGenerator_) +
                turnOnBias_[i];
    }
    return noise_;
  }

  odometryNoise::odometryNoise(double sampleTime, const Yaml::Node& cfg) : noise(sampleTime) {
    pos_std_ = cfg["pos_std"].template As<float>();
    orient_std_ = cfg["orient_std"].template As<float>();
    lin_vel_std_ = cfg["lin_vel_std"].template As<float>();
    ang_vel_std_ = cfg["ang_vel_std"].template As<float>();
  }

  Eigen::VectorXd odometryNoise::getNoise() {
    Eigen::Vector3d pos_noise(standardNormalDistribution_(randomGenerator_),
                              standardNormalDistribution_(randomGenerator_),
                              standardNormalDistribution_(randomGenerator_));
    pos_noise *= pos_std_;

    Eigen::Vector3d lin_vel_noise(standardNormalDistribution_(randomGenerator_),
                                  standardNormalDistribution_(randomGenerator_),
                                  standardNormalDistribution_(randomGenerator_));
    lin_vel_noise *= lin_vel_std_;

    Eigen::Vector3d ang_vel_noise(standardNormalDistribution_(randomGenerator_),
                                  standardNormalDistribution_(randomGenerator_),
                                  standardNormalDistribution_(randomGenerator_));
    ang_vel_noise *= ang_vel_std_;

    Eigen::Vector3d orient_noise_rot_vec(standardNormalDistribution_(randomGenerator_),
                                         standardNormalDistribution_(randomGenerator_),
                                         standardNormalDistribution_(randomGenerator_));
    Eigen::AngleAxisd orient_noise(standardNormalDistribution_(randomGenerator_) * orient_std_, orient_noise_rot_vec);
    Eigen::Quaterniond orient_noise_quat(orient_noise);
    orient_noise_quat.normalize();

    Eigen::VectorXd noise(13);
    noise << pos_noise, orient_noise_quat.coeffs(), lin_vel_noise, ang_vel_noise;
    return noise;
  }


    void imu::update() {
    accel_->update();

    gyro_->update();

    measure_.head(3) =  accel_->getMeas();
    measure_.tail(3) =   gyro_->getMeas();
    measureGT_.head(3) =accel_->getMeasGT();
    measureGT_.tail(3) = gyro_->getMeasGT();
  }

  void vicon::update() {
    Eigen::Vector3d pos;
    Eigen::Vector4d quat;
    raisim::Vec<4> robot_orientation;
    raisim::Vec<3> robot_pos_W;
  
    if (articulated_) {
      robot_->getPosition(baseLink_, Eigen::Vector3d(0,0,0), robot_pos_W);
      robot_->getBaseOrientation(robot_orientation);
    } else {
      robot_pos_W = obj_->getPosition();
      robot_orientation = obj_->getQuaternion();
    }

    measureGT_ << robot_pos_W.e(), robot_orientation.e();

    if(noiseSource_!=NULL) {
      measure_ = measureGT_ + noiseSource_->getNoise();
    } else {
      measure_ = measureGT_;
    }
  }  
  
  void odometry::update() {
    raisim::Vec<3> point_W, velocity_W, velocity_B, angularVelocity_W, angularVelocity_B;
    raisim::Vec<4> orient_W_wrong_order;
    raisim::Mat<3, 3>  orientMat;

    robot_->getPosition(baseLink_, Eigen::Vector3d(0,0,0), point_W);
    robot_->getBaseOrientation(orientMat);
    raisim::rotMatToQuat(orientMat,orient_W_wrong_order);
    // raisim quat as (w, x, y, z) to eigen quat with (x, y, z, w)
    Eigen::Vector4d orient_W(orient_W_wrong_order[1], orient_W_wrong_order[2], orient_W_wrong_order[3], orient_W_wrong_order[0]);
    robot_->getFrameVelocity(baseLink_, velocity_W);
    robot_->getFrameAngularVelocity(baseLink_, angularVelocity_W);

    velocity_B = orientMat.transpose() * velocity_W;
    angularVelocity_B = orientMat.transpose() * angularVelocity_W;
  
    measureGT_ << point_W.e(), orient_W, velocity_B.e(), angularVelocity_B.e();

    if(noiseSource_!=NULL) {
      measure_ = measureGT_ + noiseSource_->getNoise();
    } else {
      measure_ = measureGT_;
    }
  }

  void force::update() {

    size_t forceTipIdx = baseLink_; //robot_->getBodyIdx(parallelVer_+"/tool");
    Eigen::Vector3d contact_impulse_B;
    Eigen::Vector3d contact_torque_impulse_B;
    raisim::Mat<3, 3>  orientMat;
    contact_impulse_B.setZero();
    contact_torque_impulse_B.setZero();
    bool in_contact = false;

    //converting the tool force from world frame to the body frame. Since the sensor measures it in the body frame
    raisim::Mat<3,3> robot_orientation_rotmat;
    raisim::Vec<3> robot_position;
    robot_->getBaseOrientation(robot_orientation_rotmat);
    robot_->getBasePosition(robot_position);

    //get the ground truth from the simulator
    for (auto& contact: robot_->getContacts())
    {
        if (contact.skip()) continue;
        in_contact = true;
        contactMatProp_ = world_->getMaterialPairProperties(contact.getCollisionBodyA()->material, contact.getCollisionBodyB()->material);
        if (forceTipIdx == contact.getlocalBodyIndex()) {
          Eigen::Matrix3d rotMat_B = robot_orientation_rotmat.e().transpose()*contact.getContactFrame().e().transpose();
          Eigen::Vector3d pos_B = robot_position.e() - contact.getPosition().e();
          //Impulse transform from Impulse frame to Body frame
          contact_impulse_B += rotMat_B*contact.getImpulse().e();
          //Compute the torque from the force measurement. This is equivalent to (pos_B).cross(rotMat_B*contact.getImpulse().e()).
          contact_torque_impulse_B += Skew(pos_B)*rotMat_B*contact.getImpulse().e();
        }
    }

    Eigen::Vector3d tool_force_B = impulseFilter_.update(contact_impulse_B)/sampleTime_;
    Eigen::Vector3d interaction_torque_B = angularImpulseFilter_.update(contact_torque_impulse_B)/sampleTime_;

    measureGT_ << tool_force_B, interaction_torque_B;

    if(noiseSource_!=NULL) {
      measure_ = measureGT_ + noiseSource_->getNoise();
    } else {
      measure_ = measureGT_;
    }
  }

  void contact::update() {

    size_t forceTipIdx = baseLink_; //robot_->getBodyIdx(parallelVer_+"/tool");
    Eigen::Vector3d contact_impulse_W;
    raisim::Mat<3, 3>  orientMat;
    contact_impulse_W.setZero();
    bool in_contact = false;

    //get the ground truth from the simulator

    for (auto& contact: robot_->getContacts())
    {
        if (contact.skip()) continue;
        in_contact = true;
        contactMatProp_ = world_->getMaterialPairProperties(contact.getCollisionBodyA()->material, contact.getCollisionBodyB()->material);
        if (forceTipIdx == contact.getlocalBodyIndex()) {
            //Impulse in World frame
            contact_impulse_W += contact.getContactFrame().e().transpose()*contact.getImpulse().e();
        }
    }

    Eigen::Vector3d tool_force = impulseFilter_.update(contact_impulse_W)/sampleTime_;; //world frame

    double friction_coeff=0.0;
    if (tool_force.norm()>1e-3) {
        in_contact = true;
    }
    if (in_contact) {
        friction_coeff = contactMatProp_.c_f;
    }

    measureGT_ << tool_force, friction_coeff;

    if(noiseSource_!=NULL) {
      measure_ = measureGT_ + noiseSource_->getNoise();
    } else {
      measure_ = measureGT_;
    }
  }

}