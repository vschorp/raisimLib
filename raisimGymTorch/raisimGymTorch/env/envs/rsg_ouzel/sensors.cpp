/*
 *  Weixuan Zhang 21.07.2021
 *  Eugenio Cuniato 21.07.2021
 */

#include "ros_raisim_interface/sensors.h"

namespace raisim_sensors {

  void accelToImuMsg(const Eigen::Vector3d accel, sensor_msgs::Imu& msg) {
    msg.header.stamp = ros::Time::now();
    
    msg.orientation.x = 0;
    msg.orientation.y = 0;
    msg.orientation.z = 0;
    msg.orientation.w = 0;
    msg.orientation_covariance[0] = -1;

    msg.linear_acceleration.x = accel(0);
    msg.linear_acceleration.y = accel(1);
    msg.linear_acceleration.z = accel(2);
    for (int i = 0; i<9; i++){
      msg.orientation_covariance[i] = -1.;
      msg.linear_acceleration_covariance[i] = 0.0;
    }
  }

  void gyroToImuMsg(const Eigen::Vector3d gyro, sensor_msgs::Imu& msg) {
    msg.header.stamp = ros::Time::now();
    
    msg.orientation.x = 0;
    msg.orientation.y = 0;
    msg.orientation.z = 0;
    msg.orientation.w = 0;

    msg.angular_velocity.x = gyro(0);
    msg.angular_velocity.y = gyro(1);
    msg.angular_velocity.z = gyro(2);
    for (int i = 0; i<9; i++){
      msg.orientation_covariance[i] = -1.;
      msg.angular_velocity_covariance[i] = 0.0;
    }
  }

  void poseToTransformMsg(const Eigen::VectorXd meas, const std::string child_frame, geometry_msgs::TransformStamped& msg) {
    Eigen::Vector3d pos = meas.head(3);
    Eigen::Vector4d quat = meas.tail(4);
    msg.header.stamp = ros::Time::now();
    
    msg.header.frame_id = "world";
    msg.child_frame_id = child_frame;
    msg.transform.translation.x = pos[0];
    msg.transform.translation.y = pos[1];
    msg.transform.translation.z = pos[2];
    msg.transform.rotation.w = quat[0];
    msg.transform.rotation.x = quat[1];
    msg.transform.rotation.y = quat[2];
    msg.transform.rotation.z = quat[3];
  }

  void forceToWrenchMsg(const Eigen::VectorXd meas, const std::string child_frame, geometry_msgs::WrenchStamped& msg) {
    msg.header.stamp = ros::Time::now();
    
    msg.header.frame_id = child_frame;
    msg.wrench.force.x = meas[0];
    msg.wrench.force.y = meas[1];
    msg.wrench.force.z = meas[2];
    msg.wrench.torque.x = meas[3];
    msg.wrench.torque.y = meas[4];
    msg.wrench.torque.z = meas[5];
  }

  void forceToContactMsg(const Eigen::VectorXd meas, const std::string child_frame, const bool in_contact, ros_raisim_interface::ContactInformation& msg) {
    msg.header.stamp = ros::Time::now();
    
    msg.header.frame_id = child_frame;
    msg.contact_force.x = meas[0];
    msg.contact_force.y = meas[1];
    msg.contact_force.z = meas[2];
    msg.in_contact = in_contact;
    msg.friction_coefficient = meas[4];
  }

  void odomToOdomMsg(const Eigen::VectorXd meas, const std::string child_frame, nav_msgs::Odometry& msg) {
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.child_frame_id = child_frame;
    msg.pose.pose.position.x = meas[0];
    msg.pose.pose.position.y = meas[1];
    msg.pose.pose.position.z = meas[2];
    msg.pose.pose.orientation.w = meas[3];
    msg.pose.pose.orientation.x = meas[4];
    msg.pose.pose.orientation.y = meas[5];
    msg.pose.pose.orientation.z = meas[6];
    msg.pose.covariance.fill(0.0);
    msg.twist.twist.linear.x = meas[7];
    msg.twist.twist.linear.y = meas[8];
    msg.twist.twist.linear.z = meas[9];
    msg.twist.twist.angular.x = meas[10];
    msg.twist.twist.angular.y = meas[11];
    msg.twist.twist.angular.z = meas[12];
    msg.twist.covariance.fill(0.0);
  }

  void accelerometer::update() {
    raisim::Vec<3> robot_linear_velocity_W, imu_linear_velocity_W;
    raisim::Mat<3,3> robot_orientation_rotmat;

    //  emulate the imu signals
    robot_->getVelocity(baseLink_, robot_linear_velocity_W);
    robot_->getVelocity(baseLink_, imu_wrt_link_offset_ , imu_linear_velocity_W);
    robot_->getBaseOrientation(robot_orientation_rotmat);

    measureGT_ = robot_orientation_rotmat.e().transpose()*((imu_linear_velocity_W.e() - prev_imu_linear_velocity_W_.e())/sampleTime_ - world_->getGravity().e());

    prev_robot_linear_velocity_W_ = robot_linear_velocity_W;
    prev_imu_linear_velocity_W_ = imu_linear_velocity_W;

    if(noiseSource_!=NULL) {
      measure_ = measureGT_ + noiseSource_->getNoise();
    } else {
      measure_ = measureGT_;
    }

    accelToImuMsg(measure_,measMsg_);
    accelToImuMsg(measureGT_,measGTMsg_);
    msgReady_ = true;
    if(updatePublish_)
      publish();

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

    gyroToImuMsg(measure_,measMsg_);
    gyroToImuMsg(measureGT_,measGTMsg_);

    msgReady_ = true;
    if(updatePublish_)
      publish();

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

  void imu::update() {
    accel_->update();

    gyro_->update();

    measure_.head(3) =  accel_->getMeas();
    measure_.tail(3) =   gyro_->getMeas();
    measureGT_.head(3) =accel_->getMeasGT();
    measureGT_.tail(3) = gyro_->getMeasGT();

    accelToImuMsg(measure_.head(3),measMsg_);
    gyroToImuMsg(measure_.tail(3),measMsg_);
    accelToImuMsg(measureGT_.head(3),measGTMsg_);
    gyroToImuMsg(measureGT_.tail(3),measGTMsg_);

    msgReady_ = true;
    if(updatePublish_)
      publish();
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

    poseToTransformMsg(measure_, childFrameName_, measMsg_);
    poseToTransformMsg(measureGT_, childFrameName_, measGTMsg_);

    msgReady_ = true;
    if(updatePublish_)
      publish();

  }  
  
  void odometry::update() {
    raisim::Vec<3> point_W, velocity_W, velocity_B, angularVelocity_W, angularVelocity_B;
    raisim::Vec<4> orient_W;
    raisim::Mat<3, 3>  orientMat;

    robot_->getFramePosition(baseLink_, point_W);
    robot_->getFrameOrientation(baseLink_, orientMat);
    raisim::rotMatToQuat(orientMat,orient_W);
    robot_->getFrameVelocity(baseLink_, velocity_W);
    robot_->getFrameAngularVelocity(baseLink_, angularVelocity_W);

    velocity_B = orientMat.e().transpose()*velocity_W.e();
    angularVelocity_B = orientMat.e().transpose()*angularVelocity_W.e();
  
    measureGT_ << point_W.e(), orient_W.e(), velocity_B.e(), angularVelocity_B.e();

    if(noiseSource_!=NULL) {
      measure_ = measureGT_ + noiseSource_->getNoise();
    } else {
      measure_ = measureGT_;
    }

    odomToOdomMsg(measure_, childFrameName_, measMsg_);
    odomToOdomMsg(measureGT_, childFrameName_, measGTMsg_);

    msgReady_ = true;
    if(updatePublish_)
      publish();

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

    forceToWrenchMsg(measure_, childFrameName_, measMsg_);
    forceToWrenchMsg(measureGT_, childFrameName_, measGTMsg_);

    msgReady_ = true;
    if(updatePublish_)
      publish();

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

    forceToContactMsg(measure_, childFrameName_, in_contact, measMsg_);
    forceToContactMsg(measureGT_, childFrameName_, in_contact, measGTMsg_);

    msgReady_ = true;
    if(updatePublish_)
      publish();

  }

}