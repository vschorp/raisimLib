/*
 *  Weixuan Zhang 21.07.2021
 *  Eugenio Cuniato 21.07.2021
 */

#include "raisim/World.hpp"
#include <boost/circular_buffer.hpp>
#include <Eigen/Core>

#include <deque>

#include "Yaml.hpp"

#ifndef RAISIM_SIMULATOR_SENSORS_
#define RAISIM_SIMULATOR_SENSORS_

/// Returns the 3D cross product Skew symmetric matrix of a given 3D vector.
template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> Skew(
    const Eigen::MatrixBase<Derived> & vec) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1],
      vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
}

class medianFilter {
  public:
    medianFilter(int size) : elements_(size) {
      size_ = size;
    }

    double update(double newVal) {
      elements_.push_back(newVal);
      if(elements_.size()<size_) return newVal;

      std::vector<double> sortedElements(size_);
      for(int i=0; i<size_; i++)
        sortedElements.push_back(elements_[i]);
      std::sort(sortedElements.begin(),sortedElements.end());
      return sortedElements[floor(size_/2)];  
    }

  private:
    boost::circular_buffer<double> elements_;
    int size_;
};

class medianFilterXd {
  public:
    medianFilterXd(int size, int vectorSize) {
      vectorSize_ = vectorSize;
      for(int i=0; i<vectorSize; i++)
        filters_.push_back(new medianFilter(size));
    }

    Eigen::VectorXd update(Eigen::VectorXd newVec) {
      Eigen::VectorXd filtVec(vectorSize_);

      for(int i=0; i<vectorSize_; i++)
        filtVec(i) = filters_[i]->update(newVec(i));

      return filtVec;
    }

  private:
    std::vector<medianFilter*> filters_; 
    int vectorSize_;
};

namespace raisim_sensors {

  enum sensorTypes {
    ACCELEROMETER,
    GYROSCOPE,
    IMU,
    VICON,
    ODOMETRY,
    FORCE,
    CONTACT
  };

  /**
  * \brief IMU noise parameters
  * \param[in] noise_density
  * \param[in] random_walk
  * \param[in] bias_correlation_time
  * \param[in] turnOnBias_sigma
  */
  struct imu_param {
    double noise_density;
    double random_walk;
    double bias_correlation_time;
    double turnOnBias_sigma;
  };

  /**
   * \class noise
   * \brief Base noise class
   */
  template <class T>
  class noise {
    public:
      /// \brief Noise constructor.
      /// \param[in] T noise vector type
      /// \param[in] sampleTime noise computation sample time
      noise(double sampleTime) : sampleTime_(sampleTime) {
        standardNormalDistribution_ = std::normal_distribution<double>(0.0, 1.0);
      };

      /// \brief Get sensor noise.
      /// \return the sensor noise
      virtual T getNoise() = 0;
    protected:
      double sampleTime_;
      T noise_;
      std::mt19937 randomGenerator_;
      std::normal_distribution<double> standardNormalDistribution_;
  };

  /**
   * \class imuNoise
   * \brief IMU noise class
   */
  class imuNoise : public noise<Eigen::Vector3d> {
    public:
       /// \brief IMU noise constructor.
      /// \param[in] sampleTime noise computation sample time
      /// \param[in] parameters IMU noise parameters
      imuNoise(double sampleTime, imu_param parameters);
      /// \brief Get sensor noise.
      /// \return the sensor noise
      Eigen::Vector3d getNoise();
    private:
      imu_param params_;
      Eigen::Vector3d bias_, turnOnBias_;
  };

  /**
   * \class virtualSensor
   * \brief Virtual class to collect all the sensors together
   */
  class virtualSensor {
    public:
      /// \brief virtual sensor constructor.
      virtualSensor() {};
      /// \brief update sensor value.
      virtual void update() = 0;
      /// \brief get sensor type.
      /// \return sensor type
      virtual sensorTypes getType() = 0;
      /// \brief Update sample time.
      /// \param[in] sampleTime sample time to set in seconds
      virtual void updateSampleTime(const double sampleTime) = 0;
  };

  /**
  * \class sensor
  * \brief Base sensor class
  * \param[in] T sensor vector type
  * \param[in] msg message to be published type
  */
  template <class T>
  class sensor : public virtualSensor{
    public:
      /// \brief base sensor constructor.
      /// \param[in] sampleTime sample time of the simulated world
      /// \param[in] noise_source noise source for the measured quantity (NULL if no noise)
      sensor( double sampleTime, noise<T> * noise_source = NULL)
      : sampleTime_(sampleTime), noiseSource_(noise_source) {
        msgReady_ = false;
      };

      /// \brief update sensor value.
      virtual void update() = 0;
      /// \brief get sensor type.
      /// \return sensor type
      virtual sensorTypes getType() = 0;

      /// \brief Update sample time.
      /// \param[in] sampleTime sample time to set in seconds
      void updateSampleTime(const double sampleTime) {sampleTime_=sampleTime;};

      /// \brief get noisy measurement.
      /// \return the measurement
      T getMeas() {return measure_;};
      /// \brief get ground truth measurement.
      /// \return the measurement
      T getMeasGT() {return measureGT_;};

    protected:
      /// \brief attach sensor to the link.
      /// \param[in] sensor_link_name link name
      size_t attachToLink(std::string sensor_link_name) {
        size_t base_link;

        base_link = robot_->getFrameIdxByName(sensor_link_name);
        if(base_link!=-1) return base_link;

        //Else, let's try with _joint instead of _link
        std::string link = "link";
        const std::string joint = "joint";
        size_t pos = sensor_link_name.find(link);
        if(pos!=std::string::npos) {
          sensor_link_name.replace(pos,joint.length(),joint);
          base_link = robot_->getFrameByName(sensor_link_name).currentBodyId;
          if(base_link!=-1) {
            return base_link;
          } 
        }
        base_link = 0;
        return base_link;
      };
      noise<T> * noiseSource_;
      double sampleTime_;
      T measure_;
      T measureGT_;
      bool msgReady_, updatePublish_;
      size_t baseLink_;
      raisim::ArticulatedSystem * robot_;
  };


  /**
   * \class accelerometer
   * \brief Accelerometer sensor class
  */

  class accelerometer : public sensor<Eigen::Vector3d> {
    public:
      /// \brief accelerometer sensor constructor.
      /// \param[in] robot robot where to place the accelerometer
      /// \param[in] sampleTime sample time of the simulated world
      /// \param[in] sensor_link_name URDF link name where to place the sensor
      /// \param[in] imu_wrt_link_offset offset of the IMU w.r.t. the link frame origin
      /// \param[in] world pointer to the simulated world
      /// \param[in] noise_source noise source for the measured quantity (NULL if no noise)
      accelerometer(raisim::ArticulatedSystem * robot, double sampleTime, const std::string sensor_link_name, Eigen::Vector3d imu_wrt_link_offset, Eigen::Vector3d gravity, noise<Eigen::Vector3d> * noise_source = NULL)
      : sensor( sampleTime, noise_source), imu_wrt_link_offset_(imu_wrt_link_offset), gravity_(gravity) {
        robot_ = robot;
        baseLink_ = attachToLink(sensor_link_name);
        prev_robot_linear_velocity_W_.setZero();
        };

      /// \brief update sensor value.
      void update();
      /// \brief get sensor type.
      /// \return sensor type
      sensorTypes getType() {return sensorTypes::ACCELEROMETER;};

    private:
      Eigen::Vector3d gravity_;
      Eigen::Vector3d imu_wrt_link_offset_;
      raisim::Vec<3> prev_robot_linear_velocity_W_, prev_imu_linear_velocity_W_;
  };

  /**
   * \class gyroscope
  * \brief Gyroscope sensor class
  */
  class gyroscope : public sensor<Eigen::Vector3d> {
    public:
      /// \brief gyroscope sensor constructor.
      /// \param[in] robot robot where to place the accelerometer
      /// \param[in] sampleTime sample time of the simulated world
      /// \param[in] sensor_link_name URDF link name where to place the sensor
      /// \param[in] noise_source noise source for the measured quantity (NULL if no noise)
      gyroscope(raisim::ArticulatedSystem * robot, double sampleTime, const std::string sensor_link_name, noise<Eigen::Vector3d> * noise_source = NULL)
      : sensor(sampleTime, noise_source) {
        robot_ = robot;
        baseLink_ = attachToLink(sensor_link_name);
        };
    
      /// \brief update sensor value.
      void update();
      /// \brief get sensor type.
      /// \return sensor type
      sensorTypes getType() {return sensorTypes::GYROSCOPE;};
  };

  /**
  * \class imu
  * \brief IMU sensor class
  */
  class imu : public sensor<Eigen::VectorXd> {
    public:
      /// \brief imu sensor constructor.
      /// \param[in] robot robot where to place the accelerometer
      /// \param[in] sampleTime sample time of the simulated world
      /// \param[in] sensor_link_name URDF link name where to place the sensor
      /// \param[in] imu_wrt_link_offset offset of the IMU w.r.t. the link frame origin
      /// \param[in] world pointer to the simulated world
      /// \param[in] accel_noise noise source for the accelerometer quantity (NULL if no noise)
      /// \param[in] gyro_noise noise source for the gyroscope quantity (NULL if no noise)
      imu(raisim::ArticulatedSystem * robot, double sampleTime, const std::string sensor_link_name, Eigen::Vector3d imu_wrt_link_offset, Eigen::Vector3d gravity, noise<Eigen::Vector3d> * accel_noise = NULL, noise<Eigen::Vector3d> * gyro_noise = NULL)
      : sensor(sampleTime, NULL) {
          accel_ = new accelerometer(robot, sampleTime, sensor_link_name, imu_wrt_link_offset, gravity, accel_noise);
          gyro_  = new gyroscope(robot, sampleTime, sensor_link_name, gyro_noise);  
          measureGT_.resize(6);
          measure_.resize(6);
          measure_.setZero();
          measureGT_.setZero();
        };

      /// \brief imu sensor constructor.
      /// \param[in] robot robot where to place the accelerometer
      /// \param[in] sampleTime sample time of the simulated world
      /// \param[in] imu_wrt_link_offset offset of the IMU w.r.t. the link frame origin
      /// \param[in] world pointer to the simulated world
      /// \param[in] params IMU parameters
      imu(raisim::ArticulatedSystem * robot, double sampleTime, Eigen::Vector3d imu_wrt_link_offset, Eigen::Vector3d gravity, const Yaml::Node& cfg)
      : sensor(sampleTime, NULL) {
        raisim_sensors::imu_param accel_params, gyro_params;
        accel_params.bias_correlation_time  = cfg["accelerometerBiasCorrelationTime"].template As<float>();
        accel_params.turnOnBias_sigma       = cfg["accelerometerTurnOnBiasSigma"].template As<float>();
        accel_params.noise_density          = cfg["accelerometerNoiseDensity"].template As<float>();
        accel_params.random_walk            = cfg["accelerometerRandomWalk"].template As<float>();
        gyro_params.bias_correlation_time   = cfg["gyroscopeBiasCorrelationTime"].template As<float>();
        gyro_params.turnOnBias_sigma        = cfg["gyroscopeTurnOnBiasSigma"].template As<float>();
        gyro_params.noise_density           = cfg["gyroscopeNoiseDensity"].template As<float>();
        gyro_params.random_walk             = cfg["gyroscopeRandomWalk"].template As<float>();

        std::string link_name = cfg["linkName"].template As<std::string>();
        std::string sensorNamespace = cfg["robotNamespace"].template As<std::string>();
        imuNoise *accelNoise = new raisim_sensors::imuNoise(sampleTime,accel_params);
        imuNoise *gyroNoise = new raisim_sensors::imuNoise(sampleTime,gyro_params);

        accel_ = new accelerometer(robot, sampleTime, link_name, imu_wrt_link_offset, gravity, accelNoise);
        gyro_  = new gyroscope(robot, sampleTime, link_name, gyroNoise);  
        measureGT_.resize(6);
        measure_.resize(6);
        measure_.setZero();
        measureGT_.setZero();
      };
      imu() : sensor (0.0, NULL) { }
    
      /// \brief update sensor value.
      void update();
      /// \brief get sensor type.
      /// \return sensor type
      sensorTypes getType() {return sensorTypes::IMU;};

    private:
      gyroscope *gyro_;
      accelerometer *accel_;
  };

  /**
   * \class vicon
  * \brief Vicon sensor class
  */
  class vicon : public sensor<Eigen::VectorXd> {
    public:
      /// \brief vicon sensor constructor.
      ///\param[in] robot robot where to place the sensor
      ///\param[in] sampleTime sample time of the simulated world
      ///\param[in] child_frame_name name of the frame for the published transform
      ///\param[in] sensor_link_name URDF link name where to place the sensor
      ///\param[in] noise_source noise source for the measured quantity (NULL if no noise)
      vicon(raisim::ArticulatedSystem * robot, double sampleTime, const std::string child_frame_name, const std::string sensor_link_name , noise<Eigen::VectorXd> * noise_source = NULL)
      : sensor(sampleTime, noise_source) {
        robot_ = robot;
        baseLink_ = attachToLink(sensor_link_name);
        childFrameName_ = child_frame_name;
        articulated_ = true;
        measureGT_.resize(7);
        measure_.resize(7);
        measure_.setZero();
        measureGT_.setZero();
      };

      /// \brief vicon sensor constructor.
      ///\param[in] robot robot where to place the sensor
      ///\param[in] sampleTime sample time of the simulated world
      ///\param[in] params Vicon sensor parameters
      vicon(raisim::ArticulatedSystem * robot, double sampleTime, const Yaml::Node& cfg)
      : sensor(sampleTime, NULL) {
        std::string sensor_link_name = cfg["linkName"].template As<std::string>();
        childFrameName_ = cfg["childFrameId"].template As<std::string>();
        robot_ = robot;
        baseLink_ = attachToLink(sensor_link_name);
        articulated_ = true;
        measureGT_.resize(7);
        measure_.resize(7);
        measure_.setZero();
        measureGT_.setZero();
      };
      
      /// \brief vicon sensor constructor.
      ///\param[in] obj Single object where to place the sensor
      ///\param[in] sampleTime sample time of the simulated world
      ///\param[in] child_frame_name name of the frame for the published transform
      ///\param[in] sensor_link_name URDF link name where to place the sensor
      ///\param[in] noise_source noise source for the measured quantity (NULL if no noise)
      vicon(raisim::SingleBodyObject * obj, double sampleTime, const std::string child_frame_name, noise<Eigen::VectorXd> * noise_source = NULL)
      : sensor(sampleTime, noise_source) {
        childFrameName_ = child_frame_name;
        obj_ = obj;
        articulated_ = false;
        measureGT_.resize(7);
        measure_.resize(7);
        measure_.setZero();
        measureGT_.setZero();
      };

      /// \brief vicon sensor constructor.
      ///\param[in] obj Single object where to place the sensor
      ///\param[in] sampleTime sample time of the simulated world
      ///\param[in] params Vicon sensor parameters
      vicon(raisim::SingleBodyObject * obj, double sampleTime, const Yaml::Node& cfg)
      : sensor(sampleTime, NULL) {
        childFrameName_ = cfg["childFrameId"].template As<std::string>();
        obj_ = obj;
        articulated_ = false;
        measureGT_.resize(7);
        measure_.resize(7);
        measure_.setZero();
        measureGT_.setZero();
      };
    
      /// \brief update sensor value.
      void update();
      /// \brief get sensor type.
      /// \return sensor type
      sensorTypes getType() {return sensorTypes::VICON;};

    private:
      raisim::SingleBodyObject * obj_;
      std::string childFrameName_;
      bool articulated_;
  };

  /**
   * \class odometry
  * \brief Odometry sensor class
  */
  class odometry : public sensor<Eigen::VectorXd> {
    public:
      /// \brief odometry sensor constructor.
      /// \param[in] robot robot where to place the sensor
      /// \param[in] sampleTime sample time of the simulated world
      /// \param[in] child_frame_name name of the frame for the published transform
      /// \param[in] sensor_link_name URDF link name where to place the sensor
      /// \param[in] noise_source noise source for the measured quantity (NULL if no noise)
      odometry(raisim::ArticulatedSystem *robot, double sampleTime, const std::string child_frame_name, const std::string sensor_link_name , noise<Eigen::VectorXd> * noise_source = NULL)
      : sensor(sampleTime, noise_source) {
        robot_ = robot;
        baseLink_ = attachToLink(sensor_link_name);
        childFrameName_ = child_frame_name;
        measureGT_.resize(7+6);
        measure_.resize(7+6);
        measure_.setZero();
        measureGT_.setZero();
      };

      /// \brief odometry sensor constructor.
      /// \param[in] robot robot where to place the sensor
      /// \param[in] sampleTime sample time of the simulated world
      /// \param[in] params Odometry sensor parameters
      odometry(raisim::ArticulatedSystem *robot, double sampleTime, const Yaml::Node& cfg)
      : sensor(sampleTime, NULL) {
        robot_ = robot;
        std::string sensor_link_name = cfg["linkName"].template As<std::string>();
        childFrameName_ = cfg["childFrameId"].template As<std::string>();
        baseLink_ = attachToLink(sensor_link_name);
        measureGT_.resize(7+6);
        measure_.resize(7+6);
        measure_.setZero();
        measureGT_.setZero();
      };

      odometry() : sensor(0.0, NULL) { }
    
      /// \brief update sensor value.
      void update();
      /// \brief get sensor type.
      /// \return sensor type
      sensorTypes getType() {return sensorTypes::ODOMETRY;};

    private:
      std::string childFrameName_;
  };

  /**
  * \class force
  * \brief Force sensor class
  */
  class force : public sensor<Eigen::VectorXd> {
    public:
      /// \brief force sensor constructor.
      /// \param[in] robot robot where to place the force sensor
      /// \param[in] world pointer to the simulated world
      /// \param[in] sampleTime sample time of the simulated world
      /// \param[in] sensor_link_name URDF link name where to place the sensor
      /// \param[in] child_frame_name name of the frame for the published transform
      /// \param[in] noise_source noise source for the measured quantity (NULL if no noise)
      force(raisim::ArticulatedSystem *robot, raisim::World *world, double sampleTime, const std::string sensor_link_name, const std::string child_frame_name, noise<Eigen::VectorXd> * noise_source = NULL)
      : sensor(sampleTime, noise_source), impulseFilter_(3,3),angularImpulseFilter_(3,3) {
        childFrameName_ = child_frame_name;
        robot_ = robot;
        world_ = world;
        baseLink_ = attachToLink(sensor_link_name);
        measureGT_.resize(6);
        measure_.resize(6);
        measure_.setZero();
        measureGT_.setZero();
      };

      /// \brief force sensor constructor.
      /// \param[in] robot robot where to place the force sensor
      /// \param[in] world pointer to the simulated world
      /// \param[in] sampleTime sample time of the simulated world
      /// \param[in] child_frame_name name of the frame for the published transform
      /// \param[in] params force sensor params
      force(raisim::ArticulatedSystem *robot, raisim::World *world, double sampleTime, const std::string child_frame_name, const Yaml::Node& cfg)
      : sensor(sampleTime, NULL), impulseFilter_(3,3),angularImpulseFilter_(3,3) {
        std::string sensor_link_name = cfg["linkName"].template As<std::string>();
        childFrameName_ = child_frame_name;
        robot_ = robot;
        world_ = world;
        baseLink_ = attachToLink(sensor_link_name);
        measureGT_.resize(6);
        measure_.resize(6);
        measure_.setZero();
        measureGT_.setZero();
      };
    
      /// \brief update sensor value.
      void update();
      /// \brief get sensor type.
      /// \return sensor type
      sensorTypes getType() {return sensorTypes::FORCE;};

    private:
      raisim::World * world_;
      raisim::MaterialPairProperties contactMatProp_;
      medianFilterXd impulseFilter_;
      medianFilterXd angularImpulseFilter_;
      std::string childFrameName_;
  };

  /**
   * \class contact
  * \brief Contact sensor class. Gives info on the contact friction.
  */
  class contact : public sensor<Eigen::VectorXd> {
    public:
      /// \brief contact sensor constructor.
      /// \param[in] robot robot where to place the force sensor
      /// \param[in] world pointer to the simulated world
      /// \param[in] sampleTime sample time of the simulated world
      /// \param[in] sensor_link_name URDF link name where to place the sensor
      /// \param[in] child_frame_name name of the frame for the published transform
      /// \param[in] noise_source noise source for the measured quantity (NULL if no noise)
      contact(raisim::ArticulatedSystem *robot, raisim::World *world, double sampleTime, const std::string sensor_link_name, const std::string child_frame_name, noise<Eigen::VectorXd> * noise_source = NULL)
      : sensor(sampleTime, noise_source), impulseFilter_(3,3) {
        childFrameName_ = child_frame_name;
        robot_ = robot;
        world_ = world;
        baseLink_ = attachToLink(sensor_link_name);
        measureGT_.resize(4);
        measure_.resize(4);
        measure_.setZero();
        measureGT_.setZero();
      };

      /// \brief contact sensor constructor.
      /// \param[in] robot robot where to place the force sensor
      /// \param[in] world pointer to the simulated world
      /// \param[in] sampleTime sample time of the simulated world
      /// \param[in] child_frame_name name of the frame for the published transform
      /// \param[in] params contact sensor parameters
      contact(raisim::ArticulatedSystem *robot, raisim::World *world, double sampleTime, const std::string child_frame_name, const Yaml::Node& cfg)
      : sensor(sampleTime, NULL), impulseFilter_(3,3) {
        std::string sensor_link_name = cfg["linkName"].template As<std::string>();
        childFrameName_ = child_frame_name;
        robot_ = robot;
        world_ = world;
        baseLink_ = attachToLink(sensor_link_name);
        measureGT_.resize(4);
        measure_.resize(4);
        measure_.setZero();
        measureGT_.setZero();
      };
    
      /// \brief update sensor value.
      void update();
      /// \brief get sensor type.
      /// \return sensor type
      sensorTypes getType() {return sensorTypes::CONTACT;};

    private:
      raisim::World * world_;
      raisim::MaterialPairProperties contactMatProp_;
      medianFilterXd impulseFilter_;
      std::string childFrameName_;
  };



}

#endif // !RAISIM_SIMULATOR_SENSORS_