/*
 *  Weixuan Zhang 21.07.2021
 *  Eugenio Cuniato 21.07.2021
 */

#include "ros/ros.h"
#include "raisim/World.hpp"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/WrenchStamped.h"
#include "ros_raisim_interface/ContactInformation.h"
#include <boost/circular_buffer.hpp>
#include "ros_raisim_interface/parser.h"
#include <Eigen/Core>

#include <deque>

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
      std::default_random_engine randomGenerator_;
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
   * \class sensorPublisher
   * \brief Sensor publisher class
   * \param[in] T type of the measured quantity (usually a Eigen::VectorXd or similiar)
   */
  template <class T>
  class sensorPublisher {
    public:
      /// \brief sensorPublisher constructor.
      /// \param[in] n ros node handle
      /// \param[in] topic_name name of the topic to publish
      /// \param[in] rate_hz publishing rate (-1 means publish on update)
      /// \param[in] publish_gt true if ground truth has to be published
      sensorPublisher(ros::NodeHandle* n=NULL, std::string topic_name="", double rate_hz = -1, bool publish_gt=false)
      : n_(n), rateHz_(rate_hz), topicName_(topic_name) {
        init_ = false;
        if(n!=NULL) {
          pub_ = n_->advertise<T>(topic_name, 1);
          if (publishGT_)
            pubGT_ = n_->advertise<T>("ground_truth/"+topic_name, 1);
          init_ = true;
        }
        measDelay_ = 0;
        publishGT_ = publish_gt;
      };

      /// \brief message publisher.
      /// \param[in] value noisy message to publish
      /// \param[in] value ground thruth message to publish
      void publish(T value, T value_gt) {
        if(measDelay_< 0) {
          pub_.publish(value);
        } else {
          double currentWorldTime = ros::Time::now().toSec();
          msgQueue_.push_back(std::make_pair(currentWorldTime+measDelay_, value));
          if (currentWorldTime > msgQueue_.front().first ){
            T delayedMsg;
            delayedMsg = msgQueue_.front().second;
            msgQueue_.pop_front();
            pub_.publish(delayedMsg);
          }
        }

        if(publishGT_) {
          pubGT_.publish(value_gt);
        }
      }

      /// \brief publisher init.
      /// \param[in] n ros node handle
      /// \param[in] topic_name name of the topic to publish
      /// \param[in] rate_hz publishing rate (-1 means publish on update)
      /// \param[in] publish_gt true if ground truth has to be published
      void init(ros::NodeHandle* n, std::string topic_name, double rate_hz = -1, bool publish_gt=false) {
        
        if(n!=NULL) {
          n_ = n;
          pub_ = n_->advertise<T>(topic_name, 1000);
          pubGT_ = n_->advertise<T>("ground_truth/"+topic_name, 1);
          init_ = true;
          publishGT_ = publish_gt;
          rateHz_ = rate_hz;
        }else {
          ROS_ERROR("Node handle not valid. Publisher not initialized");
        }
      }

      /// \brief get publisher rate.
      /// \return publishing rate
      double getHz() {return rateHz_;};
      /// \brief is node initialized.
      /// \return true if initialized
      bool isInit() {return init_;};
      /// \brief Set sensor meas delay.
      /// \param[in] delay delay to set in seconds
      void setMeasDelay(double delay) {measDelay_=delay;};
      /// \brief get the node handle.
      /// \return the node handle
      ros::NodeHandle* getHandle() {return n_;};

    protected:
      std::string topicName_;
      bool publishGT_;
      ros::NodeHandle* n_;
      ros::Publisher pub_, pubGT_;
      double rateHz_, measDelay_;
      bool init_;
      std::deque<std::pair<double, T>> msgQueue_;
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
      /// \brief start the publisher.
      /// \return true if all went right
      virtual bool startPublishing() = 0;
      /// \brief set measurement delay.
      /// \param[in] delay delay to set in seconds
      virtual void setMeasDelay(double delay) = 0;
      /// \brief Update sample time.
      /// \param[in] sampleTime sample time to set in seconds
      virtual void updateSampleTime(const double sampleTime) = 0;
      /// \brief publisher init.
      /// \param[in] n ros node handle
      /// \param[in] topic_name name of the topic to publish
      /// \param[in] rate_hz publishing rate (-1 means publish on update)
      /// \param[in] publish_gt true if ground truth has to be published
      virtual void initPublisher(ros::NodeHandle* n, const std::string topic_name, double rate_hz = -1, bool publish_gt=false) = 0;
  };

  /**
  * \class sensor
  * \brief Base sensor class
  * \param[in] T sensor vector type
  * \param[in] msg message to be published type
  */
  template <class T, class msg>
  class sensor : public virtualSensor{
    public:
      /// \brief base sensor constructor.
      /// \param[in] sampleTime sample time of the simulated world
      /// \param[in] noise_source noise source for the measured quantity (NULL if no noise)
      sensor( double sampleTime, noise<T> * noise_source = NULL)
      : sampleTime_(sampleTime), noiseSource_(noise_source) {
        msgReady_ = false;
        updatePublish_ = false;
      };

      /// \brief update sensor value.
      virtual void update() = 0;
      /// \brief get sensor type.
      /// \return sensor type
      virtual sensorTypes getType() = 0;
      
      /// \brief start the publisher.
      /// \return true if all went right
      bool startPublishing() {
        if (!publisher.isInit())
          return false;
        if(publisher.getHz()<=0)
          updatePublish_ = true; //publish on update
        else {
          // Create a ROS timer for timed publishing
          updatePublish_ = false;
          timerPublish_ =
            publisher.getHandle()->createTimer(ros::Duration(1.0/publisher.getHz()),
                          std::bind(&sensor<T,msg>::publish, this));
        }
        return true; 
      }
      /// \brief Update sample time.
      /// \param[in] sampleTime sample time to set in seconds
      void updateSampleTime(const double sampleTime) {sampleTime_=sampleTime;};
      /// \brief publisher init.
      /// \param[in] n ros node handle
      /// \param[in] topic_name name of the topic to publish
      /// \param[in] rate_hz publishing rate (-1 means publish on update)
      /// \param[in] publish_gt true if ground truth has to be published
      void initPublisher(ros::NodeHandle* n, const std::string topic_name, double rate_hz = -1, bool publish_gt=false) {
        publisher.init(n, topic_name,rate_hz,publish_gt);
      }
      /// \brief set measurement delay.
      /// \param[in] delay delay to set in seconds
      void setMeasDelay(double delay) {publisher.setMeasDelay(delay);};
      /// \brief get noisy measurement.
      /// \return the measurement
      T getMeas() {return measure_;};
      /// \brief get ground truth measurement.
      /// \return the measurement
      T getMeasGT() {return measureGT_;};

    protected: 
      /// \brief send messages to the publisher.
      void publish() {
        if(msgReady_)
          publisher.publish(measMsg_,measGTMsg_);
      }

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
          //ROS_WARN_STREAM("Could not find sensor link "<<sensor_link_name);
          sensor_link_name.replace(pos,joint.length(),joint);
          //ROS_WARN_STREAM("Trying with a joint "<<sensor_link_name);
          //ROS_WARN_STREAM("Got link: "<<robot_->getFrameByName(sensor_link_name).currentBodyId);
          base_link = robot_->getFrameByName(sensor_link_name).currentBodyId;
          if(base_link!=-1) {
            ROS_INFO_STREAM("Sensor attached to link: "<<robot_->getFrameByName(sensor_link_name).bodyName);
            return base_link;
          } 
        }
        ROS_WARN_STREAM("Could not find sensor link "<<sensor_link_name<<". Attaching to ROOT link.");
        base_link = 0;
        return base_link;
      };
      noise<T> * noiseSource_;
      double sampleTime_;
      T measure_;
      T measureGT_;
      msg measMsg_;
      msg measGTMsg_;
      bool msgReady_, updatePublish_;
      sensorPublisher<msg> publisher;
      ros::Timer timerPublish_;
      size_t baseLink_;
      raisim::ArticulatedSystem * robot_;
  };


  /**
   * \class accelerometer
   * \brief Accelerometer sensor class
  */

  class accelerometer : public sensor<Eigen::Vector3d, sensor_msgs::Imu> {
    public:
      /// \brief accelerometer sensor constructor.
      /// \param[in] robot robot where to place the accelerometer
      /// \param[in] sampleTime sample time of the simulated world
      /// \param[in] sensor_link_name URDF link name where to place the sensor
      /// \param[in] imu_wrt_link_offset offset of the IMU w.r.t. the link frame origin
      /// \param[in] world pointer to the simulated world
      /// \param[in] noise_source noise source for the measured quantity (NULL if no noise)
      accelerometer(raisim::ArticulatedSystem * robot, double sampleTime, const std::string sensor_link_name, Eigen::Vector3d imu_wrt_link_offset, raisim::World *world, noise<Eigen::Vector3d> * noise_source = NULL)
      : sensor( sampleTime, noise_source), imu_wrt_link_offset_(imu_wrt_link_offset), world_(world) {
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
      raisim::World * world_;
      Eigen::Vector3d imu_wrt_link_offset_;
      raisim::Vec<3> prev_robot_linear_velocity_W_, prev_imu_linear_velocity_W_;
  };

  /**
   * \class gyroscope
  * \brief Gyroscope sensor class
  */
  class gyroscope : public sensor<Eigen::Vector3d, sensor_msgs::Imu> {
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
  class imu : public sensor<Eigen::VectorXd, sensor_msgs::Imu> {
    public:
      /// \brief imu sensor constructor.
      /// \param[in] robot robot where to place the accelerometer
      /// \param[in] sampleTime sample time of the simulated world
      /// \param[in] sensor_link_name URDF link name where to place the sensor
      /// \param[in] imu_wrt_link_offset offset of the IMU w.r.t. the link frame origin
      /// \param[in] world pointer to the simulated world
      /// \param[in] accel_noise noise source for the accelerometer quantity (NULL if no noise)
      /// \param[in] gyro_noise noise source for the gyroscope quantity (NULL if no noise)
      imu(raisim::ArticulatedSystem * robot, double sampleTime, const std::string sensor_link_name, Eigen::Vector3d imu_wrt_link_offset, raisim::World *world, noise<Eigen::Vector3d> * accel_noise = NULL, noise<Eigen::Vector3d> * gyro_noise = NULL)
      : sensor(sampleTime, NULL) {
          accel_ = new accelerometer(robot, sampleTime, sensor_link_name, imu_wrt_link_offset, world, accel_noise);
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
      imu(raisim::ArticulatedSystem * robot, double sampleTime, Eigen::Vector3d imu_wrt_link_offset, raisim::World *world, imuParams& params)
      : sensor(sampleTime, NULL) {
        raisim_sensors::imu_param accel_params, gyro_params;

        accel_params.bias_correlation_time  = params.getParamDouble("accelerometerBiasCorrelationTime");
        accel_params.turnOnBias_sigma       = params.getParamDouble("accelerometerTurnOnBiasSigma");
        accel_params.noise_density          = params.getParamDouble("accelerometerNoiseDensity");
        accel_params.random_walk            = params.getParamDouble("accelerometerRandomWalk");
        gyro_params.bias_correlation_time   = params.getParamDouble("gyroscopeBiasCorrelationTime");
        gyro_params.turnOnBias_sigma        = params.getParamDouble("gyroscopeTurnOnBiasSigma");
        gyro_params.noise_density           = params.getParamDouble("gyroscopeNoiseDensity");
        gyro_params.random_walk             = params.getParamDouble("gyroscopeRandomWalk");

        std::string imu_topic = params.getParamString("imuTopic");
        std::string link_name = params.getParamString("linkName");
        std::string sensorNamespace = params.getParamString("robotNamespace");
        imuNoise *accelNoise = new raisim_sensors::imuNoise(sampleTime,accel_params);
        imuNoise *gyroNoise = new raisim_sensors::imuNoise(sampleTime,gyro_params);

        accel_ = new accelerometer(robot, sampleTime, link_name, imu_wrt_link_offset, world, accelNoise);
        gyro_  = new gyroscope(robot, sampleTime, link_name, gyroNoise);  
        measureGT_.resize(6);
        measure_.resize(6);
        measure_.setZero();
        measureGT_.setZero();
      };
    
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
  class vicon : public sensor<Eigen::VectorXd, geometry_msgs::TransformStamped> {
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
      vicon(raisim::ArticulatedSystem * robot, double sampleTime, odometryParams& params)
      : sensor(sampleTime, NULL) {
        std::string sensor_link_name = params.getParamString("linkName");
        childFrameName_ = params.getParamString("childFrameId");
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
      vicon(raisim::SingleBodyObject * obj, double sampleTime, viconParams& params)
      : sensor(sampleTime, NULL) {
        childFrameName_ = params.getParamString("childFrameId");
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
  class odometry : public sensor<Eigen::VectorXd, nav_msgs::Odometry> {
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
      odometry(raisim::ArticulatedSystem *robot, double sampleTime, odometryParams &params)
      : sensor(sampleTime, NULL) {
        robot_ = robot;
        std::string sensor_link_name = params.getParamString("linkName");
        childFrameName_ = params.getParamString("childFrameId");
        baseLink_ = attachToLink(sensor_link_name);
        measureGT_.resize(7+6);
        measure_.resize(7+6);
        measure_.setZero();
        measureGT_.setZero();
      };
    
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
  class force : public sensor<Eigen::VectorXd, geometry_msgs::WrenchStamped> {
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
      force(raisim::ArticulatedSystem *robot, raisim::World *world, double sampleTime, const std::string child_frame_name, forceParams &params)
      : sensor(sampleTime, NULL), impulseFilter_(3,3),angularImpulseFilter_(3,3) {
        std::string sensor_link_name = params.getParamString("linkName");
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
  class contact : public sensor<Eigen::VectorXd, ros_raisim_interface::ContactInformation> {
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
      contact(raisim::ArticulatedSystem *robot, raisim::World *world, double sampleTime, const std::string child_frame_name, forceParams &params)
      : sensor(sampleTime, NULL), impulseFilter_(3,3) {
        std::string sensor_link_name = params.getParamString("linkName");
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