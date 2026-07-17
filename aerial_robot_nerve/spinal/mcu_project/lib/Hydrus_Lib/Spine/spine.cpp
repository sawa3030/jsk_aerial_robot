/**
******************************************************************************
* File Name          : spine.cpp
* Description        : can-based internal comm network, spine side interface
 ------------------------------------------------------------------*/

#include "spine.h"
#include <cstdio>
#include <std_msgs/String.h>

namespace Spine
{
  /* components */
  /* CAUTIONS: be careful about the order of the var definition and func definition */
  namespace
  {
    std::vector<Neuron> neuron_;
    CANMotorSendDevice can_motor_send_device_;
    std::vector<std::reference_wrapper<Servo>> servo_;
    std::vector<std::reference_wrapper<Servo>> servo_with_send_flag_;
    CANInitializer can_initializer_(neuron_);
    std::vector<float> imu_weight_;

    uint8_t slave_num_ = 0;
    uint8_t servo_num_ = 0;
    int8_t uav_model_ = -1;
    uint8_t baselink_ = 2;

    /* sensor fusion */
    StateEstimate* estimator_;

    /* flight controller */
    FlightControl* controller_;

    /* ros */
    constexpr uint8_t SERVO_PUB_INTERVAL = 20; //[ms]
    constexpr uint8_t NEURON_IMU_PUB_INTERVAL = 5; //[ms]
    constexpr uint32_t SERVO_TORQUE_PUB_INTERVAL = 1000; //[ms]
    spinal::ServoStates servo_state_msg_;
    spinal::ServoTorqueStates servo_torque_state_msg_;
    ros::Publisher servo_state_pub_("servo/states", &servo_state_msg_);
    // merge torque_states to states
    ros::Publisher servo_torque_state_pub_("servo/torque_states", &servo_torque_state_msg_);
    std_msgs::String neuron_imu_diag_msg_;
    char neuron_imu_diag_buf_[256];
    ros::Publisher neuron_imu_diag_pub_("debug/neuron_imu_diag", &neuron_imu_diag_msg_);
    std::vector<spinal::Imu*> neuron_imu_msg_;
    std::vector<ros::Publisher*> neuron_imu_pub_;
    std::vector<char*> neuron_imu_topic_name_;
    std::vector<uint32_t> neuron_imu_diag_last_warn_time_;
    std::vector<uint32_t> neuron_imu_diag_last_pub_time_;

    // rename following subscriber.
    // taget_states -> target_position
    // torque_enable -> control_enable
    ros::Subscriber<spinal::ServoControlCmd> servo_position_sub_("servo/target_states", servoPositionCallback);
    ros::Subscriber<spinal::ServoControlCmd> servo_current_sub_("servo/target_current", servoCurrentCallback);
    ros::Subscriber<spinal::ServoTorqueCmd> servo_torque_ctrl_sub_("servo/torque_enable", servoTorqueControlCallback);

    ros::ServiceServer<spinal::GetBoardInfo::Request, spinal::GetBoardInfo::Response> board_info_srv_("get_board_info", boardInfoCallback);
    ros::ServiceServer<spinal::SetBoardConfig::Request, spinal::SetBoardConfig::Response> board_config_srv_("set_board_config", boardConfigCallback);

    spinal::GetBoardInfo::Response board_info_res_;

    ros::NodeHandle* nh_;
    uint32_t servo_last_pub_time_ = 0;
    uint32_t neuron_imu_last_pub_time_ = 0;
    uint32_t servo_torque_last_pub_time_ = 0;
    unsigned int can_idle_count_ = 0;
    bool servo_control_flag_ = true;

    uint32_t can_tx_idle_start_time_ = 0; // for pause CAN TX -> TODO: change to another task for spinal process
    uint32_t CAN_TX_PAUSE_TIME = 2000; // 2000 ms for 1Khz task rate. TODO: change to another task for spinal process
    unsigned int send_board_index = 0; // incremental board id assignment for CAN TX

    uint32_t last_connected_time_ =0;
  }

  void boardInfoCallback(const spinal::GetBoardInfo::Request& req, spinal::GetBoardInfo::Response& res)
  {
    for (unsigned int i = 0; i < slave_num_; i++) {
      Neuron& neuron = neuron_.at(i);
      spinal::BoardInfo& board = board_info_res_.boards[i];
      board.imu_send_data_flag = neuron.can_imu_.getSendDataFlag() ? 1 : 0;
      board.dynamixel_ttl_rs485_mixed = neuron.can_servo_.getDynamixelTTLRS485Mixed() ? 1 : 0;
      board.servo_pulley_skip_thresh = neuron.can_servo_.getPulleySkipThresh();
      board.slave_id = neuron.getSlaveId();

      for (unsigned int j = 0; j < board.servos_length; j++) {
        Servo& s = neuron.can_servo_.servo_.at(j);
        board.servos[j].id = s.getId();
        board.servos[j].p_gain = s.getPGain();
        board.servos[j].i_gain = s.getIGain();
        board.servos[j].d_gain = s.getDGain();
        board.servos[j].profile_velocity = s.getProfileVelocity();
        board.servos[j].current_limit = s.getCurrentLimit();
        board.servos[j].send_data_flag = s.getSendDataFlag() ? 1 : 0;
        board.servos[j].external_encoder_flag = s.getExternalEncoderFlag() ? 1 : 0;
        board.servos[j].joint_resolution = s.getJointResolution();
        board.servos[j].servo_resolution = s.getServoResolution();
      }
    }
    res = board_info_res_;
  }

  void servoPositionCallback(const spinal::ServoControlCmd& control_msg)
  {
    if (!servo_control_flag_) return;
    if (control_msg.index_length != control_msg.angles_length) return;
    for (unsigned int i = 0; i < control_msg.index_length; i++) {
      servo_.at(control_msg.index[i]).get().setGoalPosition(control_msg.angles[i]);
    }
  }

  void servoCurrentCallback(const spinal::ServoControlCmd& control_msg)
  {
    if (!servo_control_flag_) return;
    if (control_msg.index_length != control_msg.angles_length) return;
    for (unsigned int i = 0; i < control_msg.index_length; i++) {
      servo_.at(control_msg.index[i]).get().setGoalCurrent(control_msg.angles[i]);
      // TODO: change angles -> commands
    }
  }

  void servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg)
  {
    if (control_msg.index_length != control_msg.torque_enable_length) return;
    for (unsigned int i = 0; i < control_msg.index_length; i++) {
      servo_.at(control_msg.index[i]).get().setTorqueEnable((control_msg.torque_enable[i] != 0) ? true : false);

      /* update the target angle */
      if (servo_.at(control_msg.index[i]).get().getSendDataFlag()) {
        servo_.at(control_msg.index[i]).get().setGoalPosition(servo_.at(control_msg.index[i]).get().getPresentPosition());
      }
    }
  }

  void boardConfigCallback(const spinal::SetBoardConfig::Request& req, spinal::SetBoardConfig::Response& res)
  {
    /* Pause the spinal sending command for neuron to have enough time for flashmemory erase&write */
    can_tx_idle_start_time_ = HAL_GetTick();
    // TODO: change the return value to bool
    can_initializer_.configDevice(req);

    // TODO: please add string type message for consoling
    res.success = true;
  }

  bool init(CAN_GeranlHandleTypeDef* hcan, ros::NodeHandle* nh, StateEstimate* estimator, FlightControl* controller, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
  {
    /* CAN */
    CANDeviceManager::init(hcan, GPIOx, GPIO_Pin);

    /* Estimation */
    estimator_ = estimator;

    /* Control */
    controller_ = controller;

    HAL_Delay(5000); //wait neuron initialization
    CANDeviceManager::addDevice(can_initializer_);
    CANDeviceManager::CAN_START();
    can_initializer_.initDevices();

    slave_num_ = neuron_.size();
    if(slave_num_ == 0) return false;

    //add CAN devices to CANDeviceManager
    for (unsigned int i = 0; i < neuron_.size(); i++) {
      CANDeviceManager::addDevice(neuron_.at(i).can_motor_);
      can_motor_send_device_.addMotor(neuron_.at(i).can_motor_);
      CANDeviceManager::addDevice(neuron_.at(i).can_imu_);
      CANDeviceManager::addDevice(neuron_.at(i).can_servo_);
      for (unsigned int j = 0; j < neuron_.at(i).can_servo_.servo_.size(); j++) {
        neuron_.at(i).can_servo_.servo_.at(j).setIndex(servo_.size());
        servo_.push_back(neuron_.at(i).can_servo_.servo_.at(j));
        if (neuron_.at(i).can_servo_.servo_.at(j).getSendDataFlag()) {
          servo_with_send_flag_.push_back(neuron_.at(i).can_servo_.servo_.at(j));
        }
      }
    }
    servo_num_ = servo_.size();

    /* ros */
    nh_ = nh;

    if (servo_num_ > 0)
      {
        nh_->advertise(servo_state_pub_);
        nh_->advertise(servo_torque_state_pub_);
        nh_->subscribe(servo_position_sub_);
        nh_->subscribe(servo_current_sub_);
        nh_->subscribe(servo_torque_ctrl_sub_);
      }

    nh_->advertiseService(board_info_srv_);
    nh_->advertiseService(board_config_srv_);
    neuron_imu_diag_msg_.data = neuron_imu_diag_buf_;
    nh_->advertise(neuron_imu_diag_pub_);

    neuron_imu_msg_.reserve(slave_num_);
    neuron_imu_pub_.reserve(slave_num_);
    neuron_imu_topic_name_.reserve(slave_num_);
    neuron_imu_diag_last_warn_time_.assign(slave_num_, 0);
    neuron_imu_diag_last_pub_time_.assign(slave_num_, 0);
    for (unsigned int i = 0; i < slave_num_; i++) {
      char* topic_name = new char[20];
      snprintf(topic_name, 20, "imu/neuron%u", neuron_.at(i).getSlaveId());
      spinal::Imu* imu_msg = new spinal::Imu();
      ros::Publisher* imu_pub = new ros::Publisher(topic_name, imu_msg);
      neuron_imu_topic_name_.push_back(topic_name);
      neuron_imu_msg_.push_back(imu_msg);
      neuron_imu_pub_.push_back(imu_pub);
      nh_->advertise(*imu_pub);
    }

    /* uav model: special rule based on the number of gimbals (no send data flag servos) */
    uint8_t gimbal_servo_num = servo_num_ - servo_with_send_flag_.size();

    /* TODO: not good case processing */
    if(gimbal_servo_num == 0)
      {
        uav_model_ = spinal::UavInfo::HYDRUS;
      }
    if(gimbal_servo_num  == slave_num_)
      {
        uav_model_ = spinal::UavInfo::HYDRUS_XI;
      }
    if(gimbal_servo_num  == 2 * slave_num_)
      {
        uav_model_ = spinal::UavInfo::DRAGON;
      }

    /* update controller */
    controller_->setUavModel(uav_model_);
    controller_->setMotorNumber(slave_num_);

    servo_state_msg_.servos_length = servo_with_send_flag_.size();
    servo_state_msg_.servos = new spinal::ServoState[servo_with_send_flag_.size()];
    servo_torque_state_msg_.torque_enable_length = servo_num_;
    servo_torque_state_msg_.torque_enable = new uint8_t[servo_num_];

    /* other component */
    imu_weight_.resize(slave_num_ + 1);

    /* set IMU weights */
    // no fusion
    imu_weight_[0] = 1.0;
    for (uint i = 1; i < imu_weight_.size(); i++) imu_weight_[i] = 0.0;

    for (int i = 0; i < slave_num_; i++) {
      HAL_Delay(100);
      neuron_.at(i).can_imu_.init();

      IMU_ROS_CMD::addImu(&(neuron_.at(i).can_imu_));
    }

    //set response for get_board_info
    board_info_res_.boards_length = slave_num_;
    board_info_res_.boards = new spinal::BoardInfo[slave_num_];
    for (unsigned int i = 0; i < slave_num_; i++) {
      Neuron& neuron = neuron_.at(i);
      spinal::BoardInfo& board = board_info_res_.boards[i];
      board.servos_length = neuron.can_servo_.servo_.size();
      board.servos = new spinal::ServoInfo[board.servos_length];
    }

    return true;
  }

  void send()
  {
    if (slave_num_ == 0) return;

    if(HAL_GetTick() < can_tx_idle_start_time_ + CAN_TX_PAUSE_TIME) return;

    if(HAL_GetTick() % 2 == 0) {
      // 500Hz
      can_motor_send_device_.sendData();
    }
    else {
      if (slave_num_ != 0) {
        // 500Hz
        neuron_.at(send_board_index).can_servo_.sendData();
        send_board_index++;
        if (send_board_index == slave_num_) send_board_index = 0;
      }
    }

    can_initializer_.sendData(); // if necessary
  }

  void update(void)
  {
    if (slave_num_ == 0) return;

    /* update the motor PWM command */
    for(int i = 0; i < slave_num_; i++) {
      float pwm_rate = controller_->getTargetPwm(i);
      uint16_t pwm_bit = pwm_rate * 2000 - 1000;
      neuron_.at(i).can_motor_.setPwm(pwm_bit);
    }

    /* uodate IMU */
    for (int i = 0; i < slave_num_; i++)
      neuron_.at(i).can_imu_.update();

    /* ros publish */
    neuronImuPublish();
    servoPublish();

    CANDeviceManager::tick(1);

    uint32_t now_time = HAL_GetTick();
    if(CANDeviceManager::connected()) last_connected_time_ = now_time;

    if(now_time - last_connected_time_ > 1000 /* ms */)
      {
        if(nh_->connected()) nh_->logerror("CAN disconnected!!");
        last_connected_time_ = now_time;
      }
  }

  void useRTOS(osMailQId* handle)
  {
    CANDeviceManager::useRTOS(handle);
  }

  void setMotorPwm(uint16_t pwm, uint8_t motor)
  {
    if(slave_num_ == 0) {
      return;
    }
    neuron_.at(motor).can_motor_.setPwm(pwm);
  }

  bool connected()
  {
    if (slave_num_ > 0) return true;

    return false;
  }

  uint8_t getSlaveNum()
  {
    return slave_num_;
  }

  int8_t getUavModel()
  {
    return uav_model_;
  }

  void setServoControlFlag(bool flag)
  {
    servo_control_flag_ = flag;
  }

  void servoPublish()
  {
    if (servo_num_ == 0) return;

    uint32_t now_time = HAL_GetTick();
    if( now_time - servo_last_pub_time_ >= SERVO_PUB_INTERVAL)
      {
        /* send servo */
        servo_state_msg_.stamp = nh_->now();
        for (unsigned int i = 0; i < servo_with_send_flag_.size(); i++)
          {
            spinal::ServoState servo;

            servo.index = servo_with_send_flag_.at(i).get().getIndex();
            servo.angle = servo_with_send_flag_.at(i).get().getPresentPosition();
            servo.temp = servo_with_send_flag_.at(i).get().getPresentTemperature();
            servo.load = servo_with_send_flag_.at(i).get().getPresentCurrent();
            servo.error = servo_with_send_flag_.at(i).get().getError();

            servo_state_msg_.servos[i] = servo;
          }

        servo_state_pub_.publish(&servo_state_msg_);
        servo_last_pub_time_ = now_time;
      }

    if( now_time - servo_torque_last_pub_time_ >= SERVO_TORQUE_PUB_INTERVAL)
      {
        for (unsigned int i = 0; i < servo_num_; i++)
          {
            servo_torque_state_msg_.torque_enable[i] = servo_.at(i).get().getTorqueEnable() ? 1 : 0;
          }
        servo_torque_state_pub_.publish(&servo_torque_state_msg_);
        servo_torque_last_pub_time_ = now_time;
      }
  }

  void neuronImuPublish()
  {
    if (slave_num_ == 0) return;

    uint32_t now_time = HAL_GetTick();
    if (now_time - neuron_imu_last_pub_time_ < NEURON_IMU_PUB_INTERVAL) return;
    neuron_imu_last_pub_time_ = now_time;

    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;
    // CAN IMU returns only gyro/acc/mag, so reuse the current spinal attitude estimate.
    if (estimator_ && estimator_->getAttEstimator() && estimator_->getAttEstimator()->getEstimator()) {
      ap::Quaternion q = estimator_->getAttEstimator()->getEstimator()->getQuaternion();
      qx = q[1];
      qy = q[2];
      qz = q[3];
      qw = q[0];
    }

    ros::Time stamp = nh_->now();
    for (unsigned int i = 0; i < slave_num_; i++) {
      CANIMU& can_imu = neuron_.at(i).can_imu_;
      ap::Vector3f acc = can_imu.getAcc();
      ap::Vector3f gyro = can_imu.getGyro();
      ap::Vector3f mag = can_imu.getMag();
      int32_t acc_milli_x = static_cast<int32_t>(acc[0] * 1000.0f);
      int32_t acc_milli_y = static_cast<int32_t>(acc[1] * 1000.0f);
      int32_t acc_milli_z = static_cast<int32_t>(acc[2] * 1000.0f);
      int32_t mag_milli_x = static_cast<int32_t>(mag[0] * 1000.0f);
      int32_t mag_milli_y = static_cast<int32_t>(mag[1] * 1000.0f);
      int32_t mag_milli_z = static_cast<int32_t>(mag[2] * 1000.0f);

      if (now_time - neuron_imu_diag_last_pub_time_.at(i) > 1000) {
        snprintf(neuron_imu_diag_buf_, sizeof(neuron_imu_diag_buf_),
                 "neuron%u imu state: send=%u g=%u a=%u m=%u age=%lu/%lu/%lu acc_raw=%d,%d,%d mag_raw=%d,%d,%d acc_mg=%ld,%ld,%ld mag_milli=%ld,%ld,%ld",
                 neuron_.at(i).getSlaveId(),
                 can_imu.getSendDataFlag() ? 1 : 0,
                 can_imu.hasGyroData() ? 1 : 0,
                 can_imu.hasAccData() ? 1 : 0,
                 can_imu.hasMagData() ? 1 : 0,
                 static_cast<unsigned long>(now_time - can_imu.getLastGyroReceiveTime()),
                 static_cast<unsigned long>(now_time - can_imu.getLastAccReceiveTime()),
                 static_cast<unsigned long>(now_time - can_imu.getLastMagReceiveTime()),
                 can_imu.getRawAccData(0),
                 can_imu.getRawAccData(1),
                 can_imu.getRawAccData(2),
                 can_imu.getRawMagData(0),
                 can_imu.getRawMagData(1),
                 can_imu.getRawMagData(2),
                 static_cast<long>(acc_milli_x),
                 static_cast<long>(acc_milli_y),
                 static_cast<long>(acc_milli_z),
                 static_cast<long>(mag_milli_x),
                 static_cast<long>(mag_milli_y),
                 static_cast<long>(mag_milli_z));
        neuron_imu_diag_pub_.publish(&neuron_imu_diag_msg_);
        neuron_imu_diag_last_pub_time_.at(i) = now_time;
      }

      if (!can_imu.getSendDataFlag()) continue;

      if (!can_imu.hasData()) {
        if (can_imu.hasGyroData() &&
            (!can_imu.hasAccData() || !can_imu.hasMagData()) &&
            now_time - neuron_imu_diag_last_warn_time_.at(i) > 1000) {
          char log_msg[96];
          snprintf(log_msg, sizeof(log_msg),
                   "neuron%u imu missing: gyro=%u acc=%u mag=%u",
                   neuron_.at(i).getSlaveId(),
                   can_imu.hasGyroData() ? 1 : 0,
                   can_imu.hasAccData() ? 1 : 0,
                   can_imu.hasMagData() ? 1 : 0);
          snprintf(neuron_imu_diag_buf_, sizeof(neuron_imu_diag_buf_), "%s", log_msg);
          nh_->logwarn(log_msg);
          neuron_imu_diag_pub_.publish(&neuron_imu_diag_msg_);
          neuron_imu_diag_last_warn_time_.at(i) = now_time;
        }
        continue;
      }

      uint32_t gyro_age = now_time - can_imu.getLastGyroReceiveTime();
      uint32_t acc_age = now_time - can_imu.getLastAccReceiveTime();
      uint32_t mag_age = now_time - can_imu.getLastMagReceiveTime();
      if (gyro_age < 100 && (acc_age > 100 || mag_age > 100) &&
          now_time - neuron_imu_diag_last_warn_time_.at(i) > 1000) {
        char log_msg[96];
        snprintf(log_msg, sizeof(log_msg),
                 "neuron%u imu stale: gyro=%lums acc=%lums mag=%lums",
                 neuron_.at(i).getSlaveId(),
                 static_cast<unsigned long>(gyro_age),
                 static_cast<unsigned long>(acc_age),
                 static_cast<unsigned long>(mag_age));
        snprintf(neuron_imu_diag_buf_, sizeof(neuron_imu_diag_buf_), "%s", log_msg);
        nh_->logwarn(log_msg);
        neuron_imu_diag_pub_.publish(&neuron_imu_diag_msg_);
        neuron_imu_diag_last_warn_time_.at(i) = now_time;
      }

      spinal::Imu* imu_msg = neuron_imu_msg_.at(i);
      imu_msg->stamp = stamp;

      for (int axis = 0; axis < 3; axis++) {
        imu_msg->acc[axis] = acc[axis];
        imu_msg->gyro[axis] = gyro[axis];
        imu_msg->mag[axis] = mag[axis];
      }

      imu_msg->quaternion[0] = qx;
      imu_msg->quaternion[1] = qy;
      imu_msg->quaternion[2] = qz;
      imu_msg->quaternion[3] = qw;

      neuron_imu_pub_.at(i)->publish(imu_msg);
    }
  }
};
