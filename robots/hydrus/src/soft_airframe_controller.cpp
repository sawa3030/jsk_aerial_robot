#include <hydrus/soft_airframe_controller.h>
using namespace aerial_robot_control;

SoftAirframeController::SoftAirframeController() : PoseLinearController()
{
}

void SoftAirframeController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                        boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                        boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                        boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                        double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  
  rosParamInit();
  target_base_thrust_.resize(motor_num_);

  //publisher
  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  q_mat_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("q_matrix", 1);
  rotor_attitude_contributions_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("rotor_attitude_contributions", 1);
  
  // subscriber
  joint_state_sub_ = nh_.subscribe("joint_states", 1, &SoftAirframeController::jointStateCallback, this);
  rpy_pid_sub_ = nh_.subscribe("rpy/pid", 1, &SoftAirframeController::publishRotorAttitudeContributions, this);

  torque_allocation_matrix_inv_pub_stamp_ = 0.0;
  q_mat_update_stamp_ = 0.0;
  prev_target_vectoring_f_ = Eigen::VectorXd::Zero(motor_num_);
  n_constraints = motor_num_ + 4;
}

void SoftAirframeController::controlCore()
{
  PoseLinearController::controlCore();

  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;

  if(navigator_->getForceLandingFlag())
  {
    target_pitch_ = 0;
    target_roll_ = 0;
  }

  if (ros::Time::now().toSec() - q_mat_update_stamp_ > torque_allocation_matrix_inv_pub_interval_)
  {
    q_mat_update_stamp_ = ros::Time::now().toSec();  
    q_mat_ = getQMat();
    q_mat_inv_ = aerial_robot_model::pseudoinverse(q_mat_);
  }

  Eigen::VectorXd target_vectoring_f_ = Eigen::VectorXd::Zero(motor_num_);
  Eigen::VectorXd z_rpy_ddot = Eigen::VectorXd::Zero(4);
  if(hovering_approximate_)
    {
      target_pitch_ = target_acc_dash.x() / aerial_robot_estimation::G;
      target_roll_ = -target_acc_dash.y() / aerial_robot_estimation::G;
      z_rpy_ddot(0) = target_acc_w.z();
    }
  else
    {
      target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
      target_roll_ = atan2(-target_acc_dash.y(), sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));
      z_rpy_ddot(0) = target_acc_w.length();
    }
  // z_rpy_ddot(1) = pid_controllers_.at(ROLL).result();
  // z_rpy_ddot(2) = pid_controllers_.at(PITCH).result();
  // z_rpy_ddot(3) = pid_controllers_.at(YAW).result();

  z_rpy_ddot(1) = 0.0;
  z_rpy_ddot(2) = 0.0;
  z_rpy_ddot(3) = 0.0;

  // solve the thrust allocation with QP
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(motor_num_, motor_num_);
  H.diagonal().setConstant(2.0);
  Eigen::VectorXd g = prev_target_vectoring_f_ * -2.0;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_constraints, motor_num_);
  A.topRows(4) = q_mat_;
  for (int i = 0; i < motor_num_; i++)
  {
    A(i + 4, i) = 1.0;
  }

  Eigen::VectorXd lb(n_constraints);
  Eigen::VectorXd ub(n_constraints);

  lb.head(4) = z_rpy_ddot;
  for (int i = 0; i < motor_num_; i++)
  {
    lb(i + 4) = 0.0;
  }

  ub.head(4) = z_rpy_ddot;
  for (int i = 0; i < motor_num_; i++)
  {
    ub(i + 4) = 20.0;
  }

  
  Eigen::SparseMatrix<double> H_s = H.sparseView();
  Eigen::SparseMatrix<double> A_s = A.sparseView();
  if(!target_vectoring_qp_solver_.isInitialized())
  {
    target_vectoring_qp_solver_.settings()->setVerbosity(false);
    target_vectoring_qp_solver_.settings()->setWarmStart(true);
    target_vectoring_qp_solver_.settings()->setPolish(false);
    target_vectoring_qp_solver_.settings()->setMaxIteraction(1000);
    target_vectoring_qp_solver_.settings()->setAbsoluteTolerance(1e-4);
    target_vectoring_qp_solver_.settings()->setRelativeTolerance(1e-4);

    target_vectoring_qp_solver_.data()->setNumberOfVariables(motor_num_);
    target_vectoring_qp_solver_.data()->setNumberOfConstraints(n_constraints);
    target_vectoring_qp_solver_.data()->setHessianMatrix(H_s);
    target_vectoring_qp_solver_.data()->setGradient(g);
    target_vectoring_qp_solver_.data()->setLinearConstraintsMatrix(A_s);
    target_vectoring_qp_solver_.data()->setLowerBound(lb);
    target_vectoring_qp_solver_.data()->setUpperBound(ub);
    target_vectoring_qp_solver_.initSolver();
  }
  else
  {
    target_vectoring_qp_solver_.updateHessianMatrix(H_s);
    target_vectoring_qp_solver_.updateGradient(g);
    target_vectoring_qp_solver_.updateLinearConstraintsMatrix(A_s);
    target_vectoring_qp_solver_.updateBounds(lb, ub);
  }

  bool solved = target_vectoring_qp_solver_.solve();
  if(solved){
    target_vectoring_f_ = target_vectoring_qp_solver_.getSolution();
  } else {
    target_vectoring_f_ = q_mat_inv_ * z_rpy_ddot;
    target_vectoring_f_.noalias() += prev_target_vectoring_f_;
    target_vectoring_f_.noalias() -= q_mat_inv_ * (q_mat_ * prev_target_vectoring_f_);
    std::cout << "QP not solved!: " << target_vectoring_f_.transpose() << std::endl;
  }
  ROS_DEBUG_STREAM("target vectoring f: \n" << target_vectoring_f_.transpose());

  for (int i = 0; i < motor_num_; i++)
  {
    if (target_vectoring_f_(i) < lb(i + 4)){
      target_vectoring_f_(i) = lb(i + 4);
    }
    if (target_vectoring_f_(i) > ub(i + 4)){
      target_vectoring_f_(i) = ub(i + 4);
    }
  }
  prev_target_vectoring_f_ = target_vectoring_f_;

  for(int i = 0; i < motor_num_; i++)
  {
    target_base_thrust_.at(i) = target_vectoring_f_(i);
  }

  // special process for yaw since the bandwidth between PC and spinal
  double max_yaw_scale = 0; // for reconstruct yaw control term in spinal
  for (unsigned int i = 0; i < motor_num_; i++)
    {
      if(q_mat_inv_(i, YAW - 2) > max_yaw_scale) max_yaw_scale = q_mat_inv_(i, YAW - 2);
    }
  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;


  navigator_->setTargetPitch(target_pitch_);
  navigator_->setTargetRoll(target_roll_);
  pid_msg_.roll.total.at(0) = pid_controllers_.at(ROLL).result();
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = pid_controllers_.at(PITCH).result();
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  ROS_INFO_STREAM_THROTTLE(0.5, "[SoftAirframeController] controlCore");
}

Eigen::MatrixXd SoftAirframeController::getQMat()
{
  // wrench allocation matrix
  std::vector<Eigen::Vector3d> rotors_origin = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  std::vector<Eigen::Vector3d> rotors_normal = robot_model_->getRotorsNormalFromCog<Eigen::Vector3d>();
  auto& rotor_direction = robot_model_->getRotorDirection();
  
  Eigen::MatrixXd q_mat = Eigen::MatrixXd::Zero(4, motor_num_);
  for (unsigned int i = 0; i < motor_num_; ++i) {
    double m_f_rate = robot_model_->getMFRate();
    q_mat(0, i) = rotors_normal.at(i).z();
    q_mat.block(1, i, 3, 1) = (rotors_origin.at(i).cross(rotors_normal.at(i)) + m_f_rate * rotor_direction.at(i + 1) * rotors_normal.at(i));
  }
  double mass_inv = 1.0 / robot_model_->getMass();
  Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
  q_mat.topRows(1) =  mass_inv * q_mat.topRows(1) ;
  q_mat.bottomRows(3) =  inertia_inv * q_mat.bottomRows(3);
  return q_mat;
}

void SoftAirframeController::sendCmd()
{
  PoseLinearController::sendCmd();

  sendFourAxisCommand();
  sendTorqueAllocationMatrixInv();
  publishQMat();
  ROS_INFO_STREAM_THROTTLE(0.5, "[SoftAirframeController] sendCmd");
}

void SoftAirframeController::reset()
{
  PoseLinearController::reset();

  setAttitudeGains();
}

void SoftAirframeController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.angles[0] = target_roll_;
  flight_command_data.angles[1] = target_pitch_;
  flight_command_data.angles[2] = candidate_yaw_term_;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_.publish(flight_command_data);
}

void SoftAirframeController::jointStateCallback(const sensor_msgs::JointState& msg)
{
  // if (msg.position.empty())
  //   {
  //     ROS_WARN_THROTTLE(1.0, "[SoftAirframeController] received joint_states with empty position; skip gimbal angle update");
  //     return;
  //   }

  // gimbal_current_angle = msg.position.at(0); // todo: think a robust implementation
  // gimbal_update_time = ros::Time::now();
  return;
}

void SoftAirframeController::sendTorqueAllocationMatrixInv()
{
  if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
    {
      torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now().toSec();

      spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
      torque_allocation_matrix_inv_msg.rows.resize(motor_num_);
      Eigen::MatrixXd torque_allocation_matrix_inv = q_mat_inv_.rightCols(3);
      if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
        ROS_ERROR("Torque Allocation Matrix overflow");
      for (unsigned int i = 0; i < motor_num_; i++)
        {
          torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i,0) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i,1) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i,2) * 1000;
        }
      torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
    }
}

void SoftAirframeController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "hovering_approximate", hovering_approximate_, false);
  getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_, 0.05);
}

void SoftAirframeController::setAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg; //for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_i = pid_controllers_.at(ROLL).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).yaw_d = pid_controllers_.at(YAW).getDGain() * 1000;
  // rpy_gain_msg.motors.at(0).roll_p = 0;
  // rpy_gain_msg.motors.at(0).roll_i = 0;
  // rpy_gain_msg.motors.at(0).roll_d = 0;
  // rpy_gain_msg.motors.at(0).pitch_p = 0;
  // rpy_gain_msg.motors.at(0).pitch_i = 0;
  // rpy_gain_msg.motors.at(0).pitch_d = 0;
  // rpy_gain_msg.motors.at(0).yaw_d = 0;
  rpy_gain_pub_.publish(rpy_gain_msg);
}

void SoftAirframeController::publishQMat()
{
  std_msgs::Float32MultiArray q_mat_msg;
  q_mat_msg.layout.dim.resize(2);
  q_mat_msg.layout.dim[0].label = "rows";
  q_mat_msg.layout.dim[0].size = q_mat_.rows();
  q_mat_msg.layout.dim[0].stride = q_mat_.cols();
  q_mat_msg.layout.dim[1].label = "cols";
  q_mat_msg.layout.dim[1].size = q_mat_.cols();
  q_mat_msg.layout.dim[1].stride = 1;
  q_mat_msg.data.resize(q_mat_.rows() * q_mat_.cols());
  for (int i = 0; i < q_mat_.rows(); i++)
    {
      for (int j = 0; j < q_mat_.cols(); j++)
        {
          q_mat_msg.data[i * q_mat_.cols() + j] = q_mat_(i, j);
        }
      }
  q_mat_pub_.publish(q_mat_msg);
}

void SoftAirframeController::publishRotorAttitudeContributions(const spinal::RollPitchYawTerms &control_term_msg_)
{
  std_msgs::Float32MultiArray rotor_attitude_contributions_msg;
  rotor_attitude_contributions_msg.layout.dim.resize(2);
  rotor_attitude_contributions_msg.layout.dim[0].label = "rows";
  rotor_attitude_contributions_msg.layout.dim[0].size = motor_num_;
  rotor_attitude_contributions_msg.layout.dim[0].stride = 3;
  rotor_attitude_contributions_msg.layout.dim[1].label = "cols";
  rotor_attitude_contributions_msg.layout.dim[1].size = 3;
  rotor_attitude_contributions_msg.layout.dim[1].stride = 1;
  rotor_attitude_contributions_msg.data.resize(motor_num_ * 3);

  Eigen::MatrixXd q_mat_temp = getQMat();

  if (control_term_msg_.motors.size() != motor_num_){
    return;
  }
  
    for (int i = 0; i < motor_num_; i++)
      {
        rotor_attitude_contributions_msg.data[i * 3] = q_mat_temp(1, i) * (control_term_msg_.motors[i].roll_p + control_term_msg_.motors[i].roll_i + control_term_msg_.motors[i].roll_d) * 0.001f;
        rotor_attitude_contributions_msg.data[i * 3 + 1] = q_mat_temp(2, i) * (control_term_msg_.motors[i].roll_p + control_term_msg_.motors[i].roll_i + control_term_msg_.motors[i].roll_d) * 0.001f;
        rotor_attitude_contributions_msg.data[i * 3 + 2] = q_mat_temp(3, i) * control_term_msg_.motors[i].yaw_d * 0.001f;
      }

  rotor_attitude_contributions_pub_.publish(rotor_attitude_contributions_msg);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::SoftAirframeController, aerial_robot_control::ControlBase);
