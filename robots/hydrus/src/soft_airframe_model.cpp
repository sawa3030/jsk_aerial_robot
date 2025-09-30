#include <hydrus/soft_airframe_model.h>

using namespace aerial_robot_model;

SoftAirframeRobotModel::SoftAirframeRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon):
  HydrusTiltedRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
  , rotors_origin_from_cog_with_mocap_update(this->virtual_motor_num_)
  , rotors_normal_from_cog_with_mocap_update(this->virtual_motor_num_)
  , nh_("~")
{
    rotor5_pose_sub_ = nh_.subscribe("thrust5/mocap/pose", 1, &SoftAirframeRobotModel::Rotor5MocapCallback, this);
    body_pose_sub_ = nh_.subscribe("mocap/pose", 1, &SoftAirframeRobotModel::BodyMocapCallback, this);
}

void SoftAirframeRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
    HydrusTiltedRobotModel::updateRobotModelImpl(joint_positions);
    
    std::vector<KDL::Vector> rotors_origin_from_cog = getRotorsOriginFromCog<KDL::Vector>();
    std::vector<KDL::Vector> rotors_normal_from_cog = getRotorsNormalFromCog<KDL::Vector>();

    assert(rotors_origin_from_cog.size() == 5);
    assert(rotors_normal_from_cog.size() == 5);
    assert(virtual_motor_num_ == 6);
    assert(rotors_origin_from_cog_with_mocap_update.size() == 6);
    assert(rotors_normal_from_cog_with_mocap_update.size() == 6);

    std::copy(rotors_origin_from_cog.begin(), rotors_origin_from_cog.end(), rotors_origin_from_cog_with_mocap_update.begin());
    std::copy(rotors_normal_from_cog.begin(), rotors_normal_from_cog.end(), rotors_normal_from_cog_with_mocap_update.begin());
    KDL::Frame cog = getCog<KDL::Frame>();

    if (ros::Time::now().toSec() - rotor5_pose_update_time_.toSec() < 1.0 && 
        ros::Time::now().toSec() - body_pose_update_time_.toSec() < 1.0){
        KDL::Frame body_pose_from_root_ = getSegmentsTf().at("fc");
        KDL::Frame rotor5_pose_from_root = body_pose_from_root_ * body_pose_from_world_.Inverse() * rotor5_pose_from_world_;
        rotors_origin_from_cog_with_mocap_update.at(4) = (cog.Inverse() * rotor5_pose_from_root).p;
        rotors_normal_from_cog_with_mocap_update.at(4) = (cog.Inverse() * rotor5_pose_from_root).M * KDL::Vector(0,0,1);
    }

    KDL::Frame f = getSegmentsTf().at("rotor5");
    rotors_origin_from_cog_with_mocap_update.at(5) = (cog.Inverse() * f).p;
    rotors_normal_from_cog_with_mocap_update.at(5) = (cog.Inverse() * f).M * KDL::Vector(0,-1,0);
}


void SoftAirframeRobotModel::Rotor5MocapCallback(const geometry_msgs::PoseStamped& msg)
{
  tf2::fromMsg(msg.pose, rotor5_pose_from_world_);
  rotor5_pose_update_time_ = ros::Time::now();
}

void SoftAirframeRobotModel::BodyMocapCallback(const geometry_msgs::PoseStamped& msg)
{
  tf2::fromMsg(msg.pose, body_pose_from_world_);
  body_pose_update_time_ = ros::Time::now();
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SoftAirframeRobotModel, aerial_robot_model::RobotModel);
