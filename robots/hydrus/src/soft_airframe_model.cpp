#include <hydrus/soft_airframe_model.h>

SoftAirframeRobotModel::SoftAirframeRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon):
  HydrusTiltedRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SoftAirframeRobotModel, aerial_robot_model::RobotModel);
