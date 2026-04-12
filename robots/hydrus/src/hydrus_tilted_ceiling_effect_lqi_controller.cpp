#include <hydrus/hydrus_tilted_ceiling_effect_lqi_controller.h>

using namespace aerial_robot_control;

HydrusTiltedCeilingEffectLQIController::HydrusTiltedCeilingEffectLQIController():
  UnderActuatedTiltedLQIController()
{
}

void HydrusTiltedCeilingEffectLQIController::initialize(ros::NodeHandle nh,
                                                        ros::NodeHandle nhp,
                                                        boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                        boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                        boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                                        double ctrl_loop_rate)
{
  UnderActuatedTiltedLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
}

bool HydrusTiltedCeilingEffectLQIController::checkRobotModel()
{
  if(!robot_model_->initialized())
    {
      ROS_DEBUG_NAMED("LQI gain generator", "LQI gain generator: robot model is not initiliazed");
      return false;
    }

  if(!robot_model_->stabilityCheck(verbose_))
    {
      ROS_ERROR_NAMED("LQI gain generator", "LQI gain generator: invalid pose, stability is invalid");

      return false;
    }
  return true;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusTiltedCeilingEffectLQIController, aerial_robot_control::ControlBase);
