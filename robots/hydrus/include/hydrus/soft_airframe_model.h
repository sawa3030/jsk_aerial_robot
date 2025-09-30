// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

 #pragma once

 #include <hydrus/hydrus_tilted_robot_model.h>
 
 namespace aerial_robot_model {
 class SoftAirframeRobotModel : public HydrusTiltedRobotModel {
 public:
   SoftAirframeRobotModel(bool init_with_rosparam = true,
                          bool verbose = false,
                          double fc_t_min_thre = 0,
                          double epsilon = 10);
   virtual ~SoftAirframeRobotModel() = default;

   unsigned int virtual_motor_num_ = 6;

  ros::Subscriber rotor5_pose_sub_;
  ros::Subscriber body_pose_sub_;

  // mocap of rotor5
  KDL::Frame rotor5_pose_from_world_;
  KDL::Frame body_pose_from_world_;
  ros::Time rotor5_pose_update_time_;
  ros::Time body_pose_update_time_;
 
   std::vector<KDL::Vector> rotors_origin_from_cog_with_mocap_update;
   std::vector<KDL::Vector> rotors_normal_from_cog_with_mocap_update;

   void updateRobotModelImpl(const KDL::JntArray& joint_positions);
    void Rotor5MocapCallback(const geometry_msgs::PoseStamped& msg);
    void BodyMocapCallback(const geometry_msgs::PoseStamped& msg);
    template<class T> std::vector<T> getRotorsNormalFromCogWithMocapUpdate();
   template<class T> std::vector<T> getRotorsOriginFromCogWithMocapUpdate();

    private:
    ros::NodeHandle nh_;
 };
 
 template<> inline std::vector<KDL::Vector> SoftAirframeRobotModel::getRotorsNormalFromCogWithMocapUpdate()
  {
    // todo: it might be better to use mutex
    return rotors_normal_from_cog_with_mocap_update;
  }

  template<> inline std::vector<Eigen::Vector3d> SoftAirframeRobotModel::getRotorsNormalFromCogWithMocapUpdate()
  {
    return aerial_robot_model::kdlToEigen(SoftAirframeRobotModel::getRotorsNormalFromCogWithMocapUpdate<KDL::Vector>());
  }

  template<> inline std::vector<geometry_msgs::PointStamped> SoftAirframeRobotModel::getRotorsNormalFromCogWithMocapUpdate()
  {
    return aerial_robot_model::kdlToMsg(SoftAirframeRobotModel::getRotorsNormalFromCogWithMocapUpdate<KDL::Vector>());
  }

  template<> inline std::vector<tf2::Vector3> SoftAirframeRobotModel::getRotorsNormalFromCogWithMocapUpdate()
  {
    return aerial_robot_model::kdlToTf2(SoftAirframeRobotModel::getRotorsNormalFromCogWithMocapUpdate<KDL::Vector>());
  }

  template<> inline std::vector<KDL::Vector> SoftAirframeRobotModel::getRotorsOriginFromCogWithMocapUpdate()
  {
    // todo: it might be better to use mutex
    return rotors_origin_from_cog_with_mocap_update;
  }

  template<> inline std::vector<Eigen::Vector3d> SoftAirframeRobotModel::getRotorsOriginFromCogWithMocapUpdate()
  {
    return aerial_robot_model::kdlToEigen(SoftAirframeRobotModel::getRotorsOriginFromCogWithMocapUpdate<KDL::Vector>());
  }

  template<> inline std::vector<geometry_msgs::PointStamped> SoftAirframeRobotModel::getRotorsOriginFromCogWithMocapUpdate()
  {
    return aerial_robot_model::kdlToMsg(SoftAirframeRobotModel::getRotorsOriginFromCogWithMocapUpdate<KDL::Vector>());
  }

  template<> inline std::vector<tf2::Vector3> SoftAirframeRobotModel::getRotorsOriginFromCogWithMocapUpdate()
  {
    return aerial_robot_model::kdlToTf2(SoftAirframeRobotModel::getRotorsOriginFromCogWithMocapUpdate<KDL::Vector>());
  }

} // namespace aerial_robot_model