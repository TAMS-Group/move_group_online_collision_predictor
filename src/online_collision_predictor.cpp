/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Michael 'v4hn' Goerner
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
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the author nor the names of its
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

/* Author: Michael 'v4hn' Goerner */

#include <moveit/move_group/move_group_capability.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

#include <std_msgs/Bool.h>

#include <thread>

namespace move_group {
class OnlineCollisionPredictor : public MoveGroupCapability {
public:
  OnlineCollisionPredictor();

  virtual void initialize();

  void continuous_predict();

private:
  ros::Publisher pub_;
  double rate_;
  double horizon_;

  bool colliding_;

  std::thread checker_thread_;
};
}

move_group::OnlineCollisionPredictor::OnlineCollisionPredictor()
    : MoveGroupCapability("OnlineCollisionPredictor"), colliding_(false) {}

void move_group::OnlineCollisionPredictor::initialize() {
  // velocities have to be copied to the current state for this plugin
  context_->planning_scene_monitor_->getStateMonitorNonConst()->enableCopyDynamics(true);

  // maximum rate for collision checks
  rate_ = node_handle_.param<double>("online_collision_checker/rate", 2);
  // extrapolate this far into the future (single-step extrapolation)
  horizon_ = node_handle_.param<double>("online_collision_checker/horizon", .5);
  pub_ = root_node_handle_.advertise<std_msgs::Bool>(
      "online_collision_prediction", 1, true);
  { // publish initial msg on latched topic
    std_msgs::Bool msg;
    msg.data= colliding_;
    pub_.publish(msg);
  }
  checker_thread_ = std::thread(std::bind(
      &move_group::OnlineCollisionPredictor::continuous_predict, this));
}

void move_group::OnlineCollisionPredictor::continuous_predict() {
  ros::Rate rate(rate_);

  while (ros::ok()) {
    // force update to latest received joint state
    context_->planning_scene_monitor_->updateSceneWithCurrentState();
    { // lock planning scene for prediction only
      planning_scene_monitor::LockedPlanningSceneRO ps(
          context_->planning_scene_monitor_);
      planning_scene::PlanningScenePtr prediction(ps->diff());
      robot_state::RobotState &predicted_state =
          prediction->getCurrentStateNonConst();

      if(!ps->getCurrentState().hasVelocities()){
        ROS_ERROR_THROTTLE_NAMED(5.0, "online_collision_prediction",
          "current monitored state has no velocities. "
          "move_group/OnlineCollisionPredictor does not work.");
      }
      else {
        double *positions = predicted_state.getVariablePositions();
        double *velocities = predicted_state.getVariableVelocities();

        for (int i = 0; i < predicted_state.getVariableCount(); ++i)
          positions[i] += horizon_ * velocities[i];

        predicted_state.enforceBounds();

        // force update after changing internal variables
        predicted_state.update(true);

        // topic is latched, so publish only on change
        if (prediction->isStateColliding() != colliding_) {
          std_msgs::Bool msg;
          msg.data = !colliding_;
          pub_.publish(msg);
          colliding_ = !colliding_;
        }
      }
    }

    rate.sleep();
  };
}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::OnlineCollisionPredictor,
                            move_group::MoveGroupCapability)
