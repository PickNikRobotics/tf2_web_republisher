/*********************************************************************
 *
 *  Copyright (c) 2014, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

 *  Author: Julius Kammerl (jkammerl@willowgarage.com)
 *
 */

#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_web_republisher_msgs/action/tf_subscription.hpp>
#include <tf2_web_republisher_msgs/srv/republish_t_fs.hpp>

using namespace std::chrono_literals;

class TFRepublisher : public rclcpp::Node
{
protected:
  using GoalHandle = std::shared_ptr<const tf2_web_republisher_msgs::action::TFSubscription::Goal>;

public:
  explicit TFRepublisher(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node(name.c_str(), options)
  {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<tf2_web_republisher_msgs::action::TFSubscription>(
        this, "tf2_web_republisher", std::bind(&TFRepublisher::handle_goal, this, _1, _2),
        std::bind(&TFRepublisher::handle_cancel, this, _1), std::bind(&TFRepublisher::handle_accepted, this, _1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  ~TFRepublisher()
  {
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<tf2_web_republisher_msgs::action::TFSubscription>> gh)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("tf2_web_republisher"), "GoalHandle canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  std::string cleanTfFrame(const std::string& frame_id) const
  {
    if (frame_id[0] == '/')
    {
      return frame_id.substr(1);
    }
    return frame_id;
  }

  std::optional<geometry_msgs::msg::TransformStamped> threadSafeLookup(const std::string& target_frame,
                                                                       const std::string& source_frame)
  {
    std::scoped_lock<std::mutex> lock(tf_buffer_mutex_);
    std::optional<geometry_msgs::msg::TransformStamped> out;
    try
    {
      out = tf_buffer_->lookupTransform(cleanTfFrame(target_frame), cleanTfFrame(source_frame), tf2::TimePointZero);
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", target_frame.c_str(), source_frame.c_str(),
                  ex.what());
    }

    return out;
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID& uuid,
              std::shared_ptr<const tf2_web_republisher_msgs::action::TFSubscription::Goal> goal)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("tf2_web_republisher"), "GoalHandle request received");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<tf2_web_republisher_msgs::action::TFSubscription>>
          goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&TFRepublisher::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tf2_web_republisher_msgs::action::TFSubscription>>
                   goal_handle)
  {
    auto feedback = std::make_shared<tf2_web_republisher_msgs::action::TFSubscription::Feedback>();

    std::string target_frame = goal_handle->get_goal()->target_frame;
    feedback->transforms.reserve(goal_handle->get_goal()->source_frames.size());
    while (!goal_handle->is_canceling())
    {
      feedback->transforms.resize(0);
      for (auto& source_frame : goal_handle->get_goal()->source_frames)
      {
        if (auto transform = threadSafeLookup(target_frame, source_frame))
        {
          feedback->transforms.push_back(transform.value());
        }
      }
      // publish feedback
      goal_handle->publish_feedback(feedback);
      rclcpp::sleep_for(std::chrono::microseconds((size_t)(1E6 / goal_handle->get_goal()->rate)));
    }

    goal_handle->succeed(std::make_shared<tf2_web_republisher_msgs::action::TFSubscription::Result>());
  }

private:
  rclcpp_action::Server<tf2_web_republisher_msgs::action::TFSubscription>::SharedPtr action_server_;
  std::mutex tf_buffer_mutex_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto tf2_web_republisher = std::make_shared<TFRepublisher>("tf2_web_republisher");

  rclcpp::spin(tf2_web_republisher);

  return 0;
}
