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

#include <boost/thread/mutex.hpp>
#include "boost/thread.hpp"

#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_web_republisher_msgs/action/tf_subscription.hpp>

#include <tf2_web_republisher_msgs/srv/republish_t_fs.hpp>
#include <tf2_web_republisher_msgs/msg/tf_array.hpp>

#include "tf_pair.h"

using namespace std::chrono_literals;

class TFRepublisher : public rclcpp::Node {
protected:
    using GoalHandle = std::shared_ptr<const tf2_web_republisher_msgs::action::TFSubscription::Goal>;

    // base struct that holds information about the TFs
    // a client (either Service or Action) has subscribed to
    struct ClientInfo {
        std::vector<TFPair> tf_subscriptions_;
        unsigned int client_ID_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

    // struct for Action client info
    struct ClientGoalInfo : ClientInfo {
        rclcpp_action::GoalUUID handle;
    };

    // struct for Service client info
    struct ClientRequestInfo : ClientInfo {
        rclcpp::Publisher<tf2_web_republisher_msgs::msg::TFArray>::SharedPtr pub_;
        rclcpp::Duration unsub_timeout_;
        rclcpp::TimerBase::SharedPtr unsub_timer_;
    };

    std::list<boost::shared_ptr<ClientGoalInfo> > active_goals_;
    boost::mutex goals_mutex_;

    std::list<boost::shared_ptr<ClientRequestInfo> > active_requests_;
    boost::mutex requests_mutex_;

    boost::mutex tf_buffer_mutex_;

    unsigned int client_ID_count_;

public:

    explicit TFRepublisher(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node(name.c_str(), options) {

        using namespace std::placeholders;

        action_server_ = rclcpp_action::create_server<tf2_web_republisher_msgs::action::TFSubscription>(
                this,
                "republish_tfs",
                std::bind(&TFRepublisher::handle_goal, this, _1, _2),
                std::bind(&TFRepublisher::handle_cancel, this, _1),
                std::bind(&TFRepublisher::handle_accepted, this, _1));

        tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    }

//    tf_republish_service_ = nh_.advertiseService("republish_tfs",
//                                                 &TFRepublisher::requestCB,
//                                                 this);
//    as_.start();
//  }

    ~TFRepublisher() {}


    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tf2_web_republisher_msgs::action::TFSubscription>> gh) {
        boost::mutex::scoped_lock l(goals_mutex_);



        RCLCPP_DEBUG(rclcpp::get_logger("tf2_web_republisher"), "GoalHandle canceled");

        // search for goal handle and remove it from active_goals_ list
        for (auto it = active_goals_.begin(); it != active_goals_.end(); it++) {
            ClientGoalInfo &info = **it;
            if (info.handle == gh->get_goal_id()) {
                active_goals_.erase(it);
                info.timer_->cancel();
                return rclcpp_action::CancelResponse::ACCEPT;
            } else
                ++it;
        }

        return rclcpp_action::CancelResponse::REJECT;
    }

    const std::string cleanTfFrame(const std::string frame_id) const {
        if (frame_id[0] == '/') {
            return frame_id.substr(1);
        }
        return frame_id;
    }

    /**
     * Set up the contents of \p tf_subscriptions_ in
     * a ClientInfo struct
     */
    void setSubscriptions(boost::shared_ptr<ClientInfo> info,
                          const std::vector<std::string> &source_frames,
                          const std::string &target_frame_,
                          float angular_thres,
                          float trans_thres) const {
        std::size_t request_size_ = source_frames.size();
        info->tf_subscriptions_.resize(request_size_);

        for (std::size_t i = 0; i < request_size_; ++i) {
            TFPair &tf_pair = info->tf_subscriptions_[i];

            std::string source_frame = cleanTfFrame(source_frames[i]);
            std::string target_frame = cleanTfFrame(target_frame_);

            tf_pair.setSourceFrame(source_frame);
            tf_pair.setTargetFrame(target_frame);
            tf_pair.setAngularThres(angular_thres);
            tf_pair.setTransThres(trans_thres);
        }
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const tf2_web_republisher_msgs::action::TFSubscription::Goal> goal) {
        RCLCPP_DEBUG(rclcpp::get_logger("tf2_web_republisher"), "GoalHandle request received");

        // get goal from handle

        // generate goal_info struct
        boost::shared_ptr<ClientGoalInfo> goal_info = boost::make_shared<ClientGoalInfo>();
        goal_info->handle = uuid;
        goal_info->client_ID_ = client_ID_count_++;

        // add the tf_subscriptions to the ClientGoalInfo object
        setSubscriptions(goal_info,
                         goal->source_frames,
                         goal->target_frame,
                         goal->angular_thres,
                         goal->trans_thres);

//        auto duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
//                std::chrono::duration<double>((1.0 / goal->rate)));
//        goal_info->timer_ = create_wall_timer(duration, [this, goal_info]() { TFRepublisher::processGoal(goal_info); });

        {
            boost::mutex::scoped_lock l(goals_mutex_);
            // add new goal to list of active goals/clients
            active_goals_.push_back(goal_info);
        }

        // accept new goals
        (void) uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tf2_web_republisher_msgs::action::TFSubscription>> goal_handle) {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&TFRepublisher::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tf2_web_republisher_msgs::action::TFSubscription>> goal_handle) {

    }

//    void unadvertiseCB(boost::shared_ptr<ClientRequestInfo> request_info) {
//        RCLCPP_INFO_STREAM(rclcpp::get_logger("tf2_web_republisher"), "No subscribers on tf topic for request "
//                                << request_info->client_ID_
//                                << " for " << request_info->unsub_timeout_.seconds()
//                                << " seconds. Unadvertising topic:"
//                                << request_info->pub_->get_topic_name());
//        request_info->pub_.shutdown();
//        request_info->unsub_timer_.stop();
//        request_info->timer_.stop();
//
//        // search for ClientRequestInfo struct and remove it from active_requests_ list
//        for (std::list<boost::shared_ptr<ClientRequestInfo> >::iterator it = active_requests_.begin();
//             it != active_requests_.end(); ++it) {
//            ClientRequestInfo &info = **it;
//            if (info.pub_ == request_info->pub_) {
//                active_requests_.erase(it);
//                return;
//            }
//        }
//    }

    void updateSubscriptions(std::vector<TFPair> &tf_subscriptions,
                             std::vector<geometry_msgs::msg::TransformStamped> &transforms) {
        // iterate over tf_subscription vector
        std::vector<TFPair>::iterator it;
        std::vector<TFPair>::const_iterator end = tf_subscriptions.end();

        for (it = tf_subscriptions.begin(); it != end; ++it) {
            geometry_msgs::msg::TransformStamped transform;

            try {
                // protecting tf_buffer
                boost::mutex::scoped_lock lock(tf_buffer_mutex_);

                // lookup transformation for tf_pair
                transform = tf_buffer_->lookupTransform(it->getTargetFrame(),
                                                        it->getSourceFrame(),
                                                        rclcpp::Time(0));

                // If the transform broke earlier, but worked now (we didn't get
                // booted into the catch block), tell the user all is well again
                if (!it->is_okay) {
                    it->is_okay = true;
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("tf2_web_republisher"), "Transform from "
                            << it->getSourceFrame()
                            << " to "
                            << it->getTargetFrame()
                            << " is working again at time "
                            << transform.header.stamp.sec);
                }
                // update tf_pair with transformtion
                it->updateTransform(transform);
            }
            catch (tf2::TransformException ex) {
                // Only log an error if the transform was okay before
                if (it->is_okay) {
                    it->is_okay = false;
                    RCLCPP_ERROR(rclcpp::get_logger("tf2_web_republisher"),"%s", ex.what());
                }
            }

            // check angular and translational thresholds
            if (it->updateNeeded()) {
                transform.header.stamp = now();
                transform.header.frame_id = it->getTargetFrame();
                transform.child_frame_id = it->getSourceFrame();

                // notify tf_subscription that a network transmission has been triggered
                it->transmissionTriggered();

                // add transform to the array
                transforms.push_back(transform);
            }
        }
    }

//    void processGoal(boost::shared_ptr<ClientGoalInfo> goal_info) {
//        tf2_web_republisher::TFSubscriptionFeedback feedback;
//
//        updateSubscriptions(goal_info->tf_subscriptions_,
//                            feedback.transforms);
//
//        if (feedback.transforms.size() > 0) {
//            // publish feedback
//            goal_info->handle.publishFeedback(feedback);
//            RCLCPP_DEBUG(rclcpp::get_logger("tf2_web_republisher"), "Client %d: TF feedback published:", goal_info->client_ID_);
//        } else {
//            RCLCPP_DEBUG(rclcpp::get_logger("tf2_web_republisher"), "Client %d: No TF frame update needed:", goal_info->client_ID_);
//        }
//    }

//    void processRequest(boost::shared_ptr<ClientRequestInfo> request_info, const ros::TimerEvent &) {
//        if (request_info->pub_.getNumSubscribers() == 0) {
//            request_info->unsub_timer_.start();
//        } else {
//            request_info->unsub_timer_.stop();
//        }
//
//        tf2_web_republisher::TFArray array_msg;
//        updateSubscriptions(request_info->tf_subscriptions_,
//                            array_msg.transforms);
//
//        if (array_msg.transforms.size() > 0) {
//            // publish TFs
//            request_info->pub_->publish(array_msg);
//            RCLCPP_DEBUG(rclcpp::get_logger("tf2_web_republisher"), "Request %d: TFs published:",
//                         request_info->client_ID_);
//        } else {
//            RCLCPP_DEBUG(rclcpp::get_logger("tf2_web_republisher"), "Request %d: No TF frame update needed:",
//                         request_info->client_ID_);
//        }
//    }

private:
    rclcpp_action::Server<tf2_web_republisher_msgs::action::TFSubscription>::SharedPtr action_server_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto tf2_web_republisher = std::make_shared<TFRepublisher>("tf2_web_republisher");

    rclcpp::spin(tf2_web_republisher);

    return 0;
}