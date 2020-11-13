// Copyright 2020 Fraunhofer IPA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ACTION_BRIDGE__ACTION_BRIDGE_HPP_
#define ACTION_BRIDGE__ACTION_BRIDGE_HPP_

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h> //Need this for the goal state. Need a better solution
#ifdef __clang__
#pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

#include "factory_interface.hpp"

#include <algorithm>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <atomic>
#include <condition_variable>

namespace ros1_bridge
{

template <class ROS1_T, class ROS2_T>
class ActionFactory_1_2 : public ActionFactoryInterface
{
public:
    using ROS1Server = typename actionlib::ActionServer<ROS1_T>;
    using ROS1GoalHandle = typename actionlib::ActionServer<ROS1_T>::GoalHandle;
    using ROS1Goal = typename actionlib::ActionServer<ROS1_T>::Goal;
    using ROS1Feedback = typename actionlib::ActionServer<ROS1_T>::Feedback;
    using ROS1Result = typename actionlib::ActionServer<ROS1_T>::Result;
    using ROS2Goal = typename ROS2_T::Goal;
    using ROS2Feedback = typename ROS2_T::Feedback;
    using ROS2Result = typename ROS2_T::Result;
    using ROS2GoalHandle = typename rclcpp_action::ClientGoalHandle<ROS2_T>::SharedPtr;
    using ROS2ClientSharedPtr = typename rclcpp_action::Client<ROS2_T>::SharedPtr;
    using ROS2SendGoalOptions = typename rclcpp_action::Client<ROS2_T>::SendGoalOptions;

    virtual void create_server_client(ros::NodeHandle ros1_node,
                                      rclcpp::Node::SharedPtr ros2_node,
                                      const std::string action_name)
    {
        this->ros1_node_ = ros1_node;
        this->ros2_node_ = ros2_node;

        server_ = std::make_shared<ROS1Server>(ros1_node, action_name,
                                               std::bind(&ActionFactory_1_2::goal_cb, this, std::placeholders::_1),
                                               std::bind(&ActionFactory_1_2::cancel_cb, this, std::placeholders::_1),
                                               false);
        server_->start();
        client_ = rclcpp_action::create_client<ROS2_T>(ros2_node, action_name);
    }

    void cancel_cb(ROS1GoalHandle gh1)
    {
        // try to find goal and cancel it
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = goals_.find(gh1.getGoalID().id);
        if (it != goals_.end())
        {
            std::thread([handler = it->second]() mutable {
                handler->cancel();
            }).detach();
        }
    }

    void goal_cb(ROS1GoalHandle gh1)
    {
        const std::string goal_id = gh1.getGoalID().id;

        // create a new handler for the goal
        std::shared_ptr<GoalHandler> handler;
        handler.reset(new GoalHandler(gh1, client_));
        std::lock_guard<std::mutex> lock(mutex_);
        goals_.insert(std::make_pair(goal_id, handler));

        RCLCPP_INFO(ros2_node_->get_logger(), "Sending goal");
        std::thread([handler, goal_id, this]() mutable {
            // execute the goal remotely
            handler->handle();

            // clean-up
            std::lock_guard<std::mutex> lock(mutex_);
            goals_.erase(goal_id);
        }).detach();
    }

private:
    class GoalHandler
    {
    public:
        void cancel()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            canceled_ = true;
            if (gh2_)
            { // cancel goal if possible
                auto fut = client_->async_cancel_goal(gh2_);
            }
        }
        void handle()
        {
            auto goal1 = gh1_.getGoal();
            ROS2Goal goal2;
            translate_goal_1_to_2(*gh1_.getGoal(), goal2);

            if (!client_->wait_for_action_server(std::chrono::seconds(1)))
            {
                std::cout << "Action server not available after waiting" << std::endl;
                gh1_.setRejected();
                return;
            }

            // goal_response_callback signature changed after foxy, this implementation
            // works with both
            std::shared_future<ROS2GoalHandle> gh2_future;
            //Changes as per Dashing
            auto send_goal_ops = ROS2SendGoalOptions();
            send_goal_ops.goal_response_callback =
                [this, &gh2_future](auto gh2) mutable {
		    auto goal_handle = gh2_future.get();
                    if (!goal_handle)
                    {
                        gh1_.setRejected(); // goal was not accepted by remote server
                        return;
                    }

                    gh1_.setAccepted();

                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        gh2_ = goal_handle;

                        if (canceled_)
                        { // cancel was called in between
                            auto fut = client_->async_cancel_goal(gh2_);
                        }
                    }
                };

            send_goal_ops.feedback_callback =
                [this](ROS2GoalHandle, auto feedback2) mutable {
                    ROS1Feedback feedback1;
                    translate_feedback_2_to_1(feedback1, *feedback2);
                    gh1_.publishFeedback(feedback1);
                };

            // send goal to ROS2 server, set-up feedback
            gh2_future = client_->async_send_goal(goal2, send_goal_ops);

            auto future_result = client_->async_get_result(gh2_future.get());
            auto res2 = future_result.get();

            ROS1Result res1;
            translate_result_2_to_1(res1, *(res2.result));

            std::lock_guard<std::mutex> lock(mutex_);
            if (res2.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                gh1_.setSucceeded(res1);
            }
            else if (res2.code == rclcpp_action::ResultCode::CANCELED)
            {
                gh1_.setCanceled(res1);
            }
            else
            {
                gh1_.setAborted(res1);
            }
        }

        GoalHandler(ROS1GoalHandle &gh1, ROS2ClientSharedPtr &client)
            : gh1_(gh1), gh2_(nullptr), client_(client), canceled_(false) {}

    private:
        ROS1GoalHandle gh1_;
        ROS2GoalHandle gh2_;
        ROS2ClientSharedPtr client_;
        bool canceled_; // cancel was called
        std::mutex mutex_;
    };

    ros::NodeHandle ros1_node_;
    rclcpp::Node::SharedPtr ros2_node_;

    std::shared_ptr<actionlib::ActionServer<ROS1_T>> server_;
    ROS2ClientSharedPtr client_;

    std::mutex mutex_;
    std::map<std::string, std::shared_ptr<GoalHandler>> goals_;

    static void translate_goal_1_to_2(const ROS1Goal &, ROS2Goal &);
    static void translate_result_2_to_1(ROS1Result &, const ROS2Result &);
    static void translate_feedback_2_to_1(ROS1Feedback &, const ROS2Feedback &);
};

template <class ROS1_T, class ROS2_T>
class ActionFactory_2_1 : public ActionFactoryInterface
{
public:
    using ROS2ServerGoalHandle = typename rclcpp_action::ServerGoalHandle<ROS2_T>;
    using ROS1ClientGoalHandle = typename actionlib::ActionClient<ROS1_T>::GoalHandle;
    using ROS2Goal = typename ROS2_T::Goal;
    using ROS1Client = typename actionlib::ActionClient<ROS1_T>;
    using ROS1Goal = typename actionlib::ActionServer<ROS1_T>::Goal;
    using ROS1Feedback = typename actionlib::ActionServer<ROS1_T>::Feedback;
    using ROS1Result = typename actionlib::ActionServer<ROS1_T>::Result;
    using ROS1State = actionlib::SimpleClientGoalState; // There is no 'ClientGoalState'. Better solution?
    using ROS2Feedback = typename ROS2_T::Feedback;
    using ROS2Result = typename ROS2_T::Result;
    using ROS2ServerSharedPtr = typename rclcpp_action::Server<ROS2_T>::SharedPtr;
    using ROS2SendGoalOptions = typename rclcpp_action::Client<ROS2_T>::SendGoalOptions;

    virtual void create_server_client(ros::NodeHandle ros1_node,
                                      rclcpp::Node::SharedPtr ros2_node,
                                      const std::string action_name)
    {
        this->ros1_node_ = ros1_node;
        this->ros2_node_ = ros2_node;
        client_ = std::make_shared<ROS1Client>(ros1_node, action_name);

        server_ = rclcpp_action::create_server<ROS2_T>(ros2_node->get_node_base_interface(),
                                                       ros2_node->get_node_clock_interface(),
                                                       ros2_node->get_node_logging_interface(),
                                                       ros2_node->get_node_waitables_interface(),
                                                       action_name,
                                                       std::bind(&ActionFactory_2_1::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                                                       std::bind(&ActionFactory_2_1::handle_cancel, this, std::placeholders::_1),
                                                       std::bind(&ActionFactory_2_1::handle_accepted, this, std::placeholders::_1));
    }

    //ROS2 callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ROS2Goal> goal)
    {
        (void)uuid;
        (void)goal;
        if (!client_->waitForActionServerToStart(ros::Duration(1)))
        {
            std::cout << "Action server not available after waiting" << std::endl;
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        std::shared_ptr<ROS2ServerGoalHandle> gh2)
    {
        // try to find goal and cancel it
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = goals_.find(get_goal_id_hash(gh2->get_goal_id()));
        if (it != goals_.end())
        {
            std::thread([handler = it->second]() mutable {
                handler->cancel();
            })
                .detach();
        }

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(std::shared_ptr<ROS2ServerGoalHandle> gh2)
    {
        std::size_t goal_id = get_goal_id_hash(gh2->get_goal_id());
        std::shared_ptr<GoalHandler> handler;
        handler.reset(new GoalHandler(gh2, client_));
        std::lock_guard<std::mutex> lock(mutex_);
        goals_.insert(std::make_pair(goal_id, handler));

        RCLCPP_INFO(ros2_node_->get_logger(), "Sending goal");
        std::thread([handler, goal_id, this]() mutable {
            // execute the goal remotely
            handler->handle();

            // clean-up
            std::lock_guard<std::mutex> lock(mutex_);
            goals_.erase(goal_id);
        })
            .detach();
    }

private:
    class GoalHandler
    {
    public:
        void cancel()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            canceled_ = true;
            if (gh1_)
            {
                // cancel goal if possible
                gh1_->cancel();
            }
        }
        void handle()
        {
            auto goal2 = gh2_->get_goal();
            ROS1Goal goal1;
            translate_goal_2_to_1(*goal2, goal1);

            if (gh2_->is_canceling())
            {
                auto result = std::make_shared<ROS2Result>();
                gh2_->canceled(result);
                return;
            }

            std::condition_variable cond_result;
            std::atomic<bool> result_ready(false);

            auto goal_handle = client_->sendGoal(
                goal1,
                [this, &result_ready, &cond_result](ROS1ClientGoalHandle goal_handle) mutable // transition_cb
                {
                    ROS_INFO("Goal [%s]", goal_handle.getCommState().toString().c_str());
                    if (goal_handle.getCommState() == actionlib::CommState::RECALLING)
                    {
                        //cancelled before being processed
                        auto result2 = std::make_shared<ROS2Result>();
                        gh2_->canceled(result2);
                        return;
                    }
                    else if (goal_handle.getCommState() == actionlib::CommState::ACTIVE)
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        gh1_ = std::make_shared<ROS1ClientGoalHandle>(goal_handle);
                    }
                    else if (goal_handle.getCommState() == actionlib::CommState::DONE)
                    {
                        auto result2 = std::make_shared<ROS2Result>();
                        auto result1 = goal_handle.getResult();
                        translate_result_1_to_2(*result2, *result1);
                        ROS_INFO("Goal [%s]", goal_handle.getTerminalState().toString().c_str());
                        if (goal_handle.getTerminalState() == actionlib::TerminalState::SUCCEEDED)
                        {
                            gh2_->succeed(result2);
                        }
                        else
                        {
                            gh2_->abort(result2);
                        }
                        result_ready = true;
                        cond_result.notify_one();
                        return;
                    }
                },
                [this](ROS1ClientGoalHandle goal_handle, auto feedback1) mutable //feedback_cb
                {
                    (void)goal_handle;
                    auto feedback2 = std::make_shared<ROS2Feedback>();
                    translate_feedback_1_to_2(*feedback2, *feedback1);
                    gh2_->publish_feedback(feedback2);
                });

            std::mutex mutex_result;
            std::unique_lock<std::mutex> lck(mutex_result);
            cond_result.wait(lck, [&result_ready] {
                return result_ready.load();
            });
        }

        GoalHandler(std::shared_ptr<ROS2ServerGoalHandle> &gh2, std::shared_ptr<ROS1Client> &client)
            : gh2_(gh2), client_(client), canceled_(false) {}

    private:
        std::shared_ptr<ROS1ClientGoalHandle> gh1_;
        std::shared_ptr<ROS2ServerGoalHandle> gh2_;
        std::shared_ptr<ROS1Client> client_;
        bool canceled_; // cancel was called
        std::mutex mutex_;
    };

    std::size_t get_goal_id_hash(const rclcpp_action::GoalUUID &uuid)
    {
        return std::hash<rclcpp_action::GoalUUID>{}(uuid);
    }

    ros::NodeHandle ros1_node_;
    rclcpp::Node::SharedPtr ros2_node_;

    std::shared_ptr<ROS1Client> client_;
    ROS2ServerSharedPtr server_;

    std::mutex mutex_;
    std::map<std::size_t, std::shared_ptr<GoalHandler>> goals_;

    static void translate_goal_2_to_1(const ROS2Goal &, ROS1Goal &);
    static void translate_result_1_to_2(ROS2Result &, const ROS1Result &);
    static void translate_feedback_1_to_2(ROS2Feedback &, const ROS1Feedback &);
};

} // namespace ros1_bridge

#endif // ACTION_BRIDGE__ACTION_BRIDGE_HPP_
