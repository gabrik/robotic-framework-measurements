//
// Copyright (c) 2017, 2022 ZettaScale Technology.
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ZettaScale zenoh team, <zenoh@zettascale.tech>
//

#include "ros/ros.h"
#include "eval_interfaces/Evaluation.h"
#include "receiver/receiver.hpp"
#include <chrono>
#include <thread>


using ato::ros_receiver::Receiver;



Receiver::Receiver(const std::string topic_name,ros::NodeHandle &nh) {
    this->nh = nh;
    this->subscriber = this->nh.subscribe(topic_name,1024, &Receiver::receiver_callback, this);
    this->publisher = this->nh.advertise<eval_interfaces::Evaluation>("pong", 0);

    // RCLCPP_INFO(this->get_logger(), "Init Receiver with msg/s %d, pipeline lenght is %d, topic name is %s", this->msgs,  this->pipeline_length, topic_name.c_str());

}


void Receiver::receiver_callback(const eval_interfaces::Evaluation::ConstPtr& msg) {

    //RCLCPP_INFO(this->get_logger(), "Publish!, %ul", message.emitter_ts);
    this->publisher.publish(msg);

}