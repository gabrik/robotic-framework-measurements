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

#include <chrono>
#include <vector>
#include <iostream>

#include "eval_interfaces/msg/evaluation.hpp"
#include "receiver/receiver.hpp"


using ato::receiver::Receiver;

Receiver::Receiver(const std::string topic_name) : rclcpp::Node("receiver_node", rclcpp::NodeOptions().use_intra_process_comms(false)) {
    auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable();
    this->subscriber = this->create_subscription<eval_interfaces::msg::Evaluation>(topic_name, qos, std::bind(&Receiver::receiver_callback, this, std::placeholders::_1));
    this->publisher = this->create_publisher<eval_interfaces::msg::Evaluation>("pong", qos);

    // RCLCPP_INFO(this->get_logger(), "Init Receiver with msg/s %d, pipeline lenght is %d, topic name is %s", this->msgs,  this->pipeline_length, topic_name.c_str());

}


void Receiver::receiver_callback(const eval_interfaces::msg::Evaluation::SharedPtr msg) {
    //RCLCPP_INFO(this->get_logger(), "Publish!, %ul", message.emitter_ts);
    this->publisher->publish(*msg);

}