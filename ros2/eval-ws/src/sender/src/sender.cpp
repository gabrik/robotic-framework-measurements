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
#include <thread>
#include "sender/sender.hpp"
#include "eval_interfaces/msg/evaluation.hpp"


using ato::sender::Sender;

Sender::Sender(const uint64_t msgs, const uint64_t size) : rclcpp::Node("sender_node", rclcpp::NodeOptions().use_intra_process_comms(false)) {
    auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable();
    this->printable_pace = 1.0/double(msgs);
    this->size = size;
    this->data.reserve(size);

    for (uint64_t i=0; i<size; i++) {
        this->data.push_back(100);
    }

    this->pace = std::chrono::duration<double>(this->printable_pace);
    this->publisher = this->create_publisher<eval_interfaces::msg::Evaluation>("ping", qos);
    this->subscriber = this->create_subscription<eval_interfaces::msg::Evaluation>("pong", qos, std::bind(&Sender::receiver_callback, this, std::placeholders::_1));
    this->timer = this->create_wall_timer(pace, std::bind(&Sender::publish_message, this));

    // RCLCPP_INFO(this->get_logger(), "Init Sender msg/s %d pace is %f", msgs, pace);

}


void Sender::publish_message() {

        auto message = eval_interfaces::msg::Evaluation();
        auto ts = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());
        message.emitter_ts = ts.count();
        message.payload = this->data;

        //RCLCPP_INFO(this->get_logger(), "Publish!, %ul", message.emitter_ts);
        this->publisher->publish(message);


}

void Sender::receiver_callback(const eval_interfaces::msg::Evaluation::SharedPtr msg){

    auto ts = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    auto latency = ts - msg->emitter_ts;
    // RCLCPP_INFO(this->get_logger(), "Receiver!, %ul - %ul = %dus", msg->emitter_ts, ts, latency);

    // <protocol>,[latency|througput],[interval|payload],<value>,<unit>, <timestamp>, <size>,<qos>
    std::cout << "ros2,latency," << this->printable_pace << "," << latency << ",us," << ts "," << this->size << ",3" << std::endl << std::flush;

    // std::this_thread::sleep_for(this->pace);
    // this->publish_message();
}