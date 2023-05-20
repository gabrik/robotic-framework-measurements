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
#include "sender/sender.hpp"
#include <chrono>
#include <thread>


using ato::ros_sender::Sender;

Sender::Sender(const uint64_t msgs, const uint64_t size, ros::NodeHandle &nh) {
    this->nh = nh;
    this->printable_pace = 1.0/double(msgs);
    this->size = size;
    this->data.reserve(size);

    for (uint64_t i=0; i<size; i++) {
        this->data.push_back(100);
    }

    this->pace = std::chrono::duration<double>(this->printable_pace);
    this->publisher = this->nh.advertise<eval_interfaces::Evaluation>("ping", 0);
    this->subscriber = this->nh.subscribe("pong", 0, &Sender::receiver_callback, this);

    this->timer = nh.createTimer(ros::Duration(this->printable_pace), &Sender::publish_message, this);

    // ROS_INFO("Init sender msg/s %ld", msgs);
}

void Sender::publish_message(const ros::TimerEvent& event) {

        eval_interfaces::Evaluation msg;

        auto ts = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());
        msg.emitter_ts = ts.count();
        msg.payload = this->data;

        // ROS_INFO("Publish! %zu ", msg.emitter_ts);
        this->publisher.publish(msg);


}

void Sender::receiver_callback(const eval_interfaces::Evaluation::ConstPtr& msg){
    auto ts = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    auto latency = ts - msg->emitter_ts;

    // <protocol>,[latency|througput],[interval|payload],<value>,<unit>,<timestamp>,<size>,<qos>
    std::cout << "ros,latency," << this->printable_pace << "," << latency << ",us," << ts << "," << this->size << ",3" << std::endl << std::flush;

    // std::this_thread::sleep_for(this->pace);
    // this->publish_message();
}