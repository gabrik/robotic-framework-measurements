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
#include "eval_interfaces/Thr.h"

#include <chrono>
#include <thread>

namespace ato {
    namespace ros_compute {
        class Compute {

            private:
                ros::NodeHandle nh;
                ros::Publisher publisher;
                ros::Subscriber subscriber;
                void receiver_callback(const eval_interfaces::Thr::ConstPtr& msg);
                void publish_message(const eval_interfaces::Thr::ConstPtr& msg);
                std::string listen_topic;
                std::string publish_topic;

            public:
                explicit Compute(const std::string listen_topic, const std::string publish_topic, ros::NodeHandle &nh);
                virtual ~Compute() {}

        };
    }
}