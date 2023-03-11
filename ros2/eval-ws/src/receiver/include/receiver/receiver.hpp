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

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <vector>
#include "eval_interfaces/msg/evaluation.hpp"

namespace ato {
    namespace receiver {
        class Receiver : public rclcpp::Node {

            public:
                explicit Receiver(const std::string topic_name);
                virtual ~Receiver() {};

            private:
                rclcpp::Subscription<eval_interfaces::msg::Evaluation>::SharedPtr subscriber;
                rclcpp::Publisher<eval_interfaces::msg::Evaluation>::SharedPtr publisher;
                void receiver_callback(const eval_interfaces::msg::Evaluation::SharedPtr msg);

        };
    }
}