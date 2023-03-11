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
#include "receiver/receiver.hpp"

int main (int argc, char* argv[]) {
    std::string input = "ping";

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;


    auto receiver = std::make_shared<ato::receiver::Receiver>(input);
    executor.add_node(receiver);

    executor.spin();

    rclcpp::shutdown();

    return 0;

}