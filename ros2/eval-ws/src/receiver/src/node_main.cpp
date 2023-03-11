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

#include <argparse/argparse.hpp>
#include "receiver/receiver.hpp"

int main (int argc, char* argv[]) {


    argparse::ArgumentParser program("receiver");

    program.add_argument("msgs")
        .help("messages per second")
        .scan<'i', uint64_t>();

     program.add_argument("length")
        .help("length of pipeline")
        .scan<'i', uint64_t>();



    try {
        program.parse_args(argc, argv);
    }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        std::exit(1);
    }

    uint64_t msgs = program.get<uint64_t>("msgs");
    std::string input = "ping";
    uint64_t length = program.get<uint64_t>("length");


    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;


    auto receiver = std::make_shared<ato::receiver::Receiver>(msgs, input, length);
    executor.add_node(receiver);

    executor.spin();

    rclcpp::shutdown();

    return 0;

}