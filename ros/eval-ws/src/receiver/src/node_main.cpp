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
#include "receiver/receiver.hpp"

int main(int argc, char **argv) {


    std::string input = "ping";

    ros::init(argc, argv, "receiver");

    ros::NodeHandle nh;

    auto sender = ato::ros_receiver::Receiver(input, nh);


    while(nh.ok()) {
        ros::spin();
    }

   return 0;
}