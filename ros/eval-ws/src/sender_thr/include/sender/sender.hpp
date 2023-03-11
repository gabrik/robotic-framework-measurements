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


namespace ato {
    namespace ros_sender {
        class Sender {

            private:
                ros::NodeHandle nh;
                ros::Publisher publisher;
                std::vector<uint8_t> data;

            public:
                explicit Sender(const uint64_t size, ros::NodeHandle &nh);
                virtual ~Sender() {}
                void publish_message();



        };
    }
}