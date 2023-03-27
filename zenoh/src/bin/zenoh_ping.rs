//
// Copyright (c) 2022 ZettaScale Technology
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
//
use clap::Parser;
use std::path::PathBuf;
use std::time::Duration;
use std::sync::Arc;
use zenoh::config::Config;
use zenoh::prelude::{sync::*, CongestionControl};
use zenoh_config::{EndPoint, WhatAmI};
use std::time::{SystemTime, UNIX_EPOCH};
use serde::{Serialize, Deserialize};

#[derive(Debug, Parser)]
#[clap(name = "zenoh_ping")]
struct Opt {
    #[clap(short, long, value_delimiter = ',')]
    listen: Option<Vec<EndPoint>>,

    #[clap(short, long, value_delimiter = ',')]
    connect: Option<Vec<EndPoint>>,

    /// peer, router, or client
    #[clap(short, long)]
    mode: WhatAmI,

    /// interval of sending message (sec)
    #[clap(short, long)]
    interval: f64,

    /// configuration file (json5 or yaml)
    #[clap(long = "conf", value_parser)]
    config: Option<PathBuf>,
}

fn main() {
    // initiate logging
    env_logger::init();

    // Parse the args
    let opt = Opt::parse();

    let mut config: Config = if let Some(path) = &opt.config {
        Config::from_file(path).unwrap()
    } else {
        Config::default()
    };
    config.set_mode(Some(opt.mode)).unwrap();
    match opt.mode {
        WhatAmI::Peer => {
            if let Some(endpoints) = opt.listen.clone() {
                config.listen.endpoints.extend(endpoints)
            }
            if let Some(endpoints) = opt.connect.clone() {
                config.connect.endpoints.extend(endpoints)
            }
        }
        WhatAmI::Client => {
            if let Some(endpoints) = opt.connect.clone() {
                config.connect.endpoints.extend(endpoints)
            }
        }
        _ => panic!("Unsupported mode: {}", opt.mode),
    };
    config.scouting.multicast.set_enabled(Some(false)).unwrap();

    let session = Arc::new(zenoh::open(config).res().unwrap());

    // The key expression to publish data on
    let key_expr_ping = keyexpr::new("test/ping").unwrap();

    // The key expression to wait the response back

    let c_session = session.clone();
    std::thread::spawn(move || {receiving(c_session.clone(), opt.interval)});

    let publisher = session
        .declare_publisher(key_expr_ping)
        .congestion_control(CongestionControl::Block)
        .res()
        .unwrap();

    // let data: Value = (0usize..opt.payload)
    //     .map(|i| (i % 10) as u8)
    //     .collect::<Vec<u8>>()
    //     .into();

    let sleep_interval = Duration::from_secs_f64(opt.interval);

    // let mut data = Vec::with_capacity(20);

    loop {
        let lat_data = LatData{ts: get_epoch_us()};
        // bincode::serialize_into(&mut data, &lat_data).unwrap();
        let data = bincode::serialize(&lat_data).unwrap();
        // let instant = Instant::now();
        publisher.put(&*data).res().unwrap();
        std::thread::sleep(sleep_interval);

        // let _ = sub.recv();
        // // <protocol>,[latency|througput],[interval|payload],<value>,<unit>
        // println!("zenoh,latency,{},{},ns", opt.interval, instant.elapsed().as_micros() / 2);
    }
}


fn receiving(session : Arc<zenoh::Session>, interval: f64) {
    let key_expr_pong = keyexpr::new("test/pong").unwrap();
    let sub = session.declare_subscriber(key_expr_pong).res().unwrap();

    loop {
        let msg = sub.recv().unwrap();
        let now = get_epoch_us();
        let data = bincode::deserialize::<LatData>(&msg.value.payload.contiguous()).unwrap();
        let elapsed = now - data.ts;

        // <protocol>,[latency|througput],[interval|payload],<value>,<unit>
        println!("zenoh,latency,{},{},us", interval, elapsed / 2);
    }

}

pub fn get_epoch_us() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_micros()
}


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LatData {
    pub ts: u128,
}

