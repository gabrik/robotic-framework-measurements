mod opts;

use anyhow::{anyhow, Result}; // ensure, Context,
use clap::Parser;
use kafka_test::{AsyncStdFutureProducer, AsyncStdStreamConsumer, DEFAULT_GROUP_ID};
use log::{error, info, trace};
// use once_cell::sync::Lazy;
use opts::Opts;
use rdkafka::{
    consumer::Consumer, error::KafkaError, producer::FutureRecord, types::RDKafkaErrorCode,
    ClientConfig, Message,
};
use serde::{Serialize, Deserialize};
use std::{
    process,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

// static SINCE: Lazy<SystemTime> = Lazy::new(SystemTime::now);
// const MIN_PAYLOAD_SIZE: usize = 16;

// struct PayloadInfo {
//     pub ping_id: u32,
//     pub msg_idx: u32,
//     pub rtt: Duration,
// }

#[async_std::main]
async fn main() -> Result<()> {
    pretty_env_logger::init();

    let opts = Opts::parse();
    let timeout = opts.timeout;

    let future = run_latency_benchmark(opts);
    if let Some(timeout) = timeout {
        async_std::future::timeout(timeout, future)
            .await
            .map_err(|_| anyhow!("timeout"))??;
    } else {
        future.await?;
    }

    Ok(())
}

async fn run_latency_benchmark(opts: Opts) -> Result<()> {
    let ping_id = process::id();
    let client_config = {
        let mut conf = ClientConfig::new();
        conf.set("bootstrap.servers", &opts.brokers);
        conf
    };
    info!("Start ping {}", ping_id);

    run_ping_pong(&opts, &client_config, ping_id).await?;

    Ok(())
}

fn generate_payload() -> Vec<u8> {
// fn generate_payload(size: usize, ping_id: u32, msg_idx: u32) -> Vec<u8> {
    // assert!(
    //     size >= MIN_PAYLOAD_SIZE,
    //     "The minimum payload size is {} bytes",
    //     MIN_PAYLOAD_SIZE
    // );
    // let since = *SINCE; // the SINCE is inited earlier than the next SystemTime::now()
    // let dur = SystemTime::now().duration_since(since).unwrap();
    // let micros = dur.as_micros() as u64;

    // let ping_id_bytes = ping_id.to_le_bytes();
    // let msg_idx_bytes = msg_idx.to_le_bytes();
    // let time_bytes = micros.to_le_bytes();

    // let mut payload = vec![0u8; size];
    // payload[0..4].copy_from_slice(&ping_id_bytes);
    // payload[4..8].copy_from_slice(&msg_idx_bytes);
    // payload[8..16].copy_from_slice(&time_bytes);
    // payload
    let lat_data = LatData{ts: get_epoch_us()};
    bincode::serialize(&lat_data).unwrap()
}

fn parse_payload(payload: &[u8]) -> (u128,u128) { //, expect_payload_size: usize) -> u128 {//Result<PayloadInfo> {

    let data = bincode::deserialize::<LatData>(&payload).unwrap();
    let now = get_epoch_us();
    let elapsed = now - data.ts;
    (now, elapsed)
    // let payload_size = payload.len();
    // ensure!(
    //     payload_size >= MIN_PAYLOAD_SIZE,
    //     "The payload size ({} bytes) is less than the required minimum {} bytes",
    //     payload_size,
    //     MIN_PAYLOAD_SIZE
    // );

    // let ping_id_bytes = &payload[0..4];
    // let msg_idx_bytes = &payload[4..8];
    // let time_bytes = &payload[8..16];

    // let micros = u64::from_le_bytes(time_bytes.try_into().unwrap());
    // let since = *SINCE + Duration::from_micros(micros);
    // let rtt = SystemTime::now()
    //     .duration_since(since)
    //     .with_context(|| "the timestamp goes backward")?;

    // let ping_id = u32::from_le_bytes(ping_id_bytes.try_into().unwrap());
    // let msg_idx = u32::from_le_bytes(msg_idx_bytes.try_into().unwrap());

    // ensure!(
    //     payload.len() == expect_payload_size,
    //     "Expect payload size to be {} bytes, but get {} bytes",
    //     expect_payload_size,
    //     payload_size
    // );

    // Ok(PayloadInfo {
    //     msg_idx,
    //     rtt,
    //     ping_id,
    // })
}

fn create_producer(opts: &Opts, mut config: ClientConfig) -> Result<AsyncStdFutureProducer> {
    if let Some(cfgs) = &opts.producer_configs {
        cfgs.iter().for_each(|kv| {
            config.set(&kv.key, &kv.val);
        });
    }

    let producer: AsyncStdFutureProducer = config.create()?;
    Ok(producer)
}

fn create_consumer(
    opts: Opts,
    mut config: ClientConfig,
    topic: &str,
) -> Result<AsyncStdStreamConsumer> {
    config
        .set("group.id", DEFAULT_GROUP_ID)
        .set("session.timeout.ms", "6000");

    if let Some(configs) = &opts.consumer_configs {
        for kv in configs {
            config.set(&kv.key, &kv.val);
        }
    }

    let consumer: AsyncStdStreamConsumer = config.create()?;
    consumer.subscribe(&[topic])?;
    Ok(consumer)
}

async fn run_ping_pong(opts: &Opts, client_config: &ClientConfig, ping_id: u32) -> Result<()> {
    let producer: AsyncStdFutureProducer = create_producer(opts, client_config.clone())?;

    let c_opts = opts.clone();
    let c_client_config = client_config.clone();
    async_std::task::spawn(async move {
        let mut consumer = create_consumer(c_opts.clone(), c_client_config.clone(), &c_opts.pong_topic).unwrap();
        loop {
        if !recv(&c_opts, &c_client_config, &mut consumer).await.unwrap() {
            panic!("Failed to receive pong message.");
        }
    }});

    for count in 0.. {
        send(opts, &producer, ping_id, count).await?;
        async_std::task::sleep(Duration::from_secs_f64(opts.interval)).await;
    }
    Ok(())
}

async fn send(
    opts: &Opts,
    producer: &AsyncStdFutureProducer,
    ping_id: u32,
    msg_idx: u32,
) -> Result<()> {
    let record_key = ping_id.to_le_bytes();
    // let payload = generate_payload(opts.payload_size, ping_id, msg_idx);
    let payload = generate_payload();
    let record = FutureRecord::to(&opts.ping_topic)
        .payload(&payload)
        .key(&record_key);

    trace!(
        "Send a ping with ping_id {} and msg_idx {}",
        ping_id,
        msg_idx
    );
    producer
        .send(record, Duration::ZERO)
        .await
        .map_err(|(err, _msg)| err)?;
    Ok(())
}

async fn recv(
    opts: &Opts,
    client_config: &ClientConfig,
    // ping_id: u32,
    consumer: &mut AsyncStdStreamConsumer,
) -> Result<bool> {
    use KafkaError as E;
    use RDKafkaErrorCode as C;

    const RECV_TIMEOUT: Duration = Duration::from_secs(1);

    let recv = consumer.recv();
    let result = async_std::future::timeout(RECV_TIMEOUT, recv).await;

    match result {
        Ok(Ok(msg)) => {
            let payload = match msg.payload() {
                Some(payload) => payload,
                None => {
                    error!("Ignore a message without payload");
                    return Ok(false);
                }
            };

            // let info = match parse_payload(payload, opts.payload_size) {
            //     Ok(info) => info,
            //     Err(err) => {
            //         error!("Unable to parse payload: {:#}", err);
            //         return Ok(false);
            //     }
            // };

            // trace!(
            //     "Received a pong with ping_id {} and msg_idx {}",
            //     info.ping_id,
            //     info.msg_idx
            // );
            let (now, rtt) = parse_payload(payload);//, opts.payload_size);
            // ensure!(
            //     info.ping_id == ping_id,
            //     "Ignore the payload from a foreign ping ID {}",
            //     info.ping_id
            // );
            // <protocol>,[lantecy|througput],[interval|payload],<value>,<unit>,<ts>
            println!("kafka,latency,{},{},us,{}", opts.interval, rtt, now);

            Ok(true)
        }
        Ok(Err(E::MessageConsumption(C::UnknownTopicOrPartition))) => {
            trace!(
                "The topic {} is not created yet, retry again",
                opts.pong_topic
            );
            async_std::task::sleep(Duration::from_secs(1)).await;
            *consumer = create_consumer(opts.clone(), client_config.clone(), &opts.pong_topic)?;
            Ok(true)
        }
        Ok(Err(err)) => Err(err.into()),
        Err(_) => {
            trace!("Timeout receiving a message");
            Ok(true)
        }
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
