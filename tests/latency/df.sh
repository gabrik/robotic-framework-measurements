#!/usr/bin/env bash



plog () {
TS=`eval date "+%F-%T"`
   echo "[$TS]: $1"
}

usage() { printf "Usage: $0 <config> \n\t
    -z Zenoh\n\t
    -m MQTT\n\t
    -k Kafka\n]t
    -r ROS2\n\t
    -R ROS\n" 1>&2; exit 1; }

source conf/default.sh


# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

function ctrl_c() {
    cleanup
    exit
}

function ros_cleanup() {
    # ROS
    ssh $ROS_MASTER "
        killall -2 roscore
    "

    ssh $ROS_MASTER "
        killall -2 rosmaster
    "

    ssh $ROS_PING "
        killall -9 sender_node
    "

    ssh $ROS_PONG "
        killall -9 receiver_node
    "
}

function ros2_cleanup() {
    # ROS2
    ssh $ROS2_PING "
        killall -9 sender_ros
    "

    ssh $ROS2_PONG "
        killall -9 receiver_ros
    "
}

function z_cleanup () {
     # ZF
    ssh $Z_ROUTER "
        killall -9 zenohd
    "

    ssh $Z_PING "
        killall -9 zenoh_ping
    "

    ssh $Z_PONG "
        killall -9 zenoh_pong
    "
}

# kills all the processes
function cleanup() {
    ros_cleanup
    ros2_cleanup
    z_cleanup
}


# Prepare the tests
if [ -z "$1" ] || [ ! -f "$1" ] ; then
    echo "Provide a test configuration file: [ conf/100GbE.sh | conf/localhost.sh ]"
    exit
fi

source $1
shift

# BUILD and install
PREPARE_TEST_ENV_CMD="
source ~/.profile
source ~/.bashrc

killall -9 cargo

mkdir -p $WORKING_DIR
cd $WORKING_DIR
rm -rf $PERF_DIR
git clone $PERF_GIT
cd $PERF_DIR

make
"

# Preparing nodes, using ROS handles but machine are expected to be the same
# plog "[ INIT ] Preparing $ROS_SRC"
# ssh $ROS_SRC "$PREPARE_TEST_ENV_CMD" &
# SRC_PID=$!
# plog "[ INIT ] Preparing $ROS_OP"
# ssh $ROS_OP "$PREPARE_TEST_ENV_CMD" &
# OP_PID=$!
# plog "[ INIT ] Preparing $ROS_SNK"
# ssh $ROS_SNK "$PREPARE_TEST_ENV_CMD" &
# SNK_PID=$!

# wait $SRC_PID
# wait $OP_PID
# wait $SNK_PID

# Latency logs
if [ ! -d "$TEST_LOGS" ]; then
    mkdir -p $TEST_LOGS
fi

while getopts "hzrRmk" arg; do
    case ${arg} in
    h)
        usage
        ;;
    z)
        plog "[ INIT ] Testing Zenoh"
        s=$INITIAL_MSGS
        while [ $s -le $FINAL_MSGS ]
        do
            plog "[ START ] Zenoh Starting testing for $s msg/s"

            plog "[ START ] Zenoh Starting pong"
            ssh -f $Z_PONG "
                cd $WORKING_DIR/$PERF_DIR
                LISTEN=$LOCATOR_PONG CONNECT=$LOCATOR_PING NICE=$Z_PONG_NICE CPUS=$Z_PONG_CPUS ./run-single-process.sh -oz
            "
            PONG_PID=$!

            sleep 1
            plog "[ START ] Zenoh Starting ping"
            ssh $Z_PING "
                cd $WORKING_DIR/$PERF_DIR
                LISTEN=$LOCATOR_PING CONNECT=$LOCATOR_PONG DURATION=$TEST_TIME NICE=$Z_PING_NICE CPUS=$Z_PING_CPUS MSGS=$s ./run-single-process.sh -iz
            "
            PING_PID=$!
            sleep 1
            wait $PING_PID
            z_cleanup

            wait $PONG_PID

            rsync -azv "$Z_PING:~/$WORKING_DIR/$PERF_DIR/logs/*.csv" $TEST_LOGS/

            plog "[ DONE ] Zenoh Done testing for $s msg/s"
            s=$(($s * 10))
        done
        plog "[ DONE ] Testing Zenoh"
        ;;
    r)
        plog "[ INIT ] Testing ROS2"
        s=$INITIAL_MSGS
        while [ $s -le $FINAL_MSGS ]
        do
            plog "[ START ] ROS2 Starting testing for $s msg/s"

            plog "[ START ] ROS2 Starting pong"
            ssh -f $ROS2_PONG "
                cd $WORKING_DIR/$PERF_DIR
                ROS_DISTRO=foxy CYCLONEDDS_URI='$ROS2_PONG_CYCLONEDDS_URI' NICE=$ROS2_PONG_NICE CPUS=$ROS2_PONG_CPUS MSGS=$s ./run-single-process.sh -or
            "
            PONG_PID=$!
            sleep 1

            sleep 1
            plog "[ START ] ROS2 Starting ping"
            ssh $ROS2_PING "
                cd $WORKING_DIR/$PERF_DIR
                ROS_DISTRO=foxy CYCLONEDDS_URI='$ROS2_PING_CYCLONEDDS_URI' DURATION=$TEST_TIME NICE=$ROS2_PING_NICE CPUS=$ROS2_PING_CPUS MSGS=$s ./run-single-process.sh -ir
            "
            PING_PID=$!
            sleep 1
            wait $PING_PID
            ros2_cleanup

            wait $PONG_PID

            rsync -azv "$RO2_PING:~/$WORKING_DIR/$PERF_DIR/logs/*.csv" $TEST_LOGS/

            plog "[ DONE ] ROS2 Done testing for $s msg/s"
            s=$(($s * 10))
        done
        plog "[ DONE ] Testing ROS2"
        ;;
    R)
        plog "[ INIT ] Testing ROS"
        s=$INITIAL_MSGS
        while [ $s -le $FINAL_MSGS ]
        do
            plog "[ START ] ROS Starting testing for $s msg/s"

            plog "[ START ] ROS Starting ROS master"
            ssh -f $ROS_MASTER "
                cd $WORKING_DIR/$PERF_DIR
                ROS_DISTRO=noetic ROS_IP=$ROS_IP_MASTER ROS_MASTER_URI=$ROS_MASTER_URI NICE=$ROS_MASTER_NICE CPUS=$ROS_MASTER_CPUS ./run-single-process.sh -bR
            "
            MASTER_PID=$!
            sleep 3

            plog "[ START ] ROS Starting ROS pong"
            ssh -f $ROS_PONG "
                cd $WORKING_DIR/$PERF_DIR
                ROS_DISTRO=noetic ROS_IP=$ROS_IP_PONG ROS_MASTER_URI=$ROS_MASTER_URI NICE=$ROS_PONG_NICE CPUS=$ROS_PONG_CPUS ./run-single-process.sh -oR
            "
            PONG_PID=$!
            sleep 1
            plog "[ START ] ROS Starting ROS ping"
            ssh $ROS_PING "
                cd $WORKING_DIR/$PERF_DIR
                ROS_DISTRO=noetic ROS_IP=$ROS_IP_PING ROS_MASTER_URI=$ROS_MASTER_URI DURATION=$TEST_TIME NICE=$ROS_PING_NICE CPUS=$ROS_PING_CPUS MSGS=$s ./run-single-process.sh -iR
            "
            PING_PID=$!
            sleep 1
            wait $PING_PID
            ros_cleanup

            wait $MASTER_PID
            wait $PONG_PID

            rsync -azv "$ROS_PING:~/$WORKING_DIR/$PERF_DIR/logs/*.csv" $TEST_LOGS/

            plog "[ DONE ] ROS Done testing for $s msg/s"
            s=$(($s * 10))
        done
        plog "[ DONE ] Testing ROS"
        ;;
    k)
        # Kafka
        plog "[ WARN ] Kafka not yet implemented"
        ;;
    m)
        # MQTT
        plog "[ WARN ] MQTT not yet implemented"
        ;;
    *)
        usage
        ;;
    esac
done

plog "[ DONE ] Bye!"


