#!/usr/bin/env bash



plog () {
TS=`eval date "+%F-%T"`
   echo "[$TS]: $1"
}

usage() { printf "Usage: $0 <config> \n\t
   -z Zenoh Flow\n\t
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

    ssh $ROS_SRC "
        killall -9 sender_node_thr
    "

    ssh $ROS_OP "
        killall -9 compute_node_thr
    "

    ssh $ROS_SNK "
        killall -9 receiver_node_thr
    "
}

function ros2_cleanup() {
    # ROS2
    ssh $ROS2_SRC "
        killall -9 sender_ros_thr
    "

    ssh $ROS2_OP "
        killall -9 compute_ros_thr
    "

    ssh $ROS2_SNK "
        killall -9 receiver_ros_thr
    "
}

function zf_cleanup () {
     # ZF
    ssh $ZF_SRC "
        killall -9 thr-dynamic
    "

    ssh $ZF_OP "
        killall -9 thr-dynamic
    "

    ssh $ZF_SNK "
        killall -9 thr-dynamic
    "
}

# kills all the processes
function cleanup() {
    ros_cleanup
    ros2_cleanup
    zf_cleanup
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
rm -rf $ZENOH_FLOW_PERF_DIR
git clone $ZENOH_FLOW_PERF_GIT
cd $ZENOH_FLOW_PERF_DIR

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

while getopts "hzrR" arg; do
    case ${arg} in
    h)
        usage
        ;;
    z)
        plog "[ INIT ] Testing Zenoh Flow"
        s=$INITIAL_SIZE
        while [ $s -le $FINAL_SIZE ]
        do
            plog "[ START ] Zenoh Flow Starting testing for $s bytes"

            plog "[ START ] Zenoh Flow Starting sink"
            ssh -f $ZF_SNK "
                cd $WORKING_DIR/$ZENOH_FLOW_PERF_DIR
                LISTEN=$LOCATOR_SNK CONNECT=$LOCATOR_OP NICE=$ZF_SNK_NICE CPUS=$ZF_SNK_CPUS SIZE=$s ./run-single-process-thr.sh -ez
            "
            SNK_PID=$!
            sleep 1

            plog "[ START ] Zenoh Flow Starting operator"
            ssh -f $ZF_OP "
                cd $WORKING_DIR/$ZENOH_FLOW_PERF_DIR
                LISTEN=$LOCATOR_OP CONNECT=$LOCATOR_OP_SNK NICE=$ZF_OP_NICE CPUS=$ZF_OP_CPUS SIZE=$s ./run-single-process-thr.sh -oz
            "
            OP_PID=$!
            sleep 1
            plog "[ START ] Zenoh Flow Starting source"
            ssh $ZF_SRC "
                cd $WORKING_DIR/$ZENOH_FLOW_PERF_DIR
                LISTEN=$LOCATOR_SRC CONNECT=$LOCATOR_SRC_OP DURATION=$TEST_TIME NICE=$ZF_SRC_NICE CPUS=$ZF_SRC_CPUS SIZE=$s ./run-single-process-thr.sh -iz
            "
            SRC_PID=$!
            sleep 1
            wait $SRC_PID
            zf_cleanup

            wait $SNK_PID
            wait $OP_PID

            rsync -azv "$ZF_SRC:~/$WORKING_DIR/$ZENOH_FLOW_PERF_DIR/breakdown-logs/*.csv" $TEST_LOGS/

            plog "[ DONE ] Zenoh Flow Done testing for $s bytes"
            s=$(($s * 2))
        done
        plog "[ DONE ] Testing Zenoh Flow"
        ;;
    r)
        plog "[ INIT ] Testing ROS2"
        s=$INITIAL_SIZE
        while [ $s -le $FINAL_SIZE ]
        do
            plog "[ START ] ROS2 Starting testing for $s bytes"

            plog "[ START ] ROS2 Starting sink"
            ssh -f $ROS2_SNK "
                cd $WORKING_DIR/$ZENOH_FLOW_PERF_DIR
                CYCLONEDDS_URI='$ROS2_SNK_CYCLONEDDS_URI' NICE=$ROS2_SNK_NICE CPUS=$ROS2_SNK_CPUS SIZE=$s ./run-single-process-thr.sh -er
            "
            SNK_PID=$!
            sleep 1

            plog "[ START ] ROS2 Starting operator"
            ssh -f $ROS2_OP "
                cd $WORKING_DIR/$ZENOH_FLOW_PERF_DIR
                CYCLONEDDS_URI='$ROS2_OP_CYCLONEDDS_URI' NICE=$ROS2_OP_NICE CPUS=$ROS2_OP_CPUS SIZE=$s ./run-single-process-thr.sh -or
            "
            OP_PID=$!
            sleep 1
            plog "[ START ] ROS2 Starting source"
            ssh $ROS2_SRC "
                cd $WORKING_DIR/$ZENOH_FLOW_PERF_DIR
                CYCLONEDDS_URI='$ROS2_SRC_CYCLONEDDS_URI' DURATION=$TEST_TIME NICE=$ROS2_SRC_NICE CPUS=$ROS2_SRC_CPUS SIZE=$s ./run-single-process-thr.sh -ir
            "
            SRC_PID=$!
            sleep 1
            wait $SRC_PID
            ros2_cleanup

            wait $SNK_PID
            wait $OP_PID

            rsync -azv "$ROS2_SNK:~/$WORKING_DIR/$ZENOH_FLOW_PERF_DIR/breakdown-logs/*.csv" $TEST_LOGS/

            plog "[ DONE ] ROS2 Done testing for $s bytes"
            s=$(($s * 2))
        done
        plog "[ DONE ] Testing ROS2"
        ;;
    R)
        plog "[ INIT ] Testing ROS"
        s=$INITIAL_SIZE
        while [ $s -le $FINAL_SIZE ]
        do
            plog "[ START ] ROS Starting testing for $s bytes"

            plog "[ START ] ROS Starting ROS master"
            ssh -f $ROS_MASTER "
                cd $WORKING_DIR/$ZENOH_FLOW_PERF_DIR
                ROS_IP=$ROS_IP_MASTER ROS_MASTER_URI=$ROS_MASTER_URI NICE=$ROS_MASTER_NICE CPUS=$ROS_MASTER_CPUS ./run-single-process-thr.sh -mR
            "
            RMASTER_PID=$!
            sleep 3

            plog "[ START ] ROS Starting ROS sink"
            ssh -f $ROS_SNK "
                cd $WORKING_DIR/$ZENOH_FLOW_PERF_DIR
                ROS_IP=$ROS_IP_SRC ROS_MASTER_URI=$ROS_MASTER_URI NICE=$ROS_SNK_NICE CPUS=$ROS_SNK_CPUS SIZE=$s ./run-single-process-thr.sh -eR
            "
            RSNK_PID=$!
            sleep 1

            plog "[ START ] ROS Starting ROS operator"
            ssh -f $ROS_OP "
                cd $WORKING_DIR/$ZENOH_FLOW_PERF_DIR
                ROS_IP=$ROS_IP_OP ROS_MASTER_URI=$ROS_MASTER_URI NICE=$ROS_OP_NICE CPUS=$ROS_OP_CPUS SIZE=$s ./run-single-process-thr.sh -oR
            "
            ROP_PID=$!
            sleep 1
            plog "[ START ] ROS Starting ROS source"
            ssh $ROS_SRC "
                cd $WORKING_DIR/$ZENOH_FLOW_PERF_DIR
                ROS_IP=$ROS_IP_SNK ROS_MASTER_URI=$ROS_MASTER_URI DURATION=$TEST_TIME NICE=$ROS_OP_NICE CPUS=$ROS_SRC_CPUS SIZE=$s ./run-single-process-thr.sh -iR
            "
            RSRC_PID=$!
            sleep 1
            wait $RSRC_PID
            ros_cleanup

            wait $RMASTER_PID
            wait $RSNK_PID
            wait $ROP_PID

            rsync -azv "$ROS_SNK:~/$WORKING_DIR/$ZENOH_FLOW_PERF_DIR/breakdown-logs/*.csv" $TEST_LOGS/

            plog "[ DONE ] ROS Done testing for $s bytes"
            s=$(($s * 2))
        done
        plog "[ DONE ] Testing ROS"
        ;;
    *)
        usage
        ;;
    esac
done

plog "[ DONE ] Bye!"


