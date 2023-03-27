#!/usr/bin/env bash


plog () {
   LOG_TS=`eval date "+%F-%T"`
   echo "[$LOG_TS]: $1"
}

usage() { printf "Usage: $0 \n\t
   -i source\n\t
   -o dynamic\n\t
   -e sink\n\t
   -m ros_master\n\t
   -z Zenoh Flow\n\t
   -r ROS2\n\t
   -R ROS\n" 1>&2; exit 1; }



if [[ ! $@ =~ ^\-.+ ]]
then
  usage;
fi



TS=$(date +%Y%m%d.%H%M%S)

N_CPU=$(nproc)

BIN_DIR="./target/release"
ROS_BIN_DIR="./ros/eval-ws/build"

WD=$(pwd)

ZENOH_PONG="zenoh_pong"
ZENOH_PING="zenoh_ping"
KAFKA_PONG="kafka_pong"
KAFKA_PING="kafka_ping"

ROS2_COMPARISON_DIR="$WD/ros2/eval-ws"
MQTT_COMPARISON_DIR="$WD/mqtt/target"

ROS2_PING="ros2 run sender sender_ros"
ROS2_PONG="ros2 run receiver receiver_ros"

ROS_PING="sender/sender_node"
ROS_PONG="receiver/receiver_node"

MQTT_PING="mqtt_ping"
MQTT_PONG="mqtt_pong"

OUT_DIR="${OUT_DIR:-logs}"
MSGS=${MSGS:-1}
DURATION=${DURATION:-60}
SIZE=${SIZE:-8}
CPUS="${CPUS:-0,1}"
mkdir -p $OUT_DIR
NICE="${NICE:--10}"
ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
CYCLONEDDS_URI="${CYCLONEDDS_URI:-$WD/ros2/cyclonedds.xml}"
LISTEN="${LISTEN:-tcp/127.0.0.1:7447}"
CONNECT="${CONNECT:-tcp/127.0.0.1:7887}"
ROS_IP="${ROS_IP:-127.0.0.1}"
ROSDISTRO="${ROS_DISTRO:-foxy}"
ZENOHD="${ZENOHD:-/usr/bin/zenohd}"
MOSQUITTO="mqtt/docker-compose.yaml"
KAFKA="kafka/docker-compose.yaml"


# Run source by default:
# - 1 = Ping
# - 2 = Pong
# - 3 = Broker (i.e., ROS Master, Mosquitto, Kafka server, zenohd)
TORUN=1


plog "[ INIT ] Duration will be $DURATION seconds"
plog "[ INIT ] Sending rate will be $MSGS msg/s"
plog "[ INIT ] Size for throughput test will be $SIZE"
plog "[ INIT ] ROS_IP is $ROS_IP"
while getopts "iobzrRmk" arg; do
   case ${arg} in
   h)
      usage
      ;;
   i)
      # Start Ping

      plog "[ INIT ] Running a ping"
      TORUN=1
      ;;
   o)
      # Start pong

      plog "[ INIT ] Running an pong"
      TORUN=2
      ;;
   b)
      # Start the broker
      plog "[ INIT ] Running the broker"
      TORUN=3
      ;;
   k)
      #Kafka
      case ${TORUN} in
      1)
         plog "[ RUN ] Running Kafka ping with msg/s $MSGS"
         INTERVAL=$(bc -l <<< "1/$MSGS")
         LOG_FILE="$OUT_DIR/kafka-latency-$MSGS-$TS.csv"
         #echo "framework,test,metric,value,unit" > $LOG_FILE
         timeout $DURATION nice $NICE taskset -c $CPUS $BIN_DIR/$KAFKA_PING -i $INTERVAL -b $CONNECT -p 64 >> $LOG_FILE 2> /dev/null
         plog "[ DONE ] Running Kafka ping msg/s $MSGS, logged to $LOG_FILE"
         ;;
      2)
         plog "[ RUN ] Running Kafka pong"
         nice $NICE taskset -c $CPUS $BIN_DIR/$KAFKA_PONG -b $CONNECT > /dev/null 2>&1
         plog "[ DONE ] Running Kafka pong"
         ;;
      3)
         plog "[ RUN ] Running Kafka server"
         docker compose -f $KAFKA up > /dev/null 2>&1
         plog "[ DONE ] Running Kafka server"
         ;;
      *)
         usage
         ;;
      esac
      ;;
   m)
      #MQTT
      case ${TORUN} in
      1)
         plog "[ RUN ] Running MQTT ping with msg/s $MSGS"
         INTERVAL=$(bc -l <<< "1/$MSGS")
         LOG_FILE="$OUT_DIR/mqtt-latency-$MSGS-$TS.csv"
         echo "framework,test,metric,value,unit" > $LOG_FILE
         timeout $DURATION nice $NICE taskset -c $CPUS $MQTT_COMPARISON_DIR/$MQTT_PING -i $INTERVAL -b $CONNECT  -p 64 >> $LOG_FILE 2> /dev/null
         plog "[ DONE ] Running MQTT ping msg/s $MSGS, logged to $LOG_FILE"
         ;;
      2)
         plog "[ RUN ] Running MQTT pong"
         nice $NICE taskset -c $CPUS $MQTT_COMPARISON_DIR/$MQTT_PONG -b $CONNECT > /dev/null 2>&1
         plog "[ DONE ] Running MQTT pong"
         ;;
      3)
         plog "[ RUN ] Running mosquitto"
         docker compose -f $MOSQUITTO up > /dev/null 2>&1
         plog "[ DONE ] Running mosquitto"
         ;;
      *)
         usage
         ;;
      esac
      ;;
   z)
      # Zenoh
      case ${TORUN} in
      1)
         plog "[ RUN ] Running Zenoh ping with msg/s $MSGS"
         INTERVAL=$(bc -l <<< "1/$MSGS")
         LOG_FILE="$OUT_DIR/zenoh-latency-$MSGS-$TS.csv"
         echo "framework,test,metric,value,unit" > $LOG_FILE
         timeout $DURATION nice $NICE taskset -c $CPUS $BIN_DIR/$ZENOH_PING -i $INTERVAL -m peer --listen $LISTEN --connect $CONNECT >> $LOG_FILE 2> /dev/null
         plog "[ DONE ] Running Zenoh ping msg/s $MSGS, logged to $LOG_FILE"
         ;;
      2)
         plog "[ RUN ] Running Zenoh pong"
         nice $NICE taskset -c $CPUS $BIN_DIR/$ZENOH_PONG -m peer --listen $LISTEN --connect $CONNECT > /dev/null 2>&1
         plog "[ DONE ] Running Zenoh pong"
         ;;
      3)
         plog "[ RUN ] Running zenohd"
         nice $NICE taskset -c $CPUS $ZENOHD --listen $LISTEN --connect $CONNECT > /dev/null 2>&1
         plog "[ DONE ] Running zenohd"
         ;;
      *)
         usage
         ;;
      esac
      ;;
   r)
      # ROS2

      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      export ROS_LOCALHOST_ONLY=1
      export CYCLONEDDS_URI=$CYCLONEDDS_URI
      source /opt/ros/$ROSDISTRO/setup.bash
      source $ROS2_COMPARISON_DIR/install/setup.bash


      case ${TORUN} in
      1)
         plog "[ RUN ] Running ROS2 ping with msg/s $MSGS"
         LOG_FILE="$OUT_DIR/ros2-latency-$MSGS-$TS.csv"
         echo "framework,test,metric,value,unit" > $LOG_FILE
         timeout $DURATION nice $NICE taskset -c $CPUS $ROS2_PING $MSGS >> $LOG_FILE 2> /dev/null
         plog "[ DONE ] Running ROS2 ping with msg/s $MSGS"
         ps -ax | grep sender_ros | awk {'print $1'} | xargs kill -9 > /dev/null  2>&1
         ;;
      2)
         plog "[ RUN ] Running ROS2 pong "
         nice $NICE taskset -c $CPUS $ROS2_PONG  > /dev/null  2>&1
         plog "[ DONE ] Running ROS2 pong"
         ps -ax | grep receiver_ros | awk {'print $1'} | xargs kill -9 > /dev/null  2>&1
         ;;
      *)
         usage
         ;;
      esac
      unset RMW_IMPLEMENTATION
      unset ROS_LOCALHOST_ONLY
      unset CYCLONEDDS_URI
      ;;
   R)
      # ROS

      source /opt/ros/$ROSDISTRO/setup.bash
      export ROS_MASTER_URI=$ROS_MASTER_URI
      export ROS_IP="$ROS_IP"

      case ${TORUN} in
      1)
         plog "[ RUN ] Running ROS ping with msg/s $MSGS"
         LOG_FILE="$OUT_DIR/ros-latency-$MSGS-$TS.csv"
         echo "framework,test,metric,value,unit" > $LOG_FILE
         timeout $DURATION nice $NICE taskset -c $CPUS $ROS_BIN_DIR/$ROS_PING $MSGS >> $LOG_FILE 2> /dev/null
         plog "[ DONE ] Running ROS source with msg/s $MSGS"
         ps -ax | grep sender_node | awk {'print $1'} | xargs kill -9 > /dev/null 2>&1
         ;;
      2)
         plog "[ RUN ] Running ROS pong"
         nice $NICE taskset -c $CPUS $ROS_BIN_DIR/$ROS_PONG > /dev/null 2>&1
         plog "[ DONE ] Running ROS pong"
         ps -ax | grep receiver_node | awk {'print $1'} | xargs kill -9 > /dev/null 2>&1
         ;;
      3)
         plog "[ RUN ] Running ROS master"
         nice $NICE taskset -c $CPUS roscore -p 11311
         ;;
      *)
         usage
         ;;
      esac
      unset ROS_MASTER_URI
      ;;
   *)
      usage
      ;;
   esac
done
``
plog "Bye!"