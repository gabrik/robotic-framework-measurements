#!/usr/bin/env bash


plog () {
   LOG_TS=`eval date "+%F-%T"`
   echo "[$LOG_TS]: $1"
}

usage() { printf "Usage: $0 \n\t-k Kafka\n\t-m MQTT\n\t-z zenoh\n\t-r ROS2\n\t-R ROS\n" 1>&2; exit 1; }



if [[ ! $@ =~ ^\-.+ ]]
then
  usage;
fi



TS=$(date +%Y%m%d.%H%M%S)

N_CPU=$(nproc)

INITIAL_MSGS=1

BIN_DIR="./target/release"
ROS_BIN_DIR="./ros/eval-ws/build"

WD=$(pwd)

ZENOH_PONG="zenoh_pong"
ZENOH_PING="zenoh_ping"
KAFKA_PONG="kafka_pong"
KAFKA_PING="kafka_ping"

ROS2_COMPARISON_DIR="./ros2/eval-ws"
MQTT_COMPARISON_DIR="./mqtt"

ROS2_PING="ros2 run sender sender_ros"
ROS2_PONG="ros2 run receiver receiver_ros"

ROS_PING="sender/sender_node"
ROS_PONG="receiver/receiver_node"

OUT_DIR="${OUT_DIR:-logs}"

FINAL_MSGS=${FINAL_MSGS:-1000000}
DURATION=${DURATION:-60}

CYCLONEDDS_URI="$WD/ros2/cyclonedds.xml"
ROSDISTRO="foxy"
INITIAL_SIZE=1
FINAL_SIZE=${FINAL_SIZE:-134217728} # 128MB

mkdir -p $OUT_DIR


plog "[ INIT ] Duration will be $DURATION seconds for each test"
plog "[ INIT ] Max sending rate will be $FINAL_MSGS msg/s for each test"
plog "[ INIT ] Max size for throughput tests will be $FINAL_SIZE"
while getopts "kmzrR" arg; do
   case ${arg} in
   h)
      usage
      ;;
   k)
      # Kafka
      ;;
   m)
      # MQTT

      ;;
   z)
      # Zenoh ping

      plog "[ START ] baseline Zenoh Latency test"
      LOG_FILE="$OUT_DIR/zenoh-pp-$TS.csv"
      echo "framework,test,metric,value,unit" > $LOG_FILE
      s=$INITIAL_MSGS
      while [ $s -le $FINAL_MSGS ]
      do
         plog "[ RUN ] baseline Zenoh for msg/s $s"
         INTERVAL=$(bc -l <<< "1/$s")

         nohup taskset -c 1 $BIN_DIR/$ZENOH_PONG > /dev/null 2>&1
         sleep 3

         timeout $DURATION  taskset -c 0 $BIN_DIR/$ZENOH_PING -i $INTERVAL >> $LOG_FILE 2> /dev/null &

         ps -ax | grep $ZENOH_PING | awk {'print $1'} | xargs kill -9 > /dev/null  2>&1

         plog "[ DONE ] baseline Zenoh for msg/s $s"
         sleep 1
         echo "Still running $ZENOH_PING: $(ps -ax | grep $ZENOH_PING | wc -l) - This should be 0"

         s=$(($s * 10))
      done
      plog "[ END ] baseline Zenoh Latency test"
      ;;
   r)
      # ROS2

      plog "[ START ] ROS2 Latency test"
      LOG_FILE="$OUT_DIR/ros2-lat-$TS.csv"
      echo "framework,test,metric,value,unit" > $LOG_FILE
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      export ROS_LOCALHOST_ONLY=1
      source /opt/ros/$ROSDISTRO/setup.bash
      source $ROS2_COMPARISON_DIR/install/setup.bash

      s=$INITIAL_MSGS
      while [ $s -le $FINAL_MSGS ]
      do
         plog "[ RUN ] ROS2 for msg/s $s"

         nohup taskset -c 0,1 $ROS2_PONG > /dev/null 2>&1 &

         sleep 3

         timeout $DURATION taskset -c 2,3 $ROS_PING $s  >> $LOG_FILE 2> /dev/null &

         ps -ax | grep receiver_ros | awk {'print $1'} | xargs kill -9 > /dev/null  2>&1
         ps -ax | grep sender_ros | awk {'print $1'} | xargs kill -9 > /dev/null  2>&1


         plog "[ DONE ] ROS2 for msg/s $s w/ Ping"
         sleep 1
         echo "Still running receiver_ros: $(ps -A | grep receiver_ros | wc -l) - This should be 0"
         echo "Still running sender_ros: $(ps -A | grep sender_ros | wc -l) - This should be 0"
         s=$(($s * 10))

      done
      unset RMW_IMPLEMENTATION
      unset ROS_LOCALHOST_ONLY
      plog "[ END ] ROS2 Latency test"
      ;;
   R)
      # ROS

      plog "[ START ] ROS Latency test"
      LOG_FILE="$OUT_DIR/ros-lat-$TS.csv"
      echo "framework,scenario,test,pipeline,payload,rate,value,unit" > $LOG_FILE

      source /opt/ros/noetic/setup.bash
      export ROS_MASTER_URI=http://127.0.0.1:11311
      nohup roscore -p 11311 > /dev/null 2>&1 &
      sleep 3


      s=$INITIAL_MSGS
      while [ $s -le $FINAL_MSGS ]
      do
         plog "[ RUN ] ROS for msg/s $s"


         nohup taskset -c 0,1 $ROS_BIN_DIR/$ROS_SINK $s 1 "out_1" >> $LOG_FILE 2> /dev/null &
         sleep 3

         nohup taskset -c 2,3 $ROS_BIN_DIR/$ROS_OP "out_0" "out_1" > /dev/null 2>&1 &
         sleep 3

         timeout $DURATION taskset -c 4,5 $ROS_BIN_DIR/$ROS_SRC $s #> /dev/null 2>&1

         ps -ax | grep compute_node | awk {'print $1'} | xargs kill -9 > /dev/null 2>&1
         ps -ax | grep receiver_node | awk {'print $1'} | xargs kill -9 > /dev/null 2>&1
         ps -ax | grep sender_node | awk {'print $1'} | xargs kill -9 > /dev/null 2>&1



         plog "[ DONE ] ROS for msg/s $s w/ Ping"
         sleep 1
         echo "Still running compute_node: $(ps -A | grep compute_node | wc -l) - This should be 0"
         echo "Still running receiver_node: $(ps -A | grep receiver_node | wc -l) - This should be 0"
         echo "Still running sender_node: $(ps -A | grep sender_node | wc -l) - This should be 0"
         s=$(($s * 10))

      done

      ps -ax | grep roscore | awk {'print $1'} | xargs kill -2 > /dev/null 2>&1
      ps -ax | grep rosmaster | awk {'print $1'} | xargs kill -9 > /dev/null 2>&1
      echo "Still running roscore: $(ps -A | grep compute_node | wc -l) - This should be 0"
      echo "Still running rosmaster: $(ps -A | grep compute_node | wc -l) - This should be 0"

      plog "[ END ] ROS Latency test"
      unset ROS_MASTER_URI
      ;;
   *)
      usage
      ;;
   esac
done
``
plog "Bye!"

