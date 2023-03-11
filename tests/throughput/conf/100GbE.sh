CONF=$(dirname "${BASH_SOURCE[0]}")
source $CONF/testbed.sh


#ROS
ROS_MASTER=$C17
ROS_MASTER_NICE="-10"
ROS_MASTER_CPUS="0,1"
ROS_MASTER_URI="http://$C17_C16:11311"
ROS_IP_MASTER=$C17_C16

ROS_SRC=$C16
ROS_SRC_NICE="-10"
ROS_SRC_CPUS="0,1"
ROS_IP_SRC=$C16_C17

ROS_OP=$C17
ROS_OP_NICE="-10"
ROS_OP_CPUS="2,3"
ROS_IP_OP=$C17_C16

ROS_SNK=$C16
ROS_SNK_NICE="-10"
ROS_SNK_CPUS="2,3"
ROS_IP_SNK=$C16_C17
#ROS2

ROS2_SRC=$C16
ROS2_SRC_NICE="-10"
ROS2_SRC_CPUS="0,1"
ROS2_SRC_CYCLONEDDS_URI=$C16_CYCLONEDDS_URI


ROS2_OP=$C17
ROS2_OP_NICE="-10"
ROS2_OP_CPUS="0,1"
ROS2_OP_CYCLONEDDS_URI=$C17_CYCLONEDDS_URI

ROS2_SNK=$C16
ROS2_SNK_NICE="-10"
ROS2_SNK_CPUS="2,3"
ROS2_SNK_CYCLONEDDS_URI=$C16_CYCLONEDDS_URI

# ZF
ZF_SRC=$C16
ZF_SRC_NICE="-10"
ZF_SRC_CPUS="0,1,2"

ZF_OP=$C17
ZF_OP_NICE="-10"
ZF_OP_CPUS="0,1,2"

ZF_SNK=$C16
ZF_SNK_NICE="-10"
ZF_SNK_CPUS="3,4,5"

LOCATOR_SRC="tcp/$C16_C17:7447"
LOCATOR_OP="tcp/$C17_C16:7447"
LOCATOR_SNK="tcp/$C16_C17:7887"

LOCATOR_SRC_OP="tcp/$C17_C16:7447"
LOCATOR_OP_SNK="tcp/$C16_C17:7887"