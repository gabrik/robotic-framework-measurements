.PHONY: clean zk ros ros2 clean_ros clean_zf mqtt

all: zk ros ros2 mqtt

mqtt:
	cd mqtt && make

zk:
	RUSTFLAGS='-C target-cpu=native'  cargo build --release --all-targets

ros:
	bash -c "source /opt/ros/noetic/setup.bash && cd ros/eval-ws && catkin_make"

ros2:
	bash -c "source /opt/ros/galactic/setup.bash && cd ros2/eval-ws && colcon build"


clean_ros:
	rm -rf ros/eval-ws/build/
	rm -rf ros/eval-ws/devel/

clean_ros2:
	rm -rf ros2/eval-ws/build/
	rm -rf ros2/eval-ws/install/
	rm -rf ros2/eval-ws/log/

clean_mqtt:
	rm -rf mqtt/mosquitto
	rm -rf mqtt/mqtt-lib
	rm -rf mqtt/target

clean_zk:
	cargo clean

clean: clean_ros clean_ros2 clean_zk clean_mqtt

