
all: sync

sync:
	rosdep install -i --from-path src --rosdistro jazzy -y --as-root pip:false
	colcon build --symlink-install

network:
	fastdds discovery -i 0 -t 192.168.128.104 -q 42100

run-movement:
	ros2 launch movement movement_launch.py

clean:
	rm -rf build/ install/ package/

	
