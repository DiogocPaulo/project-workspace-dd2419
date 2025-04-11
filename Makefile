all: sync

sync:
	mkdir -p logs
	rosdep install -i --from-path src --rosdistro jazzy -y --as-root pip:false
	colcon build --symlink-install

network:
	fastdds discovery -i 0 -t 192.168.128.104 -q 42100

run-setup:
	ros2 launch localisation setup_launch.py | tee logs/setup.log

run-localisation:
	ros2 launch localisation localisation_launch.py | tee logs/localisation.log

run-joystick:
	ros2 launch navigation joystick_launch.py | tee logs/joystick.log
	
run-navigation:
	ros2 launch navigation navigation_launch.py | tee logs/navigation.log

run-collection:
	ros2 launch project_master master_launch.py | tee logs/collection.log

clean:
	rm -rf build/ install/ package/ logs/

	
