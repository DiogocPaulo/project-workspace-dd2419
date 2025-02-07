
all: sync

sync:
	rosdep install -i --from-path src --rosdistro jazzy -y --as-root pip:false
	colcon build --symlink-install

run-movement:
	ros2 launch movement movement_launch.py

run-localisation:
	ros2 launch localisation localisation_launch.py | tee test_logs/localisation.log

clean:
	rm -rf build/ install/ package/

	
