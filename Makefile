
sync:
	rosdep install -i --from-path src --rosdistro jazzy -y --as-root pip:false
	colcon build --symlink-install
