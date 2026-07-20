# Robotics Project Group 4
ROS2 workspace for robotics project (DD2419). Group 4 with robot **Sneezy**!

# System Overview

The runtime behavior is built around three connected capabilities:

- **Localization & state estimation:** odometry/IMU/wheel encoder fusion and frame handling.
- **Navigation & mapping:** workspace/object/obstacle map generation with path-following and joystick fallback.
- **Perception & manipulation:** camera-based detection and arm pickup pipeline for object collection tasks.

Core workspace packages (under `src/`):

| Package | Purpose |
| --- | --- |
| `project_interfaces` | Custom ROS 2 messages, services, and actions shared across packages. |
| `localization_cpp` | C++ localization pipeline with odometry and EKF-related processing. |
| `mapping` | Python map generation nodes for workspace, objects, and obstacles. |
| `navigation` | Navigation execution (including pure pursuit) and manual joystick control mode. |
| `detection` | Vision/perception pipeline (RGB-D based) for object detection. |
| `pick_up` | Arm control and pickup sequence integration. |

Additional repository directories:

| Directory | Purpose |
| --- | --- |
| `maps/` | Stored map files used for experiments and mission runs. |
| `runs/` | Runtime outputs / experiment artifacts. |
| `workspaces/` | Workspace-related resources used during development. |

# Demo

Below are demonstrations of the autonomous exploration and object collection phases.

| Autonomous Exploration | Object Collection |
| :---: | :---: |
| ![Exploration](docs/media/exploration_preview.gif) <br> *Sneezy mapping the environment* | ![Collection](docs/media/collection_preview.gif) <br> *Arm manipulation and object retrieval* |

# Localization and Navigation Steps
Have to launch phidgets before Localization and navigation. Thus for movement do the following:
1. Launch phidgets and frames:
```bash
make run-setup

```


2. Launch localization (includes odometry):
```bash
make run-localisation

```


3. Launch your preferred navigation method:
* For pure pursuit:
```bash
make run-navigation

```


* For joystick control:
```bash
make run-joystick

```


---

# Arm Control

To initiate the arm control sequence:

```bash
ros2 launch pick_up pick_up_launch.py

```

# Arm Camera

To launch the arm camera:

```bash
ros2 launch robp_launch arm_camera_launch.yaml

```

**RViz Instructions:**

* Click **Add**
* Select **By topic**
* Navigate to `Arm_camera/image_raw` -> `image`

---

# Detection

First, configure your frames depending on whether you are using odometry.

**Option A: With Odometry**

```bash
ros2 launch robp_launch frames_launch.xml
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom

```

**Option B: Without Odometry**

```bash
ros2 launch robp_launch frames_launch.xml
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link

```

Next, launch the camera and the detection node:

```bash
ros2 launch robp_launch rs_d435i_launch.py
ros2 launch detection detection_launch.py

```

**RViz Instructions:**

* Click **Add**
* Select **By topic**
* Navigate to `camera_depth/color/points_transformed` -> `PointCloud2`

---

# Lidar

To launch the Lidar sensor:

```bash
ros2 launch robp_launch lidar_launch.yaml

```

**RViz Instructions:**

* Click **Add**
* Select **By topic**
* Navigate to `laser_scan`
* *Important:* Make sure the fixed frame is set to `lidar_link`.



# Repository Structure

The `src/` directory is organized into functional ROS 2 packages:

* **`project_interfaces/`** - Contains custom ROS 2 messages (`.msg`), services (`.srv`), and actions (`.action`) used for communication across nodes (e.g., `Object`, `NavPath`, `PickUpObject`).
* **`localization_cpp/`** - Performance-critical C++ logic handling wheel encoders, IMU data, Extended Kalman Filtering (EKF), and coordinate transforms (`/odom` $\rightarrow$ `/base_link`).
* **`mapping/`** - Python nodes responsible for workspace, object, and obstacle mapping (`map_workspace`, `map_objects`, `map_obstacles`).
* **`navigation/`** - Implements navigation flows such as Pure Pursuit and manual joystick control.
* **`detection/`** - Vision and perception pipelines handling depth and color data from the Intel RealSense camera.
* **`pick_up/`** - Actuation and control workflows for the robotic arm and servo mechanisms.


# Contributors

This project was developed by **Group 4** for the KTH DD2419 Robotics course in VT2025.

*   Diogo Paulo
*   Elias Wetterwik
*   Gabriell Åhlin
*   Sebastian Thaeron
