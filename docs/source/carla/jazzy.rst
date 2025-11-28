==================================================================
CARLA Setup Guide - ROS 2 Jazzy (Docker)
==================================================================

.. list-table::
   :widths: 40 60
   :header-rows: 1
   :class: compact-table
   

   * - **Component**
     - **Version/Details**
   * - Course
     - ENPM818Z — On-Road Automated Vehicles
   * - ROS Distribution
     - Jazzy Jalisco
   * - Ubuntu Version
     - 24.04 (Noble Numbat)
   * - CARLA Version
     - 0.9.16
   * - Installation Method
     - Docker 🐳

---------------------------------------------------------
Overview
---------------------------------------------------------

This guide walks you through setting up CARLA 0.9.16 with ROS 2 Jazzy using Docker. You will use a **custom ROS 2 bridge package** that bypasses CARLA's native ROS 2 implementation due to a known bug (see :ref:`known-issue`).

**What You will Install:**

- Docker runtime with NVIDIA GPU support
- CARLA 0.9.16 Docker image
- CARLA Python client (via apt)
- Custom ROS 2 bridge package

.. note::
   **Why Docker?** Due to the lack of native support for ROS 2 Jazzy in CARLA, we will deploy the simulator within a Docker container. This containerized instance will interface directly with the ROS 2 Jazzy environment running on the host system.

---------------------------------------------------------
Prerequisites
---------------------------------------------------------

System Requirements
~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 30 70
   :header-rows: 1
   :class: compact-table

   * - **Component**
     - **Requirement**
   * - Operating System
     - Ubuntu 24.04 (Noble Numbat)
   * - ROS 2 Distribution
     - Jazzy Jalisco (already installed)
   * - GPU
     - NVIDIA GPU (recommended for performance)
   * - RAM
     - Minimum 8 GB, Recommended 16 GB
   * - Disk Space
     - ~15 GB for Docker image and dependencies
   * - Python
     - Python 3.12 (comes with Ubuntu 24.04)


---------------------------------------------------------
Step 1: Pull CARLA Docker Image
---------------------------------------------------------

Download the CARLA 0.9.16 Docker image:

.. code-block:: bash

   # Pull CARLA image (this may take 10-15 minutes)
   docker pull carlasim/carla:0.9.16

   # Verify image is downloaded
   docker images | grep carla

Expected output:

.. code-block:: text

   carlasim/carla:0.9.16             98d224668ad0       20.7GB             0B 

---------------------------------------------------------
Step 2: Install Additional Dependencies
---------------------------------------------------------

.. code-block:: bash

   # Install required Python packages
   pip3 install numpy pygame
   
   # Install CARLA Python client
   pip3 install carla==0.9.16
   
   # Verify installations
   python3 -c "import numpy; import pygame; import carla; print('All dependencies OK')"

---------------------------------------------------------
Step 3: Clone and Build ROS 2 Bridge Package
---------------------------------------------------------

Clone the Repository
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Create workspace
   mkdir -p ~/carla_ws/src
   cd ~/carla_ws/src

   # Clone the Jazzy branch
   git clone -b jazzy https://github.com/zeidk/enpm818z-fall-2025-carla.git carla_ros2_bridge

   # Return to workspace root
   cd ~/carla_ws

Build the Package
~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Source ROS 2
   source /opt/ros/jazzy/setup.bash

   # Build
   colcon build --symlink-install

   # Source the workspace
   source install/setup.bash

Verify the build:

.. code-block:: bash

   # Check if package is available
   ros2 pkg list | grep carla_ros2_bridge

   # Check executables
   ros2 pkg executables carla_ros2_bridge

Expected output:

.. code-block:: text

   carla_ros2_bridge carla_bridge
   carla_ros2_bridge carla_camera_publisher
   carla_ros2_bridge carla_camera_publisher_with_display
   carla_ros2_bridge carla_image_subscriber

---------------------------------------------------------
Step 4: Setup Environment Configuration
---------------------------------------------------------

Create a setup function that configures your environment for CARLA and ROS 2.

Create Setup Script
~~~~~~~~~~~~~~~~~~~


Add the following function to your ``~/.bashrc``:


.. code-block:: bash

   # Add this function at the end:
   carla_setup() {
       # Configuration
       CARLA_WS="/home/$USER/carla_ws"
       
       echo "🔧 Setting up CARLA ROS 2 environment..."
       
       # Setup environment variables
       export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
       export ROS_DOMAIN_ID=0
       unset ROS_LOCALHOST_ONLY
       unset FASTRTPS_DEFAULT_PROFILES_FILE
       
       # Source ROS 2 Jazzy
       if [ -f "/opt/ros/jazzy/setup.bash" ]; then
           source /opt/ros/jazzy/setup.bash
       else
           echo "❌ Error: ROS 2 Jazzy not found"
           return 1
       fi
       
       # Source CARLA workspace
       if [ -d "$CARLA_WS" ]; then
           if [ -f "$CARLA_WS/install/setup.bash" ]; then
               source "$CARLA_WS/install/setup.bash"
               echo "✅ Sourced CARLA workspace"
           else
               echo "⚠️  Warning: CARLA workspace not built"
               echo "   Run: cd $CARLA_WS && colcon build"
           fi
       fi
       
       # Reset ROS 2 daemon
       echo "🔄 Restarting ROS 2 daemon..."
       ros2 daemon stop > /dev/null 2>&1
       sleep 1
       ros2 daemon start > /dev/null 2>&1
       
       # Create CARLA Docker alias
       alias carla='xhost +local:root && docker run \
           --rm \
           --privileged \
           --runtime=nvidia \
           --gpus all \
           --net=host \
           --ipc=host \
           -v /dev/shm:/dev/shm \
           -e DISPLAY=$DISPLAY \
           -e NVIDIA_VISIBLE_DEVICES=all \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
           carlasim/carla:0.9.16 \
           /bin/bash -c "./CarlaUE4.sh -nosound -quality-level=Low -vulkan"'
       
       echo "✅ CARLA Setup Complete!"
       echo ""
       echo "📋 Usage:"
       echo "   1. Start CARLA:  carla"
       echo "   2. Run ROS 2 bridge:"
       echo "      ros2 run carla_ros2_bridge carla_camera_publisher"
       echo ""
       echo "🔍 Environment:"
       echo "   ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
       echo "   RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
       
       cd "$CARLA_WS"
   }
   
   # Auto-run setup when starting new terminal
   carla_setup

Save and reload:

.. code-block:: bash

   # Save (Ctrl+O, Enter, Ctrl+X)
   
   # Reload bashrc
   source ~/.bashrc

.. _known-issue:

---------------------------------------------------------
Understanding the CARLA 0.9.16 ROS 2 Bug
---------------------------------------------------------

Issue Description
~~~~~~~~~~~~~~~~~

CARLA 0.9.16 introduced **native ROS 2 support** via the ``--ros2`` flag and ``enable_for_ros()`` API. However, there is a **critical bug** in the topic name generation:

**The Bug:**

CARLA creates topic names with double slashes: ``/carla//camera/image``

**Why This Matters:**

ROS 2 Jazzy strictly validates topic names and **rejects topics with consecutive slashes** as invalid. This means:

- ❌ ``ros2 topic echo`` doesn't work
- ❌ ``ros2 topic hz`` doesn't work  
- ❌ ``rviz2`` can't subscribe to topics
- ❌ Custom nodes fail to receive data

**Example:**

.. code-block:: bash

   # CARLA publishes (broken):
   /carla//front_camera/image  ← Double slash!
   
   # ROS 2 Jazzy says:
   Invalid topic name: topic name must not contain repeated '/'

Our Solution
~~~~~~~~~~~~

The **custom ROS 2 bridge package** you installed:

1. **Bypasses CARLA's native ROS 2** - Doesn't use ``enable_for_ros()``
2. **Uses Python API directly** - Gets data via ``camera.listen()`` callbacks
3. **Publishes to clean topics** - ``/carla/camera/image`` (no double slash)
4. **Works with all ROS 2 tools** - Full compatibility

**This is why we don't use the** ``--ros2`` **flag with CARLA!**

GitHub Issue Reference
~~~~~~~~~~~~~~~~~~~~~~

This is a known issue tracked here: https://github.com/carla-simulator/carla/issues/9278

Expected to be fixed in future CARLA releases, but for now, our bridge is the solution.

---------------------------------------------------------
Running CARLA with ROS 2
---------------------------------------------------------

Basic Workflow
~~~~~~~~~~~~~~

**Terminal 1: Start CARLA Server**

.. code-block:: bash

   carla # this is the alias from carla_setup()

Wait for the CARLA window to open and the world to load (~30 seconds).

**Terminal 2: Run ROS 2 Bridge**

.. code-block:: bash

   # Source environment (automatic if you added carla_setup to bashrc)
   source ~/.bashrc
   
   # Run the camera publisher node
   ros2 run carla_ros2_bridge carla_camera_publisher

You should see:

.. code-block:: text

   ============================================================
   CARLA Camera Publisher Node
   ============================================================
   Connecting to CARLA at localhost:2000...
   ✓ Connected to CARLA 0.9.16
   ✓ Spawned vehicle at Location(x=..., y=..., z=...)
   ✓ Autopilot enabled
   ✓ Camera spawned and attached
   ✓ Camera listening
   ============================================================
   Publishing to:
     /carla/camera/image
     /carla/camera/camera_info
     /carla/vehicle/odometry
   ============================================================

**Terminal 3: Verify Topics**

.. code-block:: bash

   # List topics
   ros2 topic list | grep carla

   # Check image topic rate
   ros2 topic hz /carla/camera/image

   # Echo camera info
   ros2 topic echo /carla/camera/camera_info --once

**Terminal 4: Run Test Subscriber**

.. code-block:: bash

   ros2 run carla_ros2_bridge carla_image_subscriber

You should see:

.. code-block:: text

   [INFO] [carla_image_subscriber]: ✓ Received first image!
   [INFO] [carla_image_subscriber]:   Size: 800x600
   [INFO] [carla_image_subscriber]:   Encoding: rgb8
   [INFO] [carla_image_subscriber]: Frame 30: ~20.0 Hz

---------------------------------------------------------
Package Overview
---------------------------------------------------------

The ``carla_ros2_bridge`` package contains the following nodes:

carla_camera_publisher
~~~~~~~~~~~~~~~~~~~~~~

**Purpose:** Main bridge node that connects to CARLA, spawns a vehicle with camera, and publishes sensor data.

**Subscriptions:** None

**Publications:**

.. list-table::
   :widths: 35 25 40
   :header-rows: 1
   :class: compact-table

   * - **Topic**
     - **Type**
     - **Description**
   * - ``/carla/camera/image``
     - sensor_msgs/Image
     - RGB camera images at ~20 Hz
   * - ``/carla/camera/camera_info``
     - sensor_msgs/CameraInfo
     - Camera calibration parameters
   * - ``/carla/vehicle/odometry``
     - nav_msgs/Odometry
     - Vehicle pose and velocity at 20 Hz

**Parameters:**

.. list-table::
   :widths: 25 15 15 45
   :header-rows: 1
   :class: compact-table

   * - **Parameter**
     - **Type**
     - **Default**
     - **Description**
   * - ``carla_host``
     - string
     - localhost
     - CARLA server hostname
   * - ``carla_port``
     - int
     - 2000
     - CARLA server port
   * - ``image_width``
     - int
     - 800
     - Camera image width (pixels)
   * - ``image_height``
     - int
     - 600
     - Camera image height (pixels)
   * - ``camera_fov``
     - float
     - 110.0
     - Camera field of view (degrees)
   * - ``camera_x``
     - float
     - 1.6
     - Camera X position relative to vehicle (forward)
   * - ``camera_z``
     - float
     - 1.2
     - Camera Z position relative to vehicle (up)
   * - ``spawn_vehicle``
     - bool
     - true
     - Automatically spawn vehicle
   * - ``autopilot``
     - bool
     - true
     - Enable autopilot

**Example with custom parameters:**

.. code-block:: bash

   ros2 run carla_ros2_bridge carla_camera_publisher \
       --ros-args \
       -p image_width:=1280 \
       -p image_height:=720 \
       -p camera_fov:=90.0 \
       -p autopilot:=false

carla_image_subscriber
~~~~~~~~~~~~~~~~~~~~~~

**Purpose:** Example subscriber node for testing. Displays image statistics.

**Subscriptions:**

.. list-table::
   :widths: 35 25 40
   :header-rows: 1
   :class: compact-table

   * - **Topic**
     - **Type**
     - **Description**
   * - ``/carla/camera/image``
     - sensor_msgs/Image
     - Camera images

**Publications:** None

**Parameters:**

.. list-table::
   :widths: 25 15 15 45
   :header-rows: 1
   :class: compact-table

   * - **Parameter**
     - **Type**
     - **Default**
     - **Description**
   * - ``topic``
     - string
     - /carla/camera/image
     - Topic to subscribe to

---------------------------------------------------------
Advanced Usage
---------------------------------------------------------

Visualizing in RViz2
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Start RViz2
   rviz2

   # In RViz2:
   # 1. Click "Add" → "By topic"
   # 2. Select "/carla/camera/image" → "Image"
   # 3. Set Fixed Frame to "camera_link"

Recording Data
~~~~~~~~~~~~~~

.. code-block:: bash

   # Record camera and odometry
   ros2 bag record /carla/camera/image /carla/vehicle/odometry

   # Record all CARLA topics
   ros2 bag record -r "/carla/.*"

Custom Resolution
~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # High resolution
   ros2 run carla_ros2_bridge carla_camera_publisher \
       --ros-args \
       -p image_width:=1920 \
       -p image_height:=1080

   # Lower resolution for better performance
   ros2 run carla_ros2_bridge carla_camera_publisher \
       --ros-args \
       -p image_width:=640 \
       -p image_height:=480

---------------------------------------------------------
Troubleshooting
---------------------------------------------------------

CARLA Won't Start
~~~~~~~~~~~~~~~~~

**Symptom:** ``docker: Error response from daemon...``

**Solutions:**

1. Verify Docker is running:

   .. code-block:: bash

      sudo systemctl status docker
      sudo systemctl start docker

2. Check GPU access:

   .. code-block:: bash

      docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi

3. Allow X11 forwarding:

   .. code-block:: bash

      xhost +local:root

Cannot Connect to CARLA
~~~~~~~~~~~~~~~~~~~~~~~~

**Symptom:** ``Failed to connect to CARLA: timeout``

**Solutions:**

1. Wait for CARLA to fully load (30-60 seconds after window appears)

2. Check CARLA is running:

   .. code-block:: bash

      docker ps
      netstat -tuln | grep 2000

3. Verify port is accessible:

   .. code-block:: bash

      python3 -c "import carla; c = carla.Client('localhost', 2000); c.set_timeout(2.0); print(c.get_server_version())"

No Topics Visible
~~~~~~~~~~~~~~~~~

**Symptom:** ``ros2 topic list`` doesn't show ``/carla/camera/image``

**Solutions:**

1. Verify bridge node is running:

   .. code-block:: bash

      ros2 node list

2. Check for errors in bridge output

3. Restart ROS 2 daemon:

   .. code-block:: bash

      ros2 daemon stop
      ros2 daemon start

4. Re-source workspace:

   .. code-block:: bash

      source ~/.bashrc

Build Errors
~~~~~~~~~~~~

**Symptom:** ``colcon build`` fails

**Solutions:**

1. Clean workspace:

   .. code-block:: bash

      cd ~/carla_ws
      rm -rf build install log
      colcon build --symlink-install

2. Verify dependencies:

   .. code-block:: bash

      rosdep install --from-paths src --ignore-src -r -y

3. Check ROS 2 is sourced:

   .. code-block:: bash

      echo $ROS_DISTRO  # Should show "jazzy"

---------------------------------------------------------
Performance Optimization
---------------------------------------------------------

For Better Performance
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Lower graphics quality
   alias carla='... ./CarlaUE4.sh -quality-level=Low -nosound -vulkan'

   # Reduce camera resolution
   ros2 run carla_ros2_bridge carla_camera_publisher \
       --ros-args -p image_width:=640 -p image_height:=480

For Better Quality
~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Higher graphics
   alias carla='... ./CarlaUE4.sh -quality-level=Epic -vulkan'

   # Higher resolution
   ros2 run carla_ros2_bridge carla_camera_publisher \
       --ros-args -p image_width:=1920 -p image_height:=1080

---------------------------------------------------------
References
---------------------------------------------------------

- CARLA Documentation: https://carla.readthedocs.io/en/0.9.16/
- ROS 2 Jazzy: https://docs.ros.org/en/jazzy/
- Docker Documentation: https://docs.docker.com/
- NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/
- GitHub Issue (Double Slash Bug): https://github.com/carla-simulator/carla/issues/9278