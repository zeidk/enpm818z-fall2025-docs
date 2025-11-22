=====================================
CARLA Simulator
=====================================

CARLA (CAR Learning to Act) is an open-source autonomous driving simulator designed to support development, training, and validation of autonomous driving systems.

.. important::
   **Why CARLA 0.9.16?**
   
   We specifically use CARLA 0.9.16 for this course because:
   
   - **No need for carla-ros-bridge**: The official carla-ros-bridge package is heavy, complex, and adds unnecessary overhead for educational purposes
   - **Lightweight custom bridge**: We use a streamlined custom ROS 2 bridge that provides exactly what we need for the course
   - **Simplified setup**: Avoiding the official bridge reduces installation complexity and system requirements
   - **Better performance**: Our custom bridge has minimal dependencies and runs more efficiently
   - **Educational focus**: Students can understand the bridge code rather than dealing with a black-box package
   
   While newer CARLA versions exist, 0.9.16 offers the perfect balance of features, stability, and simplicity for learning autonomous vehicle concepts.

.. card:: 🚗 CARLA Overview
   :class-header: bg-primary text-white
   :class-card: sd-mt-3

   CARLA provides:
   
   - **Realistic urban environments** with detailed cityscapes
   - **Physics simulation** powered by Unreal Engine 4
   - **Sensor suite** including cameras, LiDAR, radar, GNSS, and IMU
   - **Python API** for complete control and automation
   - **Weather control** and day-night cycles
   - **Blueprint library** with vehicles, pedestrians, and props
   - **Traffic simulation** with AI-controlled agents
   - **ROS integration** for robotics applications

.. toctree::
   :maxdepth: 1
   :caption: Setup Guides

   humble
   jazzy

Course Configuration
--------------------

.. tab-set::

   .. tab-item:: Native Setup (Recommended)
      :sync: native

      .. card:: 💻 Native Installation
         :class-header: bg-success text-white

         .. list-table::
            :widths: 40 60
            :class: table-hover

            * - **Course**
              - ENPM818Z — On-Road Automated Vehicles
            * - **ROS Distribution**
              - :bdg-primary:`Humble Hawksbill`
            * - **Ubuntu Version**
              - :bdg-success:`22.04` (Jammy Jellyfish)
            * - **CARLA Version**
              - :bdg-warning:`0.9.16`
            * - **Installation Method**
              - :bdg-info:`Native` (No Docker)
            * - **Python Version**
              - 3.10
            * - **Complexity**
              - ⭐⭐ Simple

   .. tab-item:: Docker Setup
      :sync: docker

      .. card:: 🐳 Docker Installation
         :class-header: bg-primary text-white

         .. list-table::
            :widths: 40 60
            :class: table-hover

            * - **Course**
              - ENPM818Z — On-Road Automated Vehicles
            * - **ROS Distribution**
              - :bdg-info:`Jazzy Jalisco` :bdg-light:`Latest`
            * - **Ubuntu Version**
              - :bdg-success:`24.04` (Noble Numbat) :bdg-light:`LTS`
            * - **CARLA Version**
              - :bdg-warning:`0.9.16`
            * - **Installation Method**
              - :bdg-primary:`Docker` 🐳
            * - **Python Version**
              - 3.12
            * - **Complexity**
              - ⭐⭐⭐ Moderate

CARLA Command-Line Options
--------------------------

.. card:: 🎮 Launch Options Reference
   :class-header: bg-dark text-white

   Complete reference for ``./CarlaUE4.sh`` command-line options

.. tab-set::

   .. tab-item:: Graphics Settings
      :sync: graphics

      .. list-table::
         :widths: 35 65
         :header-rows: 1
         :class: table-striped

         * - **Option**
           - **Description**
         * - ``-quality-level=Low``
           - Lowest graphics quality (best performance)
         * - ``-quality-level=Medium``
           - Balanced quality/performance
         * - ``-quality-level=High``
           - High quality graphics
         * - ``-quality-level=Epic``
           - Maximum quality (requires powerful GPU)
         * - ``-vulkan``
           - Use Vulkan renderer (recommended for NVIDIA)
         * - ``-opengl``
           - Use OpenGL renderer (compatibility fallback)
         * - ``-dx11``
           - Use DirectX 11 (Windows only)
         * - ``-dx12``
           - Use DirectX 12 (Windows only)

      **Examples:**
      
      .. code-block:: bash

         # Best performance
         ./CarlaUE4.sh -quality-level=Low -vulkan

         # Best quality
         ./CarlaUE4.sh -quality-level=Epic -vulkan

         # Compatibility mode
         ./CarlaUE4.sh -quality-level=Low -opengl

   .. tab-item:: Window Settings
      :sync: window

      .. list-table::
         :widths: 35 65
         :header-rows: 1
         :class: table-striped

         * - **Option**
           - **Description**
         * - ``-windowed``
           - Run in windowed mode (not fullscreen)
         * - ``-ResX=1920``
           - Set window width (pixels)
         * - ``-ResY=1080``
           - Set window height (pixels)
         * - ``-fullscreen``
           - Force fullscreen mode
         * - ``-vr``
           - Enable VR mode
         * - ``-RenderOffScreen``
           - Headless mode (no display window)

      **Examples:**
      
      .. code-block:: bash

         # Windowed at 1280x720
         ./CarlaUE4.sh -windowed -ResX=1280 -ResY=720

         # Headless server mode
         ./CarlaUE4.sh -RenderOffScreen

         # Fullscreen at native resolution
         ./CarlaUE4.sh -fullscreen

   .. tab-item:: Performance Options
      :sync: performance

      .. list-table::
         :widths: 35 65
         :header-rows: 1
         :class: table-striped

         * - **Option**
           - **Description**
         * - ``-nosound``
           - Disable all audio (saves CPU)
         * - ``-fps=30``
           - Set target framerate
         * - ``-benchmark``
           - Run benchmark mode
         * - ``-deterministic``
           - Fixed timestep for reproducibility
         * - ``-fixed-delta-seconds=0.05``
           - Set fixed timestep (seconds)
         * - ``-no-rendering``
           - Disable rendering (simulation only)

      **Examples:**
      
      .. code-block:: bash

         # Optimized for simulation
         ./CarlaUE4.sh -nosound -fps=20 -quality-level=Low

         # Deterministic mode for testing
         ./CarlaUE4.sh -deterministic -fixed-delta-seconds=0.05

         # Maximum performance
         ./CarlaUE4.sh -no-rendering -nosound

   .. tab-item:: Network Settings
      :sync: network

      .. list-table::
         :widths: 35 65
         :header-rows: 1
         :class: table-striped

         * - **Option**
           - **Description**
         * - ``-carla-rpc-port=2000``
           - Set RPC port (default: 2000)
         * - ``-carla-streaming-port=2001``
           - Set streaming port (default: 2001)
         * - ``-carla-secondary-port=2002``
           - Set secondary port (default: 2002)
         * - ``-carla-host=0.0.0.0``
           - Set host IP to bind
         * - ``-world-port=8000``
           - Set world port
         * - ``-ros2``
           - Enable ROS 2 mode (buggy in 0.9.16)

      **Examples:**
      
      .. code-block:: bash

         # Custom ports
         ./CarlaUE4.sh -carla-rpc-port=3000 -carla-streaming-port=3001

         # Bind to specific IP
         ./CarlaUE4.sh -carla-host=192.168.1.100
         
         # Multiple CARLA instances
         # Instance 1
         ./CarlaUE4.sh -carla-rpc-port=2000
         # Instance 2
         ./CarlaUE4.sh -carla-rpc-port=3000

   .. tab-item:: Map Selection
      :sync: maps

      .. list-table::
         :widths: 35 65
         :header-rows: 1
         :class: table-striped

         * - **Option**
           - **Description**
         * - ``-map=Town01``
           - Load Town01 (small town)
         * - ``-map=Town02``
           - Load Town02 (small town)
         * - ``-map=Town03``
           - Load Town03 (large city)
         * - ``-map=Town04``
           - Load Town04 (mountain town)
         * - ``-map=Town05``
           - Load Town05 (urban + highway)
         * - ``-map=Town06``
           - Load Town06 (highway)
         * - ``-map=Town07``
           - Load Town07 (rural)
         * - ``-map=Town10HD``
           - Load Town10 HD (downtown)

      **Examples:**
      
      .. code-block:: bash

         # Load specific map
         ./CarlaUE4.sh -map=Town05

         # Highway testing
         ./CarlaUE4.sh -map=Town06 -quality-level=Low

Common Launch Configurations
-----------------------------

.. grid:: 1 1 2 2
   :gutter: 3

   .. grid-item-card:: 🏃 Development
      :class-header: bg-success text-white

      **Fast iteration, good visibility**
      
      .. code-block:: bash

         ./CarlaUE4.sh \
           -windowed \
           -ResX=1280 -ResY=720 \
           -quality-level=Low \
           -nosound \
           -vulkan

   .. grid-item-card:: 🧪 Testing
      :class-header: bg-warning

      **Reproducible results**
      
      .. code-block:: bash

         ./CarlaUE4.sh \
           -deterministic \
           -fixed-delta-seconds=0.05 \
           -fps=20 \
           -nosound \
           -quality-level=Low

   .. grid-item-card:: 📊 Data Collection
      :class-header: bg-info text-white

      **High quality sensors**
      
      .. code-block:: bash

         ./CarlaUE4.sh \
           -quality-level=Epic \
           -fps=30 \
           -vulkan \
           -windowed

   .. grid-item-card:: 🖥️ Headless Server
      :class-header: bg-dark text-white

      **No GPU required**
      
      .. code-block:: bash

         ./CarlaUE4.sh \
           -RenderOffScreen \
           -nosound \
           -carla-rpc-port=2000

CARLA Python API
----------------

.. card:: 🐍 Python API Overview
   :class-header: bg-gradient-primary text-white

   The Python API provides complete control over the CARLA simulator

Core Concepts
~~~~~~~~~~~~~

.. tab-set::

   .. tab-item:: World
      :sync: world

      .. card:: 🌍 carla.World
         
         The simulation environment containing all actors and providing global controls.
         
         **Key Methods:**
         
         .. code-block:: python

            # Get the world
            world = client.get_world()
            
            # Spawn actors
            actor = world.spawn_actor(blueprint, transform)
            
            # Get all actors
            actors = world.get_actors()
            
            # Control weather
            weather = carla.WeatherParameters.ClearNoon
            world.set_weather(weather)
            
            # Get map
            map = world.get_map()

   .. tab-item:: Blueprints
      :sync: blueprints

      .. card:: 📘 Blueprint Library
         
         Templates for creating actors (vehicles, sensors, pedestrians).
         
         **Vehicle Blueprints:**
         
         .. code-block:: python

            # Get blueprint library
            blueprint_library = world.get_blueprint_library()
            
            # Filter vehicles
            vehicles = blueprint_library.filter('vehicle.*')
            
            # Popular vehicles
            tesla_model3 = blueprint_library.find('vehicle.tesla.model3')
            audi_tt = blueprint_library.find('vehicle.audi.tt')
            lincoln_mkz = blueprint_library.find('vehicle.lincoln.mkz_2020')
            
            # Set attributes
            bp.set_attribute('color', '255,0,0')  # Red
            bp.set_attribute('role_name', 'hero')

         **Sensor Blueprints:**
         
         .. code-block:: python

            # RGB Camera
            camera_bp = blueprint_library.find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', '1920')
            camera_bp.set_attribute('image_size_y', '1080')
            camera_bp.set_attribute('fov', '110')
            
            # LiDAR
            lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('channels', '64')
            lidar_bp.set_attribute('points_per_second', '1000000')
            lidar_bp.set_attribute('rotation_frequency', '20')
            
            # Radar
            radar_bp = blueprint_library.find('sensor.other.radar')
            radar_bp.set_attribute('horizontal_fov', '30')
            radar_bp.set_attribute('vertical_fov', '10')

   .. tab-item:: Actors
      :sync: actors

      .. card:: 🎭 Actors
         
         Entities in the simulation (vehicles, pedestrians, sensors, traffic signs).
         
         **Vehicle Control:**
         
         .. code-block:: python

            # Manual control
            control = carla.VehicleControl()
            control.throttle = 0.5
            control.steer = 0.1
            control.brake = 0.0
            vehicle.apply_control(control)
            
            # Autopilot
            vehicle.set_autopilot(True)
            
            # Physics control
            physics = vehicle.get_physics_control()
            physics.mass = 1500.0
            physics.drag_coefficient = 0.3
            vehicle.apply_physics_control(physics)
            
            # Get vehicle state
            velocity = vehicle.get_velocity()
            acceleration = vehicle.get_acceleration()
            transform = vehicle.get_transform()

   .. tab-item:: Sensors
      :sync: sensors

      .. card:: 📡 Sensors
         
         Perception devices that capture simulation data.
         
         **Sensor Types:**
         
         .. list-table::
            :widths: 30 70
            :header-rows: 1
            
            * - **Sensor**
              - **Description**
            * - RGB Camera
              - Standard color camera
            * - Depth Camera
              - Distance to objects
            * - Semantic Segmentation
              - Object classification per pixel
            * - LiDAR
              - 3D point cloud
            * - Radar
              - Velocity and distance
            * - GNSS
              - GPS coordinates
            * - IMU
              - Acceleration and orientation
            * - Collision Detector
              - Collision events
            * - Lane Invasion
              - Lane crossing events
            * - Obstacle Detector
              - Nearby obstacles

         **Sensor Callbacks:**
         
         .. code-block:: python

            def camera_callback(image):
                # Process image
                array = np.frombuffer(image.raw_data, dtype=np.uint8)
                array = array.reshape((image.height, image.width, 4))
                array = array[:, :, :3]  # Remove alpha
                
            camera.listen(camera_callback)

Common Blueprint IDs
--------------------

.. dropdown:: 🚗 Vehicle Blueprints
   :color: primary
   :icon: rocket
   :animate: fade-in-slide-down

   .. grid:: 2
      :gutter: 2

      .. grid-item-card:: Sedans
         
         - ``vehicle.audi.a2``
         - ``vehicle.audi.tt``
         - ``vehicle.bmw.grandtourer``
         - ``vehicle.chevrolet.impala``
         - ``vehicle.lincoln.mkz_2017``
         - ``vehicle.lincoln.mkz_2020``
         - ``vehicle.mercedes.coupe``
         - ``vehicle.tesla.model3``

      .. grid-item-card:: SUVs
         
         - ``vehicle.audi.etron``
         - ``vehicle.jeep.wrangler_rubicon``
         - ``vehicle.nissan.patrol``
         - ``vehicle.toyota.prius``
         - ``vehicle.carlamotors.carlacola``

      .. grid-item-card:: Trucks
         
         - ``vehicle.tesla.cybertruck``
         - ``vehicle.carlamotors.firetruck``
         - ``vehicle.ford.ambulance``

      .. grid-item-card:: Motorcycles
         
         - ``vehicle.harley-davidson.low_rider``
         - ``vehicle.kawasaki.ninja``
         - ``vehicle.yamaha.yzf``
         - ``vehicle.vespa.zx125``

.. dropdown:: 📷 Sensor Blueprints
   :color: info
   :icon: device-camera
   :animate: fade-in-slide-down

   .. list-table::
      :widths: 40 60
      :header-rows: 1
      
      * - **Blueprint ID**
        - **Description**
      * - ``sensor.camera.rgb``
        - Standard RGB camera
      * - ``sensor.camera.depth``
        - Depth map camera
      * - ``sensor.camera.semantic_segmentation``
        - Semantic segmentation camera
      * - ``sensor.camera.instance_segmentation``
        - Instance segmentation camera
      * - ``sensor.camera.dvs``
        - Dynamic Vision Sensor
      * - ``sensor.lidar.ray_cast``
        - LiDAR sensor
      * - ``sensor.lidar.ray_cast_semantic``
        - Semantic LiDAR
      * - ``sensor.other.radar``
        - Radar sensor
      * - ``sensor.other.gnss``
        - GNSS/GPS sensor
      * - ``sensor.other.imu``
        - Inertial Measurement Unit
      * - ``sensor.other.collision``
        - Collision detector
      * - ``sensor.other.lane_invasion``
        - Lane invasion detector
      * - ``sensor.other.obstacle``
        - Obstacle detector

Weather Presets
---------------

.. card:: 🌤️ Weather Parameters
   :class-header: bg-info text-white

   .. code-block:: python

      # Available presets
      weather = carla.WeatherParameters.ClearNoon
      weather = carla.WeatherParameters.CloudyNoon
      weather = carla.WeatherParameters.WetNoon
      weather = carla.WeatherParameters.WetCloudyNoon
      weather = carla.WeatherParameters.MidRainyNoon
      weather = carla.WeatherParameters.HardRainNoon
      weather = carla.WeatherParameters.SoftRainNoon
      weather = carla.WeatherParameters.ClearSunset
      weather = carla.WeatherParameters.CloudySunset
      weather = carla.WeatherParameters.WetSunset
      weather = carla.WeatherParameters.WetCloudySunset
      weather = carla.WeatherParameters.MidRainSunset
      weather = carla.WeatherParameters.HardRainSunset
      weather = carla.WeatherParameters.SoftRainSunset
      
      # Custom weather
      weather = carla.WeatherParameters(
          cloudiness=80.0,        # 0-100
          precipitation=30.0,     # 0-100
          precipitation_deposits=50.0,  # 0-100
          wind_intensity=80.0,    # 0-100
          sun_azimuth_angle=70.0, # 0-360
          sun_altitude_angle=70.0,# -90-90
          fog_density=20.0,       # 0-100
          fog_distance=0.0,       # 0-inf
          wetness=0.0             # 0-100
      )
      
      world.set_weather(weather)

Quick Start Examples
--------------------

.. tab-set::

   .. tab-item:: Basic Connection
      :sync: basic

      .. code-block:: python

         import carla
         
         # Connect to CARLA
         client = carla.Client('localhost', 2000)
         client.set_timeout(10.0)
         
         # Get world
         world = client.get_world()
         
         # Get blueprint library
         blueprint_library = world.get_blueprint_library()
         
         # Print available vehicles
         vehicles = blueprint_library.filter('vehicle.*')
         for bp in vehicles:
             print(bp.id)

   .. tab-item:: Spawn Vehicle
      :sync: spawn

      .. code-block:: python

         import carla
         import random
         
         client = carla.Client('localhost', 2000)
         world = client.get_world()
         blueprint_library = world.get_blueprint_library()
         
         # Choose a vehicle
         bp = blueprint_library.find('vehicle.tesla.model3')
         bp.set_attribute('color', '0,0,255')  # Blue
         
         # Get a spawn point
         spawn_points = world.get_map().get_spawn_points()
         spawn_point = random.choice(spawn_points)
         
         # Spawn the vehicle
         vehicle = world.spawn_actor(bp, spawn_point)
         
         # Enable autopilot
         vehicle.set_autopilot(True)
         
         print(f"Spawned {vehicle.type_id} at {spawn_point.location}")

   .. tab-item:: Add Camera
      :sync: camera

      .. code-block:: python

         import carla
         import numpy as np
         
         # Setup (assuming vehicle exists)
         camera_bp = blueprint_library.find('sensor.camera.rgb')
         camera_bp.set_attribute('image_size_x', '800')
         camera_bp.set_attribute('image_size_y', '600')
         
         # Attach camera to vehicle
         camera_transform = carla.Transform(
             carla.Location(x=1.5, z=2.4)
         )
         camera = world.spawn_actor(
             camera_bp, 
             camera_transform, 
             attach_to=vehicle
         )
         
         # Process images
         def process_image(image):
             array = np.frombuffer(image.raw_data, dtype=np.uint8)
             array = array.reshape((image.height, image.width, 4))
             array = array[:, :, :3]  # RGB only
             # Process array...
             
         camera.listen(process_image)

   .. tab-item:: Full Example
      :sync: full

      .. code-block:: python

         import carla
         import random
         import time
         
         def main():
             # Connect
             client = carla.Client('localhost', 2000)
             client.set_timeout(10.0)
             world = client.get_world()
             
             # Set weather
             weather = carla.WeatherParameters.ClearNoon
             world.set_weather(weather)
             
             # Get blueprints
             blueprint_library = world.get_blueprint_library()
             
             # Spawn vehicle
             vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz_2020')
             spawn_point = random.choice(world.get_map().get_spawn_points())
             vehicle = world.spawn_actor(vehicle_bp, spawn_point)
             
             # Add camera
             camera_bp = blueprint_library.find('sensor.camera.rgb')
             camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
             camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
             
             # Enable autopilot
             vehicle.set_autopilot(True)
             
             try:
                 # Run for 30 seconds
                 time.sleep(30)
             finally:
                 # Cleanup
                 camera.destroy()
                 vehicle.destroy()
                 
         if __name__ == '__main__':
             main()

Performance Tips
----------------

.. card:: ⚡ Optimization Guide
   :class-header: bg-warning

   .. grid:: 1 1 2 2
      :gutter: 2

      .. grid-item-card:: For Better FPS
         
         - Use ``-quality-level=Low``
         - Add ``-nosound`` flag
         - Reduce sensor resolution
         - Use ``-fps=20`` limit
         - Disable unused sensors
         - Use ``-RenderOffScreen`` for headless

      .. grid-item-card:: For Better Quality
         
         - Use ``-quality-level=Epic``
         - Increase sensor resolution
         - Enable all weather effects
         - Use ``-vulkan`` on NVIDIA
         - Remove ``-nosound``
         - Use synchronous mode

      .. grid-item-card:: For Reproducibility
         
         - Use ``-deterministic``
         - Set ``-fixed-delta-seconds=0.05``
         - Use synchronous mode in API
         - Set random seeds
         - Fix weather parameters
         - Record and replay

      .. grid-item-card:: For Development
         
         - Use ``-windowed`` mode
         - Set comfortable resolution
         - Enable reload world
         - Use spectator mode
         - Quick map switching
         - Fast vehicle spawning

Troubleshooting
---------------

.. dropdown:: 🔧 Common Issues and Solutions
   :color: danger
   :icon: tools
   :animate: fade-in-slide-down

   .. tab-set::

      .. tab-item:: Graphics Issues

         **Black Screen on Launch**
         
         .. code-block:: bash

            # Try OpenGL instead of Vulkan
            ./CarlaUE4.sh -opengl -quality-level=Low
            
            # Or try DirectX on Windows
            ./CarlaUE4.sh -dx11

         **Low FPS**
         
         .. code-block:: bash

            # Reduce quality
            ./CarlaUE4.sh -quality-level=Low -nosound
            
            # Limit framerate
            ./CarlaUE4.sh -fps=20

         **GPU Memory Errors**
         
         .. code-block:: bash

            # Lower resolution
            ./CarlaUE4.sh -windowed -ResX=1280 -ResY=720

      .. tab-item:: Connection Issues

         **Cannot Connect to CARLA**
         
         .. code-block:: python

            # Check if CARLA is running
            import socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            result = sock.connect_ex(('localhost', 2000))
            if result == 0:
                print("CARLA is running")
            else:
                print("CARLA is not accessible")

         **Timeout Errors**
         
         .. code-block:: python

            # Increase timeout
            client = carla.Client('localhost', 2000)
            client.set_timeout(30.0)  # 30 seconds

      .. tab-item:: Docker Issues

         **Permission Denied**
         
         .. code-block:: bash

            # Add user to docker group
            sudo usermod -aG docker $USER
            newgrp docker

         **No Display**
         
         .. code-block:: bash

            # Allow X11 forwarding
            xhost +local:root
            
            # Include in docker run
            -e DISPLAY=$DISPLAY \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw

         **NVIDIA GPU Not Available**
         
         .. code-block:: bash

            # Install nvidia-docker
            distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
            curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
            curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
              sudo tee /etc/apt/sources.list.d/nvidia-docker.list
            sudo apt-get update && sudo apt-get install -y nvidia-docker2
            sudo systemctl restart docker

References
----------

.. card:: 📚 Additional Resources
   :class-header: bg-dark text-white

   - **Official Documentation**: https://carla.readthedocs.io/en/0.9.16/
   - **Python API Reference**: https://carla.readthedocs.io/en/0.9.16/python_api/
   - **Blueprint Library**: https://carla.readthedocs.io/en/0.9.16/bp_library/
   - **GitHub Repository**: https://github.com/carla-simulator/carla
   - **ROS Bridge Issue**: https://github.com/carla-simulator/carla/issues/9278
   - **Discord Community**: https://discord.com/invite/8kqACuC