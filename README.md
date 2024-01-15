# Turtlebot controller ros package

 This package allows you to control the turtlebot by detecting the aruco tag in the camera image.

 Build docker image with the following Dockerfile to accomplish this package functionality:

``` dockerfile
FROM ros:humble

RUN apt-get update && apt-get -y upgrade &&\
    apt-get -y install ros-humble-desktop ros-humble-usb-cam ros-humble-turtlebot3* python3-pip

RUN pip install setuptools==58.2.0

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc &&\
    echo "source ~/niosdr_project_ws/install/setup.bash" >> ~/.bashrc &&\
    echo "cd ~/niosdr_project_ws" >> ~/.bashrc &&\
    echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
    
RUN git clone https://github.com/krzy3chu/NiOSDR_Project.git ~/niosdr_project_ws/src/turtle_controller

RUN cd ~/niosdr_project_ws/ &&\
    colcon build --symlink-install
```

Inside container run from image above, execute these nodes:
* camera node,
``` bash
ros2 run usb_cam usb_cam_node_exe 
```

* this package turtlebot controller node,
``` bash
ros2 run turtle_controller controller_node
```

* [turtlebot3 Gazebo simulation.](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
``` bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

