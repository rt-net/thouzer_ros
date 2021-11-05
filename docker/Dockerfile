FROM osrf/ros:melodic-desktop-full

WORKDIR /root/catkin_ws
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    apt-get update && \
    apt-get install -y python-vcstool python-catkin-tools python-pip python-mock \
            vim less bash bash-completion tmux byobu iputils-ping net-tools dnsutils \
            terminator && \
    rm -rf /var/lib/apt/lists/*
# Avoid org.freedesktop.DBus.Error.Spawn.ExecFailed
# https://forums.bunsenlabs.org/viewtopic.php?pid=59732#p59732
RUN mkdir -p /root/.config/terminator && \
    echo '[global_config]' | tee -a /root/.config/terminator/config && \
    echo '    dbus = "False"' | tee -a /root/.config/terminator/config
RUN mkdir -p src && \
    . /opt/ros/melodic/setup.sh && \
    catkin init && \
    catkin build
WORKDIR /root/catkin_ws/src/thouzer_ros
COPY .melodic_depends.rosinstall /root/catkin_ws/src/thouzer_ros/
COPY thouzer_bringup /root/catkin_ws/src/thouzer_ros/thouzer_bringup
COPY thouzer_description /root/catkin_ws/src/thouzer_ros/thouzer_description
COPY thouzer_driver /root/catkin_ws/src/thouzer_ros/thouzer_driver
COPY thouzer_gui /root/catkin_ws/src/thouzer_ros/thouzer_gui
COPY thouzer_msgs /root/catkin_ws/src/thouzer_ros/thouzer_msgs
COPY thouzer_slam /root/catkin_ws/src/thouzer_ros/thouzer_slam
COPY thouzer_teleop /root/catkin_ws/src/thouzer_ros/thouzer_teleop
COPY thouzer_navigation /root/catkin_ws/src/thouzer_ros/thouzer_navigation
COPY thouzer_waypoints /root/catkin_ws/src/thouzer_ros/thouzer_waypoints
RUN apt-get update && \
    . /opt/ros/melodic/setup.sh && \
    vcs import < .melodic_depends.rosinstall && \
    pip install -r mqtt_bridge/requirements.txt && \
    rosdep update && \
    rosdep install -r -y -i --from-paths . && \
    rm -rf /var/lib/apt/lists/*
WORKDIR /root/catkin_ws
RUN . /opt/ros/melodic/setup.sh && \
    catkin build && \
    . /root/catkin_ws/devel/setup.sh && \
    sed -i -e "s/^exec/#exec/g" /ros_entrypoint.sh && \
    echo "source /root/catkin_ws/devel/setup.bash" | tee -a /ros_entrypoint.sh && \
    echo 'exec "$@"' | tee -a /ros_entrypoint.sh && \
    echo "source /root/catkin_ws/devel/setup.bash" | tee -a /root/.bashrc && \
    echo "echo ========================" | tee -a /root/.bashrc && \
    echo "echo ------ thouzer commands ------" | tee -a /root/.bashrc && \
    echo "echo debugger        : roslaunch thouzer_bringup thouzer_remote_debugger.launch" | tee -a /root/.bashrc && \
    echo "echo motor control on: rosservice call /motor_on" | tee -a /root/.bashrc && \
    echo "echo ========================" | tee -a /root/.bashrc
ENV ROS_MASTER_URI=http://127.0.0.1:11311
ENV ROS_IP=127.0.0.1