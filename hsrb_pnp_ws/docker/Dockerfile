FROM osrf/ros:jazzy-desktop-full

USER root

ARG USERNAME=hsrb

RUN usermod -l $USERNAME ubuntu && \
    groupmod -n $USERNAME ubuntu && \
    usermod -d /home/$USERNAME -m $USERNAME

RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt-get update && sudo apt upgrade -y

# Install Git
RUN sudo apt-get install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

################################
## Building ros2 commands
################################
WORKDIR /workdir
COPY ../src/ src/

USER root
RUN chmod a+w src/hsr_repos_ignition_jazzy/tmc_gazebo/tmc_gazebo_worlds/worlds/
USER $USERNAME

# Build Dependencies
RUN sudo apt-get update &&\
    sudo apt-get install -y ros-jazzy-ros-gz && \
    sudo apt-get install -y ros-jazzy-gz-ros2-control && \
    sudo apt-get install -y ros-jazzy-twist-mux && \
    sudo apt-get install -y ros-jazzy-twist-stamper && \
    sudo apt-get install -y ros-jazzy-ros2-control && \
    sudo apt-get install -y ros-jazzy-ros2-controllers && \
    sudo apt-get install -y ros-jazzy-filters && \
    sudo apt-get install -y ros-jazzy-rmw-cyclonedds-cpp


#====  Install runtime Dependencies ====
RUN sudo apt-get update && sudo apt-get install -y python3-pip
RUN sudo pip install --break-system-packages "numpy-stl==3.2.0"
RUN sudo apt-get update && \
    sudo apt-get install -y ros-jazzy-joint-state-publisher && \
    sudo apt-get install -y ros-jazzy-depth-image-proc && \
    sudo apt-get install -y ros-jazzy-pointcloud-to-laserscan && \
    sudo apt-get install -y ros-jazzy-tf-transformations && \
    sudo apt-get install -y ros-jazzy-topic-tools && \
    sudo apt-get install -y ros-jazzy-nav2-common && \
    sudo apt-get install -y ros-jazzy-octomap-ros && \
    sudo apt-get install -y ros-jazzy-nav2-bringup && \
    sudo apt-get install -y ros-jazzy-joy && \
    sudo apt-get install -y ros-jazzy-joy-linux && \
    sudo apt-get install -y ros-jazzy-joint-state-broadcaster && \
    sudo apt-get install -y ros-jazzy-tinyxml-vendor

RUN sudo apt-get update && \
    # Moveit2
    sudo apt-get install -y ros-jazzy-moveit && \
    # Pinochio
    sudo apt-get install -y ros-jazzy-pinocchio && \
    # Lib
    sudo apt install -y libsdl1.2-dev libsdl-image1.2-dev


#====  Build the necessary packages ====
SHELL ["/bin/bash", "-c"]
RUN . "/opt/ros/jazzy/setup.bash" && \
    # Build Base controllers
    colcon build --packages-up-to hsrb_base_controllers --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Hsrb moveit
    colcon build --packages-up-to hsrb_moveit_config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Hsrb common launch
    colcon build --packages-up-to hsrb_common_launch --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Downloaded with 'git clone https://github.com/ros-controls/gz_ros2_control.git'
    # Branch, 0.7.12
    # IGN, located locally in hsrb_pnp_ws/others/gz_ros2_control
    colcon build --packages-select ign_ros2_control --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Build Gazebo Bringup
    colcon build --packages-up-to hsrb_gazebo_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Build Gazebo Launch
    colcon build --packages-up-to hsrb_gazebo_launch --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Build Hsrb Interface
    colcon build --packages-up-to hsrb_interface_py --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Graspnet messages
    colcon build --packages-select graspnet_ros_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Build pick and place package for hsrb
    colcon build --packages-select hsrb_pick_and_place --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install && \
    # Build pick and place messages for hsrb, especially for specify place pose
    colcon build --packages-select hsrb_pnp_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release


# Setup colcon install folder's pass
RUN echo "source /opt/ros/jazzy/setup.bash" | sudo tee /etc/profile.d/ros_base_setup.sh && \
    echo "source /workspace/install/setup.bash" | sudo tee /etc/profile.d/ros_ws_setup.sh && \
    sudo chmod +x /etc/profile.d/ros_base_setup.sh /etc/profile.d/ros_ws_setup.sh

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

CMD ["bash"]
