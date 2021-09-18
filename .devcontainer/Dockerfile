FROM osrf/ros:noetic-desktop-full-focal

# Install tools
RUN apt-get update && apt-get install git wget -y
RUN apt-get update && apt-get install python3-catkin-tools python3-osrf-pycommon -y

# Source setup file
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && source ~./bashrc

# TODO install serial github repo and setup vscode stuff