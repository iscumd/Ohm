FROM osrf/ros:noetic-desktop-full-focal

SHELL ["/bin/bash", "-c"]

# Install tools
RUN apt-get update && apt-get install git wget -y
RUN apt-get update && apt-get install python3-catkin-tools python3-osrf-pycommon -y

# Source setup file, then install libs as they need the catkin bash sourced.
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && source /opt/ros/noetic/setup.bash \ 
    # Clone serial deps
    && git clone https://github.com/wjwwood/serial.git \ 
    && git clone https://github.com/wjwwood/serial_utils.git 

# TODO switch to non-root user to avoid the files being permissioned locked
# TODO install rust ;) (and OpenCv)