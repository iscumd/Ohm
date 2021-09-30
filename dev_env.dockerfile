# Authored by Andrew Ealovega on 9/21/21.
# Sets up a docker enviorment for building and develuping Ohm.

FROM osrf/ros:noetic-desktop-full-focal

SHELL ["/bin/bash", "-c"]

# Make root's password 1234
RUN echo 'root:1234' | chpasswd

# Create a non root user with a password of 1234
RUN useradd --create-home --shell /bin/bash vscode -p 1234
RUN echo 'vscode:1234' | chpasswd
RUN adduser vscode sudo

# Install tools
RUN sudo apt-get update && sudo apt-get install git wget -y
RUN sudo apt-get update && sudo apt-get install python3-catkin-tools python3-osrf-pycommon -y
RUN sudo apt-get install python3-vcstool

# Source setup file
USER vscode
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

USER root

####################################################
# Install system OpenCv 4 and related dependancies #
####################################################

RUN sudo apt update && sudo apt install libopencv-dev

RUN sudo apt update && sudo apt install -y python-dev python3-dev python-numpy python3-numpy ccache zlibc libjpeg-dev libtiff-dev libwebp-dev libpng-dev libopenexr-dev libgtk2.0-dev libgtk-3-dev \
libdc1394-22-dev ffmpeg libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libavresample-dev libatlas-base-dev libatlas-cpp-0.6-dev libblas-dev liblapack-dev liblapacke-dev openjdk-8-jdk \
ant git libopenexr-dev libeigen3-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libv4l-dev libgflags-dev libgoogle-glog-dev hdf5-tools libhdf5-dev libtesseract-dev libleptonica-dev libceres-dev

#######################
# Install flycapture2 #
#######################

# Install listed deps
RUN sudo apt update && sudo apt-get install -y libraw1394-11 \
    ffmpeg libgtkmm-2.4-dev libglademm-2.4-dev \
    libgtkglextmm-x11-1.2-dev libusb-1.0-0

# Install the lib itself
COPY /src/white_line_detection/flycapture2-2.13.3.31-amd64 flycapture2
RUN cd flycapture2 && yes $'yes\nno' | sudo sh install_flycapture.sh 

########
# Misc #
########

# Install rust for future experimentation
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y

USER vscode
RUN rosdep update