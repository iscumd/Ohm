# Authored by Andrew Ealovega on 9/21/21.
# Sets up a docker enviorment for building and develuping Ohm.

FROM osrf/ros:noetic-desktop-full-focal

SHELL ["/bin/bash", "-c"]

# Install tools
RUN sudo apt-get update && sudo apt-get install git wget -y
RUN sudo apt-get update && sudo apt-get install python3-catkin-tools python3-osrf-pycommon -y

# Source setup file, then install libs as they need the catkin bash sourced.
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && source /opt/ros/noetic/setup.bash \ 
    # Clone serial deps
    && git clone https://github.com/wjwwood/serial.git \ 
    && git clone https://github.com/wjwwood/serial_utils.git 

# TODO switch to non-root user to avoid the files being permissioned locked

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

RUN sudo apt update && sudo apt-get install -y libraw1394-11 \
    ffmpeg libgtkmm-2.4-dev libglademm-2.4-dev \
    libgtkglextmm-x11-1.2-dev libusb-1.0-0

# Because of docker contexts, we need a copy of the lib in this dir to copy into the container.
COPY flycapture2-2.13.3.31-amd64 flycapture2
RUN cd flycapture2 && yes $'yes\nno' | sudo sh install_flycapture.sh

########
# Misc #
########

# Install rust for future experimentation
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y

# Run as non root
RUN useradd -ms /bin/bash vscode
USER vscode