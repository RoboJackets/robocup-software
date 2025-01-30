# Use phusion/baseimage if problems arise
FROM ros:humble-ros-base-jammy
LABEL maintainer="oswinso@gmail.com"

# Setup apt to be happy with no console input
ENV DEBIAN_FRONTEND noninteractive

# setup apt tools and other goodies we want
RUN apt-get update --fix-missing && apt-get -y install udev locales git ssh nano vim software-properties-common sudo tzdata keyboard-configuration && apt-get clean

# Prevent bugging us later about timezones
RUN ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime && dpkg-reconfigure --frontend noninteractive tzdata

# Use UTF-8
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

COPY . ~/robocup-software
WORKDIR ~/robocup-software

RUN sudo ./util/ubuntu-setup --yes --no-submodules
