# Use phusion/baseimage if problems arise
FROM ubuntu:14.04
MAINTAINER Jay Kamat

# This dockerimage will serve as a 'static' base for the robocup dockerimage to reduce build time

# Setup apt to be happy with no console input
ENV DEBIAN_FRONTEND noninteractive

# setup apt tools and other goodies we want
RUN apt-get update --fix-missing && apt-get -y install apt-utils wget curl htop iputils-ping vim-nox less debconf-utils w3m git software-properties-common sudo scons screen && apt-get clean

# set up user <this is for running soccer later on>
# Replace 1000 with your user / group id
RUN export uid=1000 gid=1000 && \
    mkdir -p /home/developer && \
    echo "developer:x:${uid}:${gid}:Developer,,,:/home/developer:/bin/bash" >> /etc/passwd && \
    echo "developer:x:${uid}:" >> /etc/group && \
    echo "developer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer && \
    chown ${uid}:${gid} -R /home/developer

USER developer
ENV HOME /home/developer

# clone and run ubuntu-setup to shave some time off. This will be run again in the robocup-software to compensate for changes to the setup script.

# do everything in root's home
RUN mkdir -p /home/developer/robocup-software
WORKDIR /home/developer/robocup-software
ADD * .

WORKDIR /home/developer/robocup-software
RUN util/ubuntu-setup --yes --firmware || true && sudo apt-get clean

# Do a baseline make to speed up future compiles
WORKDIR /home/developer/robocup-software
RUN make && make robot2015

# soccer needs to be run in current directory for some reason :/
WORKDIR /home/developer/robocup-software/run

# Add a docker init system and use it
ADD smell-baron /bin/smell-baron
ENTRYPOINT ["/bin/smell-baron"]
CMD [ "/home/developer/robocup-software/run/simulator & /home/developer/robocup-software/run/soccer -sim" ]
