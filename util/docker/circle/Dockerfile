FROM jgkamat/robocup-baseimage
MAINTAINER Jay Kamat

WORKDIR /home/developer/robocup-software

# Setup dockerimage to work with new repo
RUN make clean
RUN rm -rf .git
ADD .git/ .
RUN git reset --hard
RUN git clean -f

RUN util/ubuntu-setup --firmware -y && sudo apt-get clean

# Build
RUN make
RUN make robot2015
RUN make test-cpp

WORKDIR /home/developer/robocup-software/run

# to run X11 Server
# docker run -ti --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix <COMMAND NAME>
