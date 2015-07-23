
# 'latest' is the latest tagged on the machine, which should be the one created by the baseimage script.
FROM jgkamat/robocup-baseimage:latest
MAINTAINER Jay Kamat

WORKDIR /home/developer/robocup-software

# Setup dockerimage to work with new repo
RUN make clean
RUN rm -rf .git
ADD .git/ .
RUN git reset --hard
RUN git clean -f

# Build
RUN make
RUN make robot2015
RUN make test-cpp

WORKDIR /home/developer/robocup-software/run

# to run X11 Server
# docker run -ti --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix <COMMAND NAME>
