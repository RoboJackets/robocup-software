
# 'latest' is the latest tagged on the machine, which should be the one created by the baseimage script.
FROM jgkamat/robocup-baseimage:latest
MAINTAINER Jay Kamat github@jgkamat.33mail.com

WORKDIR /home/developer/robocup-software

# Use the new repo
RUN sudo rm -rf /home/developer/robocup-software/.git
COPY .git/ .git/
RUN sudo chown -R developer:developer /home/developer/robocup-software
RUN git submodule deinit -f .
RUN git submodule update --init
RUN git reset --hard HEAD
RUN git clean -f
RUN git submodule update --init

# Build
RUN make clean
RUN make
RUN make robot2015
RUN make test-cpp

WORKDIR /home/developer/robocup-software/run

# to run X11 Server
# docker run -ti --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix <COMMAND NAME>
CMD [ "/home/developer/robocup-software/run/simulator & /home/developer/robocup-software/run/soccer -sim" ]
