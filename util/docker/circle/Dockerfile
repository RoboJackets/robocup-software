
# 'latest' is the latest tagged on the machine, which should be the one created by the baseimage script.
FROM robojackets/robocup-baseimage:latest_build
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

CMD [ "/home/developer/robocup-software/run/simulator & /home/developer/robocup-software/run/soccer -sim" ]


