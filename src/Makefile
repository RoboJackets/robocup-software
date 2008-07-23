CDIRS=Logger Motion Radio Referee Skills Soccer SoccSim \
	  util/grab util/motor_test util/radio_test
DIRS=util/sslrefbox util/watchdog/cpp

ROOT=$(PWD)

.PHONY: all
all:
		+@for i in $(CDIRS); do cd $$i/bin; cmake -Wno-dev blah ..; make; cd $(PWD); done
		+@for i in $(DIRS); do make -C $$i; done

clean:
		+@for i in $(CDIRS); do make -C $$i/bin clean; done
		+@for i in $(DIRS); do make -C $$i clean; done

distclean:
		+@for i in $(CDIRS); do echo Cleaning $$i/bin; rm -rf $$i/bin/*; done
		+@for i in $(DIRS); do make -C $$i clean; done
