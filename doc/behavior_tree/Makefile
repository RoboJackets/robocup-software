.PHONY: all clean

GRAPHS=general play_a play_b play_c play_d play_e play_f
IMAGE_FORMAT=svg

IMAGES=$(addsuffix .$(IMAGE_FORMAT), $(GRAPHS))

all:	$(IMAGES)

clean:
	rm -f $(IMAGES)

%.$(IMAGE_FORMAT):	%.dot
	dot -T$(IMAGE_FORMAT) -o $@ $<
