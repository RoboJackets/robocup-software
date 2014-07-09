
# This makefile build all of the .dot files into image files

TYPE=png # this can be png or svg (maybe others too...)
SRC=$(wildcard *.dot)
OUT=$(patsubst %.dot, %.$(TYPE), $(SRC))

all: $(OUT)

%.$(TYPE): %.dot
	dot -T$(TYPE) $^ -o $@

clean:
	rm *.png *.svg

