
FILES=robocup.gv
OUT=robocup.svg



$(OUT) : $(FILES)
	dot -Tsvg $(FILES) -o $(OUT)

clean: $(OUT)
	rm -f $(OUT)
