
protocol.capnp.c++: protocol.capnp
	capnp compile -oc++ protocol.capnp


.PHONY: encode

encode: example.txt protocol.capnp
	@echo "Number of bytes without packing:"
	@cat example.txt \
	| capnp encode protocol.capnp RadioPacket \
	| wc -c

	@echo "Number of bytes with packing:"
	@cat example.txt \
	| capnp encode -p protocol.capnp RadioPacket \
	| wc -c
