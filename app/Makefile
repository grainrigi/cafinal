all:
	iverilog code205.v
	./a.out
run:
	cat memory_image.bin > /dev/ttyUSB1

clean:
	rm -f *~ a.out
