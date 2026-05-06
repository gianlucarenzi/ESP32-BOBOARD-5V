TARGET = sid_dump_player

.PHONY: all clean

all: $(TARGET).xex

$(TARGET).xex: $(TARGET).c
	cl65 -t atari -O -o $@ $<

clean:
	rm -f $(TARGET).o $(TARGET).xex
