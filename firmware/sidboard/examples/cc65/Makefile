# SID dump player (cc65): one executable per sidboard build.
#   make   sid_dump_player.xex (sid-atari-cctl, $D500) and
#          sid_dump_player_pbi.xex (sid-atari-pbi, $D100 + select $D1FF),
#          ../SIDDUMP.BIN from ../Commando_dmp.h

all: sid_dump_player.xex sid_dump_player_pbi.xex ../SIDDUMP.BIN

sid_dump_player.xex: sid_dump_player.c
	cl65 -t atari -O -DSID_PBI=0 -o $@ $<

sid_dump_player_pbi.xex: sid_dump_player.c
	cl65 -t atari -O -DSID_PBI=1 -o $@ $<

../SIDDUMP.BIN: ../Commando_dmp.h ../mkdump.py
	cd .. && python3 mkdump.py Commando_dmp.h SIDDUMP.BIN

clean:
	rm -f *.o *.xex ../SIDDUMP.BIN

.PHONY: all clean
