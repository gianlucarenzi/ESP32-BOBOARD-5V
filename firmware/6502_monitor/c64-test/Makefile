# C64 test program for the 6502_monitor GeoRAM, env c64 (cc65 toolchain).
#   make          build georam_test.prg
#   make sim      run it on py65 against a model of the GeoRAM (needs py65)
#   make simall   same, with every injected fault: each must be detected

PROG   := georam_test
CL65   ?= cl65
PYTHON ?= python3
FAULTS := page_reg_ignored block_reg_ignored page_8bit no_alias stuck_d3 \
          alias_a6 no_offset_ff small block_mask_low nodev

$(PROG).prg: $(PROG).s
	$(CL65) -t c64 -C c64-asm.cfg -u __EXEHDR__ -o $@ $<

sim: $(PROG).prg
	$(PYTHON) sim_georam.py $< none

simall: $(PROG).prg
	@for f in none $(FAULTS); do $(PYTHON) sim_georam.py $< $$f | tail -1 | sed "s/^/$$f: /"; done

clean:
	rm -f $(PROG).prg *.o *.lst *.map

.PHONY: sim simall clean
