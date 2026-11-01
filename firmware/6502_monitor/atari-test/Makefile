# Atari test program for the 6502_monitor banked RAM (cc65 toolchain).
#   make          build banked_ram_test.xex
#   make sim      run it on py65 against a model of the device (needs py65)
#   make simall   same, with every injected fault: each must be detected

PROG   := banked_ram_test
CL65   ?= cl65
PYTHON ?= python3
FAULTS := bankreg_ignored latch_ignored stuck_d3 alias_a6 shared_windows \
          bank_alias_7bit writes_when_deselected

$(PROG).xex: $(PROG).s
	$(CL65) -t atari -C atari-asm.cfg -o $@ $<

sim: $(PROG).xex
	$(PYTHON) sim_banked_ram.py $< none

simall: $(PROG).xex
	@for f in none $(FAULTS); do $(PYTHON) sim_banked_ram.py $< $$f | tail -1 | sed "s/^/$$f: /"; done

clean:
	rm -f $(PROG).xex *.o *.lst *.map

.PHONY: sim simall clean
