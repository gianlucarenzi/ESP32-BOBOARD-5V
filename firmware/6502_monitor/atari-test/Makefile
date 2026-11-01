# Atari test program for the 6502_monitor banked RAM (cc65 toolchain).
#   make          build banked_ram_test.xex
#   make sim      run it on py65 against a model of the device (needs py65),
#                 300 banks
#   make simall   same, with every injected fault: each must be detected

PROG   := banked_ram_test
CL65   ?= cl65
PYTHON ?= python3
FAULTS := bankreg_ignored bank_hi_ignored latch_ignored latch_any_bit stuck_d3 \
          alias_a6 writes_when_deselected limit_wraps count_writable \
          offset_ff_dead ram_lost_on_deselect

$(PROG).xex: $(PROG).s
	$(CL65) -t atari -C atari-asm.cfg -o $@ $<

sim: $(PROG).xex
	$(PYTHON) sim_banked_ram.py $< none

simall: $(PROG).xex
	@for f in none $(FAULTS); do $(PYTHON) sim_banked_ram.py $< $$f | tail -1 | sed "s/^/$$f: /"; done

clean:
	rm -f $(PROG).xex *.o *.lst *.map

.PHONY: sim simall clean
