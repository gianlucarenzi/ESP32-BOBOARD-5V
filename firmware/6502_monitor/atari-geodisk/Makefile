# GEODISK.SYS: D9: RAM disk on the 6502_monitor paged RAM (cc65 toolchain).
#   make          build GEODISK.SYS (resident part relocated, installer, XEX)
#   make test     run it on py65 with a fake OS and the device model (needs py65)

CL65   ?= cl65
PYTHON ?= python3

GEODISK.SYS: geodisk.s geodisk_res.s mkgeodisk.py
	$(PYTHON) mkgeodisk.py $(CL65)

test: GEODISK.SYS
	$(PYTHON) test_geodisk.py GEODISK.SYS 300 1F40
	$(PYTHON) test_geodisk.py GEODISK.SYS 300 2345

clean:
	rm -f GEODISK.SYS res.inc *.bin *.o *.lst *.map

.PHONY: test clean
