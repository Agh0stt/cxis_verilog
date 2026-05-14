# ── CXIS Makefile ────────────────────────────────────────────────────────────
CC      = gcc
CFLAGS  = -O2 -Wall -Iinclude
LDFLAGS = -lm

# ── Directories ───────────────────────────────────────────────────────────────
BINDIR  = bin
SRCDIR  = src
SIMDIR  = sim

# ── Tools ─────────────────────────────────────────────────────────────────────
TOOLS = $(BINDIR)/cxvm $(BINDIR)/cxas $(BINDIR)/cxld \
        $(BINDIR)/cxdis $(BINDIR)/cxstrip $(BINDIR)/headermod

all: $(BINDIR) $(TOOLS)

$(BINDIR):
	mkdir -p $(BINDIR)

$(BINDIR)/cxvm:      $(SRCDIR)/cxvm.c      include/cxis.h include/cxe.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

$(BINDIR)/cxas:      $(SRCDIR)/cxas.c      include/cxis.h include/cxo.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

$(BINDIR)/cxld:      $(SRCDIR)/cxld.c      include/cxis.h include/cxo.h include/cxe.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

$(BINDIR)/cxdis:     $(SRCDIR)/cxdis.c     include/cxis.h include/cxe.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

$(BINDIR)/cxstrip:   $(SRCDIR)/cxstrip.c   include/cxe.h include/cxis.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

$(BINDIR)/headermod: $(SRCDIR)/headermod.c include/cxe.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

# ── Shortcuts (so you can still type ./cxas etc from project root) ────────────
cxas cxld cxdis cxvm cxstrip headermod:
	@$(MAKE) $(BINDIR)/$@

# ── Simulation ────────────────────────────────────────────────────────────────
HEX ?= $(SIMDIR)/flash.hex
CXE ?= tests/hello.cxe

# Absolute path so $readmemh works regardless of vvp's cwd
ABS_HEX = $(abspath $(HEX))

hex:
	python3 cxe2hex.py $(CXE) $(HEX)

# Run flash simulation for a given .cxe
# Usage: make sim-flash CXE=tests/add.cxe
sim-flash: hex
	iverilog -DHEX_FILE=\"$(ABS_HEX)\" \
	         -o $(SIMDIR)/cxis_flash_sim \
	         $(SIMDIR)/cxis_flash_tb.v \
	         $(SIMDIR)/cxis_cpu.v \
	         $(SIMDIR)/cxis_mul.v \
	         $(SIMDIR)/cxis_div.v \
	         $(SIMDIR)/cxis_fpu.v
	vvp $(SIMDIR)/cxis_flash_sim

# Run flash sim for ALL test .cxe files
sim-all:
	@for f in tests/*.cxe; do \
	    base=$$(basename $$f .cxe); \
	    echo "=== $$base ==="; \
	    python3 cxe2hex.py $$f /tmp/$$base.hex; \
	    iverilog -DHEX_FILE=\"/tmp/$$base.hex\" \
	             -DSIM_TIMEOUT=2000000 \
	             -o /tmp/$${base}_sim \
	             $(SIMDIR)/cxis_flash_tb.v \
	             $(SIMDIR)/cxis_cpu.v \
	             $(SIMDIR)/cxis_mul.v \
	             $(SIMDIR)/cxis_div.v \
	             $(SIMDIR)/cxis_fpu.v; \
	    vvp /tmp/$${base}_sim; \
	done

# Legacy testbench (if cxis_cpu_tb.v exists)
sim-legacy:
	iverilog -o $(SIMDIR)/cxis_legacy_tb \
	         $(SIMDIR)/cxis_cpu_tb.v \
	         $(SIMDIR)/cxis_cpu.v
	vvp $(SIMDIR)/cxis_legacy_tb

# Run FPU unit testbench (standalone — no flash/hex needed)
# Usage: make sim-fpu
sim-fpu:
	iverilog -o $(SIMDIR)/cxis_fpu_sim \
	         $(SIMDIR)/cxis_fpu_tb.v \
	         $(SIMDIR)/cxis_fpu.v
	vvp $(SIMDIR)/cxis_fpu_sim

# ── Assemble + link a single test ─────────────────────────────────────────────
# Usage: make run TEST=add
TEST ?= hello
run:
	$(BINDIR)/cxas tests/$(TEST).cxis -o tests/$(TEST).cxo
	$(BINDIR)/cxld tests/$(TEST).cxo -o tests/$(TEST).cxe
	$(MAKE) sim-flash CXE=tests/$(TEST).cxe

# ── Clean ─────────────────────────────────────────────────────────────────────
clean:
	rm -f $(TOOLS)
	rm -f tests/*.cxo tests/*.cxe tests/*.cxbin
	rm -f $(SIMDIR)/flash.hex $(SIMDIR)/*.vcd
	rm -f $(SIMDIR)/cxis_flash_sim $(SIMDIR)/cxis_legacy_tb

.PHONY: all clean hex sim-flash sim-fpu sim-all sim-legacy run \
        cxas cxld cxdis cxvm cxstrip headermod
