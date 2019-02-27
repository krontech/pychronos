# Simple makefile to automate repetetive development tasks.

PYTHON ?= python3

all: inplace
.PHONY: clean inplace

clean:
	rm -f *.so
	$(PYTHON) setup.py clean

inplace:
	$(PYTHON) setup.py build_ext --inplace

