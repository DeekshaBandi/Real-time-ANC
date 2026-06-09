CC      ?= cc
CFLAGS  ?= -O2 -Wall -Wextra
LDLIBS  := -lm

fxlms_rt: fxlms_rt.c
	$(CC) $(CFLAGS) -o $@ fxlms_rt.c $(LDLIBS)

.PHONY: clean
clean:
	rm -f fxlms_rt data/e_c.txt
