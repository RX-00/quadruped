CC = g++
CFLAGS := -std=c++11
#CFLAGS := -std=gnu++11

exe_file = exe.o

# Handle debug case
DEBUG ?= 1
ifeq ($(DEBUG), 1)
	CFLAGS := -g -Wall
else
	CFLAGS := -DNDEBUG -O3
endif

SRCEXT := cpp
HEADEREXT := h
SOURCES := $(shell find $(SRCDIR) -type f -name "*.$(SRCEXT)")
OBJECTS := $(patsubst %.$(SRCEXT),%.o,$(SOURCES))

$(exe_file): $(OBJECTS)
	$(CC) $^ -o $(exe_file)
%.o: %.$(SRCEXT) %.$(HEADEREXT)
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f *.out *.o $(exe_file)

.PHONY: clean

