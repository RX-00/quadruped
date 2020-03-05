# Recursively get all *.cpp in this directory and any sub-directories
SRC = $(shell find . -name *.cc) $(shell find . -name *.cpp)

CC = g++
CFLAGS := -std=c++11

# This tells Make that somewhere below, you are going to convert all your source into objects
OBJ := $(addsuffix .o, $(basename $(SRCS)))

#exe_file = run

# Handle debug case
DEBUG ?= 1
ifeq ($(DEBUG), 1)
	CFLAGS := -g -Wall
else
	CFLAGS := -DNDEBUG -O3
endif

#SRCEXT := cpp
#HEADEREXT := h
#SOURCES := $(shell find $(SRCDIR) -type f -name "*.$(SRCEXT)")
#OBJECTS := $(patsubst %.$(SRCEXT),%.o,$(SOURCES))

#$(exe_file): $(OBJECTS)
#	$(CC) $^ -o $(exe_file)
#%.o: %.$(SRCEXT) %.$(HEADEREXT)
#	$(CC) $(CFLAGS) -c -o $@ $<



# Tells make your binary is called artifact_name_here and it should be in bin/
BIN = run

# all is the target (you would run make all from the command line). 'all' is dependent on $(BIN)
all: $(BIN)

# $(BIN) is dependent on objects
$(BIN): $(OBJ)

# each object file is dependent on its source file, and whenever make needs to create
# an object file, to follow this rule:
%.o: %.cc g++ -c $(INCLUDE_PATHS) $< -o $@




clean:
	rm -f *.out *.o $(exe_file)

.PHONY: clean

