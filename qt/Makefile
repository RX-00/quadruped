# Nice and simple versatile Makefile found from:
# https://stackoverflow.com/questions/9178285/how-can-makefile-use-separate-directories-for-source-code-and-binaries
# Then I edited it so it plays nice with my header files

CC          = g++
LD          = g++
CFLAG       = -Wall
PROG_NAME   = quadruped


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

SRC_DIR     = ./src
BUILD_DIR   = ./build
BIN_DIR     = ./bin
SRC_LIST = $(wildcard $(SRC_DIR)/*.cpp)
OBJ_LIST = $(BUILD_DIR)/$(notdir $(SRC_LIST:.cpp=.o))

.PHONY: all clean $(PROG_NAME) compile

all: $(PROG_NAME)

compile:
	$(exe_file): $(OBJECTS)
		$(CC) $^ -o $(exe_file)
	%.o: %.$(SRCEXT) %.$(HEADEREXT)
		$(CC) $(CFLAGS) -c -o $@ $<

$(PROG_NAME): compile
	$(LD) $(OBJ_LIST) -o $(BIN_DIR)/$@

clean:
	rm -f $(BIN_DIR)/$(PROG_NAME) $(BUILD_DIR)/*.o
