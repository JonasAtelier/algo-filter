CC       ?= cc
CXX      ?= c++
AR       ?= ar
CFLAGS   ?= -std=c99 -O2 -Wall -Wextra -Werror -pedantic
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -Werror -pedantic
CPPFLAGS += -Iinclude
LDLIBS   = -lm

LIB = libparticle.a
OBJ = src/particle_filter.o

CXX_AVAILABLE := $(shell command -v $(firstword $(CXX)) 2>/dev/null)

all: $(LIB) track test_particle_filter

$(LIB): $(OBJ)
	$(AR) rcs $@ $^

src/particle_filter.o: src/particle_filter.c include/particle_filter.h
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

track: examples/track.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

test_particle_filter: test/test_particle_filter.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

check: test_particle_filter cpp-smoke
	./test_particle_filter

ifneq ($(CXX_AVAILABLE),)
cpp-smoke: test_cpp_linkage
	./test_cpp_linkage

test_cpp_linkage: test/test_cpp_linkage.cpp $(LIB)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@
else
cpp-smoke:
	@echo "SKIP: no C++ compiler"
endif

run: track
	./track

clean:
	rm -f $(OBJ) $(LIB) track test_particle_filter test_cpp_linkage

.PHONY: all check cpp-smoke run clean
