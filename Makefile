CC       ?= cc
CXX      ?= c++
AR       ?= ar
CFLAGS   ?= -std=c99 -O2 -Wall -Wextra -Werror -pedantic
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -Werror -pedantic
CPPFLAGS += -Iinclude
LDLIBS   = -lm

LIB = libmadgwick.a
OBJ = src/madgwick_filter.o

CXX_AVAILABLE := $(shell command -v $(firstword $(CXX)) 2>/dev/null)

all: $(LIB) tilt test_madgwick_filter

$(LIB): $(OBJ)
	$(AR) rcs $@ $^

src/madgwick_filter.o: src/madgwick_filter.c include/madgwick_filter.h
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

tilt: examples/tilt.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

test_madgwick_filter: test/test_madgwick_filter.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

check: test_madgwick_filter cpp-smoke
	./test_madgwick_filter

ifneq ($(CXX_AVAILABLE),)
cpp-smoke: test_cpp_linkage
	./test_cpp_linkage

test_cpp_linkage: test/test_cpp_linkage.cpp $(LIB)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@
else
cpp-smoke:
	@echo "SKIP: no C++ compiler"
endif

run: tilt
	./tilt

clean:
	rm -f $(OBJ) $(LIB) tilt test_madgwick_filter test_cpp_linkage

.PHONY: all check cpp-smoke run clean
