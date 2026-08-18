CC       ?= cc
CXX      ?= c++
AR       ?= ar
CFLAGS   ?= -std=c99 -O2 -Wall -Wextra -Werror -pedantic
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -Werror -pedantic
CPPFLAGS += -Iinclude
LDLIBS   = -lm

LIB = libkalman.a
OBJ = src/kalman_filter.o

CXX_AVAILABLE := $(shell command -v $(firstword $(CXX)) 2>/dev/null)

all: $(LIB) basic custom_tuning test_kalman_filter

$(LIB): $(OBJ)
	$(AR) rcs $@ $^

src/kalman_filter.o: src/kalman_filter.c include/kalman_filter.h
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

basic: examples/basic.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

custom_tuning: examples/custom_tuning.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

test_kalman_filter: test/test_kalman_filter.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

check: test_kalman_filter cpp-smoke
	./test_kalman_filter

ifneq ($(CXX_AVAILABLE),)
cpp-smoke: test_cpp_linkage
	./test_cpp_linkage

test_cpp_linkage: test/test_cpp_linkage.cpp $(LIB)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@
else
cpp-smoke:
	@echo "SKIP: no C++ compiler"
endif

run: basic
	./basic

clean:
	rm -f $(OBJ) $(LIB) basic custom_tuning test_kalman_filter test_cpp_linkage

.PHONY: all check cpp-smoke run clean
