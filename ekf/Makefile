CC       ?= cc
CXX      ?= c++
AR       ?= ar
CFLAGS   ?= -std=c99 -O2 -Wall -Wextra -Werror -pedantic
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -Werror -pedantic
CPPFLAGS += -Iinclude
LDLIBS   = -lm

LIB = libekf.a
OBJ = src/ekf_filter.o

CXX_AVAILABLE := $(shell command -v $(firstword $(CXX)) 2>/dev/null)

all: $(LIB) pendulum heading test_ekf_filter

$(LIB): $(OBJ)
	$(AR) rcs $@ $^

src/ekf_filter.o: src/ekf_filter.c include/ekf_filter.h
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

pendulum: examples/pendulum.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

heading: examples/heading.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

test_ekf_filter: test/test_ekf_filter.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

check: test_ekf_filter cpp-smoke
	./test_ekf_filter

ifneq ($(CXX_AVAILABLE),)
cpp-smoke: test_cpp_linkage
	./test_cpp_linkage

test_cpp_linkage: test/test_cpp_linkage.cpp $(LIB)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@
else
cpp-smoke:
	@echo "SKIP: no C++ compiler"
endif

run: pendulum heading
	./pendulum
	./heading

clean:
	rm -f $(OBJ) $(LIB) pendulum heading test_ekf_filter test_cpp_linkage

.PHONY: all check cpp-smoke run clean
