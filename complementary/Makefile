CC       ?= cc
CXX      ?= c++
AR       ?= ar
CFLAGS   ?= -std=c99 -O2 -Wall -Wextra -Werror -pedantic
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -Werror -pedantic
CPPFLAGS += -Iinclude
LDLIBS   = -lm

LIB = libcomplementary.a
OBJ = src/complementary_filter.o

CXX_AVAILABLE := $(shell command -v $(firstword $(CXX)) 2>/dev/null)

all: $(LIB) angle test_complementary_filter

$(LIB): $(OBJ)
	$(AR) rcs $@ $^

src/complementary_filter.o: src/complementary_filter.c include/complementary_filter.h
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

angle: examples/angle.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

test_complementary_filter: test/test_complementary_filter.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

check: test_complementary_filter cpp-smoke
	./test_complementary_filter

ifneq ($(CXX_AVAILABLE),)
cpp-smoke: test_cpp_linkage
	./test_cpp_linkage

test_cpp_linkage: test/test_cpp_linkage.cpp $(LIB)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@
else
cpp-smoke:
	@echo "SKIP: no C++ compiler"
endif

run: angle
	./angle

clean:
	rm -f $(OBJ) $(LIB) angle test_complementary_filter test_cpp_linkage

.PHONY: all check cpp-smoke run clean
