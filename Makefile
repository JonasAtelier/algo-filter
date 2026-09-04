CC       ?= cc
CXX      ?= c++
AR       ?= ar
CFLAGS   ?= -std=c99 -O2 -Wall -Wextra -Werror -pedantic
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -Werror -pedantic
CPPFLAGS += -Iinclude
LDLIBS   = -lm

LIB = librls.a
OBJ = src/rls_filter.o

CXX_AVAILABLE := $(shell command -v $(firstword $(CXX)) 2>/dev/null)

all: $(LIB) motor test_rls_filter

$(LIB): $(OBJ)
	$(AR) rcs $@ $^

src/rls_filter.o: src/rls_filter.c include/rls_filter.h
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

motor: examples/motor.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

test_rls_filter: test/test_rls_filter.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

check: test_rls_filter cpp-smoke
	./test_rls_filter

ifneq ($(CXX_AVAILABLE),)
cpp-smoke: test_cpp_linkage
	./test_cpp_linkage

test_cpp_linkage: test/test_cpp_linkage.cpp $(LIB)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@
else
cpp-smoke:
	@echo "SKIP: no C++ compiler"
endif

run: motor
	./motor

clean:
	rm -f $(OBJ) $(LIB) motor test_rls_filter test_cpp_linkage

.PHONY: all check cpp-smoke run clean
