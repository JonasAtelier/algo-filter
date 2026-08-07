CC ?= cc
CXX ?= c++
CFLAGS := -std=c11 -Wall -Wextra -Werror -pedantic -I../src
CXXFLAGS := -std=c++11 -Wall -Wextra -Werror -pedantic -I../src
LDLIBS := -lm

.PHONY: all test cpp-test clean

all: test cpp-test

test: test_particle_filter
	./test_particle_filter

test_particle_filter: test_particle_filter.c ../src/particle_filter.c \
		      ../src/particle_filter.h
	$(CC) $(CFLAGS) test_particle_filter.c ../src/particle_filter.c \
		-o $@ $(LDLIBS)

cpp-test: particle_filter.o
	@if command -v "$(CXX)" >/dev/null 2>&1; then \
		$(CXX) $(CXXFLAGS) test_cpp_linkage.cpp particle_filter.o \
			-o test_cpp_linkage $(LDLIBS); \
		./test_cpp_linkage; \
		echo "PASS: C++ linkage test"; \
	else \
		echo "SKIP: no C++ compiler found"; \
	fi

particle_filter.o: ../src/particle_filter.c ../src/particle_filter.h
	$(CC) $(CFLAGS) -c ../src/particle_filter.c -o $@

clean:
	rm -f particle_filter.o test_particle_filter test_cpp_linkage
