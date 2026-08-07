CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -pedantic
CXX ?= c++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -Werror -pedantic
LDLIBS ?= -lm

TARGET := test_complementary_filter
CPP_TARGET := test_cpp_linkage
CORE_OBJECT := complementary_filter.o
CXX_AVAILABLE := $(shell command -v $(firstword $(CXX)) 2>/dev/null)

.PHONY: all test cpp-smoke clean

all: test

test: $(TARGET) cpp-smoke
	./$(TARGET)

$(CORE_OBJECT): ../src/complementary_filter.c ../src/complementary_filter.h
	$(CC) $(CFLAGS) -I../src -c ../src/complementary_filter.c -o $@

$(TARGET): test_complementary_filter.c $(CORE_OBJECT)
	$(CC) $(CFLAGS) -I../src test_complementary_filter.c $(CORE_OBJECT) \
		$(LDLIBS) -o $(TARGET)

ifneq ($(CXX_AVAILABLE),)
cpp-smoke: $(CPP_TARGET)
	./$(CPP_TARGET)

$(CPP_TARGET): test_cpp_linkage.cpp $(CORE_OBJECT)
	$(CXX) $(CXXFLAGS) -I../src test_cpp_linkage.cpp $(CORE_OBJECT) \
		$(LDLIBS) -o $(CPP_TARGET)
else
cpp-smoke:
	@echo "SKIP: C++ compiler not available"
endif

clean:
	rm -f $(TARGET) $(CPP_TARGET) $(CORE_OBJECT)
