CC ?= cc
CXX ?= c++
CFLAGS := -std=c11 -Wall -Wextra -Werror -pedantic -Isrc
CXXFLAGS := -std=c++17 -Wall -Wextra -Werror -pedantic -Isrc
LDLIBS := -lm
BUILD_DIR := tests/build
LIB_OBJECT := $(BUILD_DIR)/kalman_filter.o

.PHONY: all test clean

all: test

test: $(BUILD_DIR)/test_kalman_filter $(BUILD_DIR)/test_cpp_linkage
	./$(BUILD_DIR)/test_kalman_filter
	./$(BUILD_DIR)/test_cpp_linkage

$(BUILD_DIR):
	mkdir -p $@

$(LIB_OBJECT): src/kalman_filter.c src/kalman_filter.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/test_kalman_filter: tests/test_kalman_filter.c $(LIB_OBJECT)
	$(CC) $(CFLAGS) $^ $(LDLIBS) -o $@

$(BUILD_DIR)/test_cpp_linkage: tests/test_cpp_linkage.cpp $(LIB_OBJECT)
	$(CXX) $(CXXFLAGS) $^ $(LDLIBS) -o $@

clean:
	rm -rf $(BUILD_DIR)
