CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O3 -Wall -Wextra -pedantic -Iinclude
BIN_DIR  := bin
OBJ_DIR  := obj

SRCS_LIB := src/Motor.cpp src/Encoder.cpp src/PIDController.cpp src/SafetySystem.cpp src/MotorController.cpp
OBJS_LIB := $(patsubst src/%.cpp,$(OBJ_DIR)/%.o,$(SRCS_LIB))

TARGET_SIM   := $(BIN_DIR)/motor_sim.exe
TARGET_TESTS := $(BIN_DIR)/motor_tests.exe

.PHONY: all clean run test

all: $(TARGET_SIM) $(TARGET_TESTS)

$(OBJ_DIR)/%.o: src/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/%.o: tests/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TARGET_SIM): $(OBJS_LIB) $(OBJ_DIR)/main.o
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@

$(TARGET_TESTS): $(OBJS_LIB) $(OBJ_DIR)/test_main.o $(OBJ_DIR)/test_pid.o $(OBJ_DIR)/test_safety.o
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@

run: $(TARGET_SIM)
	./$(TARGET_SIM)

test: $(TARGET_TESTS)
	./$(TARGET_TESTS)

clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)
