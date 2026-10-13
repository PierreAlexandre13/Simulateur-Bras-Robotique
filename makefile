#  Makefile — Bras robotique
#
#  Cibles disponibles :
#  make            :  compile tout (main + tests)
#  make main       :  compile uniquement le main
#  make tests      :  compile les deux exécutables de test
#  make test_bras  :  compile uniquement test_bras
#  make test_joints:  compile uniquement test_joints
#  make run        :  exécute le main
#  make run_tests  :  exécute les deux tests
#  make clean      :  supprime les binaires


CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra

#PKG_FLAGS := $(shell pkg-config --cflags --libs pinocchio eigen3)
PKG_FLAGS := $(shell pkg-config --cflags pinocchio eigen3 | sed 's/-I/-isystem /g')
#PKG_FLAGS_URDF := $(shell pkg-config --cflags --libs pinocchio eigen3) -lpinocchio_parsers
PKG_FLAGS_URDF := $(shell pkg-config --cflags pinocchio eigen3 | sed 's/-I/-isystem /g') $(shell pkg-config --cflags --libs pinocchio eigen3) -lpinocchio_parsers

INC_DIR   := include
SRC_DIR   := src
TEST_DIR  := tests
BIN_DIR   := bin

INCLUDES  := -I$(INC_DIR)
GTEST     := -lgtest -lgtest_main -pthread

MAIN_BIN       := $(BIN_DIR)/main
TEST_BRAS_BIN  := $(BIN_DIR)/test_bras
TEST_JOINTS_BIN:= $(BIN_DIR)/test_joints


.PHONY: all
all: $(BIN_DIR) main tests

# Crée le dossier bin s'il n'existe pas
$(BIN_DIR):
	mkdir -p $(BIN_DIR)


.PHONY: main
main: $(MAIN_BIN)

$(MAIN_BIN): $(SRC_DIR)/main.cpp $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< $(PKG_FLAGS_URDF) -o $@
	@echo "main compilé : $@"


.PHONY: tests
tests: test_bras test_joints

.PHONY: test_bras
test_bras: $(TEST_BRAS_BIN)

$(TEST_BRAS_BIN): $(TEST_DIR)/test_bras.cpp $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< $(PKG_FLAGS) $(GTEST) -o $@
	@echo "test_bras compilé  : $@"

.PHONY: test_joints
test_joints: $(TEST_JOINTS_BIN)

$(TEST_JOINTS_BIN): $(TEST_DIR)/test_joints.cpp $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< $(PKG_FLAGS) $(GTEST) -o $@
	@echo "test_joints compilé  :  $@"


.PHONY: run
run: $(MAIN_BIN)
	@echo "\n==== Exécution de main ===="
	./$(MAIN_BIN)

.PHONY: run_tests
run_tests: $(TEST_BRAS_BIN) $(TEST_JOINTS_BIN)
	@echo "\n==== test_joints ===="
	./$(TEST_JOINTS_BIN)
	@echo "\n==== test_bras ===="
	./$(TEST_BRAS_BIN)


.PHONY: clean
clean:
	rm -rf $(BIN_DIR)
	@echo "binaires supprimés"