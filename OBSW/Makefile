CC     = gcc
CFLAGS = -Wall -Wextra -Wno-unused-parameter -g -O0 -DSIMULATION_MODE
LIBS   = -lm

SRC_DIR  = obsw/src
TASK_DIR = $(SRC_DIR)/tasks
HAL_DIR  = $(SRC_DIR)/hal

SOURCES = $(SRC_DIR)/main.c \
          $(SRC_DIR)/hal/sim_state.c \
          $(SRC_DIR)/algorithms/quat.c \
          $(SRC_DIR)/algorithms/ekf.c \
          $(TASK_DIR)/watchdog_task.c \
          $(TASK_DIR)/adcs_controller.c \
          $(TASK_DIR)/fdir_monitor.c \
          $(TASK_DIR)/power_manager.c \
          $(TASK_DIR)/comms_manager.c \
          $(TASK_DIR)/payload_scheduler.c \
          $(TASK_DIR)/telemetry_task.c \
          $(TASK_DIR)/flash_manager.c \
          obsw/src/protocols/tc_dispatcher.c \
          obsw/src/protocols/tm_encoder.c

INCLUDES = -I$(HAL_DIR) -I$(SRC_DIR)
TARGET   = slars_obsw_sim

all: $(TARGET)
	@echo ""
	@echo "  Build complete: ./$(TARGET)"
	@echo ""

$(TARGET): $(SOURCES)
	@echo "Building SLARS-BCG-1 OBSW..."
	$(CC) $(CFLAGS) $(INCLUDES) $(SOURCES) $(LIBS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all run clean
