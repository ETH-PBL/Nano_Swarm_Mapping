# Enable app support
APP=1
APP_STACKSIZE=8000

INCLUDES += -Isrc
VPATH += src/
PROJ_OBJ += app_main.o
PROJ_OBJ += exploration.o
PROJ_OBJ += scan.o
PROJ_OBJ += slam.o
PROJ_OBJ += util.o

# ToF sensors
INCLUDES += -Isrc/tof-matrix -Isrc/tof-matrix/drivers
VPATH += src/tof-matrix
VPATH += src/tof-matrix/drivers
PROJ_OBJ += tof_matrix.o
PROJ_OBJ += i2c_expander.o
PROJ_OBJ += i2c_platform.o
PROJ_OBJ += vl53l5cx_api.o

# ICP
INCLUDES += -Isrc/icp
VPATH += src/icp
PROJ_OBJ += icp.o

# LS-SLAM
INCLUDES += -Isrc/ls-slam
VPATH += src/ls-slam
PROJ_OBJ += circular-queue.o
PROJ_OBJ += graph-based-slam.o
PROJ_OBJ += heapsort.o
PROJ_OBJ += rcm-sparse-matrix.o
PROJ_OBJ += sparse-matrix.o
PROJ_OBJ += utils_math.o

# SPI flash storage driver
INCLUDES += -Isrc/flash
VPATH += ../spiflash_driver/src
PROJ_OBJ += spiflash.o
VPATH += src/flash
PROJ_OBJ += flash.o

# Swarm logic
INCLUDES += -Isrc/swarm
VPATH += src/swarm
PROJ_OBJ += swarm.o
PROJ_OBJ += swarm_radio.o
PROJ_OBJ += swarm_comm.o
PROJ_OBJ += swarm_graph.o

CRAZYFLIE_BASE=../crazyflie-firmware
include $(CRAZYFLIE_BASE)/Makefile

icp_test: src/icp/icp_test.c src/icp/icp.c
	clang -o icp_test $^
