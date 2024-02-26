# Enable app support
APP=1
APP_STACKSIZE=3000

VPATH += src/
PROJ_OBJ += app_main.o

# Swarm logic
INCLUDES += -I../swarm-firmware/src -I../swarm-firmware/src/swarm -I../swarm-firmware/src/flash
VPATH += ../swarm-firmware/src/swarm
PROJ_OBJ += swarm.o
PROJ_OBJ += swarm_radio.o

CRAZYFLIE_BASE=../crazyflie-firmware
include $(CRAZYFLIE_BASE)/Makefile
