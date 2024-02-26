#!/bin/bash

# Exit if any statement fails
set -e

COLOR_RESET='\033[0m'       # Text Reset
YELLOW='\033[0;33m'         # Yellow

printf "${YELLOW}Flashing CF E7${COLOR_RESET}\n"
CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7E7" make cload
printf "${YELLOW}Flashing CF E8${COLOR_RESET}\n"
CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7E8" make cload
printf "${YELLOW}Flashing CF E9${COLOR_RESET}\n"
CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7E9" make cload
printf "${YELLOW}Flashing CF EA${COLOR_RESET}\n"
CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7EA" make cload
