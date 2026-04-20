#!/usr/bin/env bash

set -e
set -o pipefail


# -----------------------------------
#   VARIABLES
# -----------------------------------

HOME_PATH="$(echo $HOME)"
CONFIG_BASH_DIR_PATH="${HOME_PATH}/.config/bash"
FUNCTIONS_FILE_PATH="${CONFIG_BASH_DIR_PATH}/px4-commander-functions.sh"
MARKER_INIT="# >>> PX4 COMMANDER FUNCTION >>>"
MARKER_END="# <<< PX4 COMMANDER FUNCTION <<<"


# -----------------------------------
#   COLORS
# -----------------------------------

if [[ -t 1 ]] && command -v tput >/dev/null 2>&1 && tput colors >/dev/null 2>&1; then
  RED='\033[31;1m'
  YELLOW='\033[33;1m'
  YELLOW_BG='\033[33;7m'
  GREEN='\033[92;1m'
  NC='\033[0m'

else
  RED=''
  YELLOW=''
  YELLOW_BG=''
  GREEN=''
  NC=''
fi


# -----------------------------------
#   INSTALL TERMINAL CONTROL FUNCTION
# -----------------------------------

if [ ! -d "${CONFIG_BASH_DIR_PATH}" ]; then
    mkdir -p "${CONFIG_BASH_DIR_PATH}"
fi

if [ ! -f "${FUNCTIONS_FILE_PATH}" ]; then
    touch "${FUNCTIONS_FILE_PATH}"
fi

cat << EOF >> "${FUNCTIONS_FILE_PATH}"
arm() {
    ros2 service call /px4_commander/arm std_srvs/srv/SetBool "data: true"  
}

disarm() {
    ros2 service call /px4_commander/disarm std_srvs/srv/SetBool "data: true"  
}

offboard() {
    ros2 service call /px4_commander/offboard std_srvs/srv/SetBool "data: true"  
}

land() {
    ros2 service call /px4_commander/land std_srvs/srv/SetBool "data: true"  
}
EOF

# Install terminal control function into the system
if ! grep -q "$MARKER_INIT" "${HOME_PATH}/.bashrc"; then
cat << EOF >> "${HOME_PATH}/.bashrc"
${MARKER_INIT}
if [ -f "${FUNCTIONS_FILE_PATH}" ]; then
    . "${FUNCTIONS_FILE_PATH}"
fi
${MARKER_END}
EOF
  echo -e "${GREEN}Funções de controle px4-commander disponíveis.${NC}"
  echo -e "${RED}Execute \"source ${HOME_PATH}/.bashrc\" para validar as alterações.${NC}"
else
  echo -e "${YELLOW}Funções de controle px4-commander já instaladas. Use o script uninstall.sh para remover.${NC}"
fi
