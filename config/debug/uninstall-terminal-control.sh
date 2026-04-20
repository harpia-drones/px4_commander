#!/usr/bin/env bash

set -e
set -o pipefail


# -------------------------------------
#   VARIABLES
# -------------------------------------

HOME_PATH="$(echo $HOME)"
CONFIG_BASH_DIR_PATH="${HOME_PATH}/.config/bash"
FUNCTIONS_FILE_PATH="${CONFIG_BASH_DIR_PATH}/px4-commander-functions.sh"
MARKER_INIT="# >>> PX4 COMMANDER FUNCTION >>>"
MARKER_END="# <<< PX4 COMMANDER FUNCTION <<<"


# -------------------------------------
#   COLORS
# -------------------------------------

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


# -------------------------------------
#   UNINSTALL TERMINAL CONTROL FUNCTION
# -------------------------------------

if [ ! -d "${CONFIG_BASH_DIR_PATH}" ]; then
    exit 0
fi

if [ -f "${FUNCTIONS_FILE_PATH}" ]; then
    rm -rf "${FUNCTIONS_FILE_PATH}"
fi

# Remove terminal control function from .bashrc
if grep -qF "$MARKER_INIT" "${HOME_PATH}/.bashrc"; then
	sed -i "/${MARKER_INIT}/,/${MARKER_INIT}/d" "${HOME_PATH}/.bashrc"
	echo -e "${GREEN}Funções de controle px4-commander removida.${NC}"
	echo -e "${RED}Execute \"source ${HOME_PATH}/.bashrc\" para validar as alterações.${NC}"
	echo -e "${RED}Execute \"unset -f arm disarm offboard land 2>/dev/null || true\" remover a função da sessão atual.${NC}"
else
	echo -e "${YELLOW}Funções de controle px4-commander já removida.${NC}"
fi