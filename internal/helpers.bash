#!/bin/bash
# Export everything in this file
set -a

signal_list=(INT QUIT TSTP TERM HUP KILL)
colour_end='\033[0m'
colour_red='\033[0;31m'
colour_green='\033[0;32m'
colour_orange='\033[0;33m'
colour_blue='\033[0;34m'
lock_prefix=".rosez-"
lock_suffix=".lock"

# This function is used to trap signals
function signal_handler() {
    echo -e "${colour_red}Execution was aborted, deleting my lock_file ($lock_file)$colour_end"
    rm -f $lockation/$lock_file
    exit
}
# This function is used to check intermediate commands'
# exit codes and then terminate if there was an error
# This is a required behaviour, in order
# to enter the shell (not exit) when a build fails.
function intermediate_error_handler() {
    if [ $1 -gt 0 ]; then
        echo -e "${colour_red}Intermediate process aborted!$colour_end"
        signal_handler
    fi
}
