#!/bin/bash
# Export everything in this file
set -a

known_params=("force-integrated" "clear-locks" "skip-compilation" "non-interactive" "no-sound")
known_params_short=("fi" "cl" "sc" "ni" "ns")
signal_list=(INT QUIT TSTP TERM HUP KILL)
colour_end='\033[0m'
colour_red='\033[0;31m'
colour_green='\033[0;32m'
colour_orange='\033[0;33m'
colour_blue='\033[0;34m'
lock_prefix=".rosez-"
lock_suffix=".lock"

rosezv[0]=""
workspaces[0]=""
volumes[0]=""
distros[0]=""
image_names[0]=""
executable_folder_names[0]=""
dockerfiles[0]=""

title="rosez"
dialog_command="dialog --title \"$title\" --clear"
# Function that simpifies the process of creating dialog "screens"
function run_dialog_command {
        tmpfile=`tempfile 2>/dev/null` || tmpfile=/tmp/test$$
        trap "rm -f $tmpfile" $signal_list
        if [ -n "$3" ] && [ $3 -eq 0 ]; then
            eval "$dialog_command $1 \"$2\" 20 61 \"$4\" 2>$tmpfile"

        else
            eval "$dialog_command $1 \"$2\" 20 60 $3 $4 2>$tmpfile"
        fi
        retval=$?
        if [ $retval -eq 0 ]; then
            choice=$(cat $tmpfile)
        else
            clear
            exit 0
        fi
    }
# This function read a txt file with all the supported ros versions along with their configuration
function get_supported_versions {
    helper_script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
    supported_versions_file=$helper_script_dir/supported_versions.txt
    vnum=0
    while read -r line; do
        while IFS=',' read -ra line_arr; do
            if [ $vnum -gt 0 ]; then
                index=$(( $vnum-1 ))
                rosezv[$index]="${line_arr[0]}"
                workspaces[$index]="${line_arr[1]}"
                volumes[$index]="${line_arr[2]}"
                distros[$index]="${line_arr[3]}"
                image_names[$index]="${line_arr[4]}"
                executable_folder_names[$index]="${line_arr[5]}"
                dockerfiles[$index]="${line_arr[6]}"
            fi
            vnum=$(( $vnum+1 ))
        done <<< $line
    done < $supported_versions_file
}

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
