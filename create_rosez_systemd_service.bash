#!/bin/bash
sudo_service_location=$(systemctl show sudo.service -P FragmentPath)
if [ -z "$sudo_service_location" ]; then
    systemd_system_service_dir=$(systemctl show default.target -P FragmentPath | xargs dirname)
else
    systemd_system_service_dir=$(dirname $sudo_service_location)
fi
if [ "$#" -gt 0 ]; then
    if [ "$#" -ne 5 ]; then
        printf "5 arguments are needed.\n\
        1st arg: rosez version.\n\
        2nd arg: rosez flags ('ni' and 'ns' are enabled by default and can't be disabled. Also 'fi' is recommended).\n\
        3rd arg: The ros command.\n\
        4th arg: Systemd restart field.\n\
        5th arg: Systemd service filename (without suffix).\n\
        Fields that are not required should be passed an empty string (\"\")\n\n
        Example 'roscore' command for ROS Melodic:\n\
        create_rosez_systemd_service.bash \"rosezm\" \"cl\" \"roscore\" \"always\" \"melodic_roscore\"
        "
        exit
    else
        rosez_version=$1
        rosez_flags=$2
        command_input=$3
        restart_field=$4
        service_filename=$5
        if [ -z "$command_input" ]; then
            printf "The ROS command cannot be empty. Exiting..."
            exit
        fi
        if [ -z "$service_filename" ]; then
            printf "The systemd service filename cannot be empty. Exiting..."
            exit
        fi
        service_filename="$service_filename.service"
        exec_start_field="ExecStart=$(command -v $rosez_version) $rosez_flags ni ns $command_input"
        if [ -n "$restart_field" ]; then
            restart_field="Restart=$restart_field"
        fi
    fi
else
    if ! type "dialog" > /dev/null 2>&1; then
        echo "'dialog' (https://linux.die.net/man/1/dialog) is not installed, please install it and run the script again!"
        exit -1
    fi
    SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
    source $SCRIPT_DIR/internal/helpers.bash
    get_supported_versions
    # Welcome screen
    run_dialog_command "--msgbox" "Welcome to the rosez systemd service creator!" 
    # rosez version
    for i in $(seq 0 $(( ${#distros[@]}-1 )) ); do
        choices="$choices\"${rosezv[$i]}\" \"${distros[$i]}\" \"on\" "
    done
    run_dialog_command "--radiolist" "Select a rosez version" ${#distros[@]} "$choices"
    selected_version=$choice
    # rosez flags
    choices=""
    for i in $(seq 0 $(( ${#known_params[@]}-1 )) ); do
        choices="$choices\"${known_params[$i]}\" \"${known_params_short[$i]}\" \"on\" "
    done
    run_dialog_command "--checklist" "Select rosez flags\n(non-interactive and no-sound are currently mandatory)" ${#known_params[@]} "$choices"
    selected_flags=$choice
    # ros command
    choice=""
    while [[ -z "${choice// }" ]]; do
        run_dialog_command "--inputbox" "Enter the command you need to run on boot" 0 ""
        command_input=$(printf '%q ' $choice)
    done
    IFS=$'\n' read -r -d '' -a systemd_targets < <(echo 'Skip' && systemctl show '*' --state=active --property=Id --value --no-pager | grep -E '+.target|+.service' | sort )
    choices=""
    for i in $(seq 0 $(( ${#systemd_targets[@]}-1 )) ); do
        choices="$choices\"${systemd_targets[$i]}\" \"\" \"on\" "
    done
    # service exec start field
    exec_start_field="ExecStart=$(command -v $selected_version) $selected_flags $command_input"
    # service restart field
    restart_options=("always" "on-success" "on-failure" "on-abnormal" "on-watchdog" "on-abort" "no")
    choices=""
    for i in $(seq 0 $(( ${#restart_options[@]}-1 )) ); do
        choices="$choices\"${restart_options[$i]}\" \"\" \"on\" "
    done
    run_dialog_command "--radiolist" "Select a [Restart] strategy" ${#restart_options[@]} "$choices"
    selected_restart=$choice
    restart_field="Restart=$selected_restart"
fi
systemd_service="\
[Unit]\n\
Description=Auto-generated service file for rosez automation with command $command_input.\n\
After=multi-user.target\n
Requires=network.target\n
\n\
[Service]\n\
Type=simple\n\
User=$(whoami)\n\
Group=$(id -gn)\n\
WorkingDirectory=$HOME\n\
$exec_start_field\n\
$restart_field\n\
\n\
[Install]\n\
WantedBy=default.target\n
"
if [ "$#" -eq 0 ]; then
    # Confirmation
    run_dialog_command "--yesno" "Does the service look correct? ('No' will quit):\n$systemd_service" 
    # Installation process
    run_dialog_command "--inputbox" "Enter the name of your service file. Do not incluce a suffix or any file-breaking characters. i.e. If you want to create a service file called 'rosez_roscore.service', simply input 'rosez_roscore'." 0 ""
    service_filename=$choice.service
    run_dialog_command "--msgbox" "The script will now exit and you will be prompted to enter your sudo password in order to install $service_filename in $systemd_system_service_dir..."
    clear
fi
printf "$systemd_service"
printf "$systemd_service" | sudo tee $systemd_system_service_dir/$service_filename > /dev/null
sudo systemctl enable $service_filename
