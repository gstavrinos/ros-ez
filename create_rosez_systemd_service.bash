#!/bin/bash
sudo_service_location=$(systemctl show sudo.service -P FragmentPath)
if [ -z "$sudo_service_location" ]; then
    systemd_system_service_dir=$(systemctl show default.target -P FragmentPath | xargs dirname)
else
    systemd_system_service_dir=$(dirname $sudo_service_location)
fi
if [ "$#" -gt 0 ]; then
    if [ "$#" -ne 9 ]; then
        printf "9 arguments are needed.\n\
        1st arg: rosez version.\n\
        2nd arg: Systemd before field.\n\
        3rd arg: Systemd after field.\n\
        4th arg: rosez flags ('ni' and 'ns' are enabled by default and can't be disabled. Also 'fi' is recommended).\n\
        5th arg: The ros command.\n\
        6th arg: Systemd restart field.\n\
        7th arg: Systemd wanted by field.\n\
        8th arg: Systemd required by field.\n\
        9th arg: Systemd service filename (without suffix).\n\
        Fields that are not required should be passed an empty string (\"\")\n\n
        Example 'roscore' command for ROS Melodic:\n\
        create_rosez_systemd_service.bash \"rosezm\" \"\" \"network.target\" \"cl\" \"roscore\" \"always\" \"graphical.target\" \"\" \"noetic_roscore\"
        "
        exit
    else
        rosez_version=$1
        before_field=$2
        after_field=$3
        rosez_flags=$4
        command_input=$5
        restart_field=$6
        wanted_by_field=$7
        required_by_field=$8
        service_filename=$9
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
        if [ -n "$before_field" ]; then
            before_field="Before=$before_field"
        fi
        if [ -n "$after_field" ]; then
            after_field="After=$after_field"
        fi
        if [ -n "$restart_field" ]; then
            restart_field="Restart=$restart_field"
        fi
        if [ -n "$wanted_by_field" ]; then
            wanted_by_field="WantedBy=$wanted_by_field"
        fi
        if [ -n "$required_by_field" ]; then
            required_by_field="RequiredBy=$required_by_field"
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
    # unit before field
    before_field=""
    run_dialog_command "--radiolist" "Select a systemd target for [Unit] Before" ${#systemd_targets[@]} "$choices"
    selected_before=$choice
    if [[ "$selected_before" != "Skip" ]]; then
        before_field="Before=$selected_before"
    fi
    # unit after field
    after_field=""
    run_dialog_command "--radiolist" "Select a systemd target for [Unit] After" ${#systemd_targets[@]} "$choices"
    selected_after=$choice
    if [[ "$selected_after" != "Skip" ]]; then
        after_field="After=$selected_after"
    fi
    # install wanted by field
    wanted_by_field=""
    run_dialog_command "--radiolist" "Select a systemd target for [Install] WantedBy" ${#systemd_targets[@]} "$choices"
    selected_wanted_by=$choice
    if [[ "$selected_wanted_by" != "Skip" ]]; then
        wanted_by_field="WantedBy=$selected_wanted_by"
    fi
    # install required by field
    required_by_field=""
    run_dialog_command "--radiolist" "Select a systemd target for [Install] RequiredBy" ${#systemd_targets[@]} "$choices"
    selected_required_by=$choice
    if [[ "$selected_required_by" != "Skip" ]]; then
        required_by_field="RequiredBy=$selected_required_by"
    fi
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
$before_field\n\
$after_field\n\
\n\
[Service]\n\
Type=simple\n\
User=$(whoami)\n\
Group=$(id -gn)\n\
WorkingDirectory=$HOME\n\
# Environment=\"DISPLAY=$DISPLAY\"\n\
# Environment=\"XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR\"\n\
$exec_start_field\n\
$restart_field\n\
\n\
[Install]\n\
$wanted_by_field\n\
$required_by_field\n\
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
