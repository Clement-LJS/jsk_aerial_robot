#!/bin/bash

# ============================================================
# Usage:
#   ./bringup.sh sim:=true
#   ./bringup.sh sim:=false
#
# sim:=true  = simulation mode
# sim:=false = real machine mode
# ============================================================

SIMULATION=true
AIRFRAME="beetle"
SESSION_NAME="gimbalrotor_bringup"
WS_SETUP="$HOME/ros/jsk_aerial_robot_ws/devel/setup.bash"

for ARG in "$@"; do
    case $ARG in
        sim:=true)
            SIMULATION=true
            ;;
        sim:=false)
            SIMULATION=false
            ;;
        *)
            echo "Unknown argument: $ARG"
            echo ""
            echo "Use:"
            echo "  ./bringup.sh sim:=true"
            echo "  ./bringup.sh sim:=false"
            exit 1
            ;;
    esac
done

# Kill old tmux session with the same name if it already exists
tmux has-session -t "$SESSION_NAME" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "Old tmux session '$SESSION_NAME' already exists."
    echo "Killing old session..."
    tmux kill-session -t "$SESSION_NAME"
fi

echo "Creating tmux session: $SESSION_NAME"

# Create new tmux session
tmux new-session -d -s "$SESSION_NAME"

# Split into two vertical panes: left and right
tmux split-window -h -t "$SESSION_NAME"

# Resize panes evenly
tmux select-layout -t "$SESSION_NAME" even-horizontal

if [ "$SIMULATION" = "true" ]; then
    echo "Starting Gimbalrotor in SIMULATION mode..."

    BRINGUP_CMD="source $WS_SETUP && roslaunch gimbalrotor bringup.launch simulation:=true headless:=false real_machine:=false airframe:=$AIRFRAME"

else
    echo "Starting Gimbalrotor in REAL MACHINE mode..."

    BRINGUP_CMD="source $WS_SETUP && roslaunch gimbalrotor bringup.launch simulation:=false real_machine:=true airframe:=$AIRFRAME"
fi

KEYBOARD_CMD="source $WS_SETUP && sleep 5 && rosrun aerial_robot_base keyboard_command.py"

# Pane 0: roslaunch
tmux send-keys -t "$SESSION_NAME:0.0" "$BRINGUP_CMD" C-m

# Pane 1: keyboard command
tmux send-keys -t "$SESSION_NAME:0.1" "$KEYBOARD_CMD" C-m

# Attach to tmux session
tmux attach-session -t "$SESSION_NAME"