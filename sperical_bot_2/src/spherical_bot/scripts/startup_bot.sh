#!/bin/bash
#
# Spherical Bot - Startup Script
# Starts the spherical bot system with proper priorities and configuration
#

set -e  # Exit on any error

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Configuration
BOT_NAME="spherical_bot_001"
LOG_DIR="$HOME/spherical_bot_data/logs"
CONFIG_DIR="$PROJECT_ROOT/config"
LAUNCH_FILE="spherical_bot.launch.py"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_color() {
    echo -e "${2}${1}${NC}"
}

# Function to log with timestamp
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

# Function to check if ROS 2 is available
check_ros2() {
    if ! command -v ros2 >/dev/null 2>&1; then
        print_color "ERROR: ROS 2 not found in PATH" "$RED"
        return 1
    fi
    
    if [ -z "$ROS_DISTRO" ]; then
        print_color "ERROR: ROS_DISTRO not set" "$RED"
        return 1
    fi
    
    print_color "ROS 2 $ROS_DISTRO detected" "$GREEN"
    return 0
}

# Function to check if workspace is built
check_workspace() {
    if [ ! -f "$PROJECT_ROOT/../../install/setup.bash" ]; then
        print_color "ERROR: Workspace not built. Run 'colcon build' first." "$RED"
        return 1
    fi
    print_color "Workspace is built" "$GREEN"
    return 0
}

# Function to set real-time priorities
set_realtime_priorities() {
    log "Setting real-time priorities..."
    
    # Set CPU affinity and nice values
    sudo chrt -f -p 99 $$ 2>/dev/null || true
    
    # Set CPU performance governor
    if [ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]; then
        echo "performance" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null 2>&1 || true
    fi
    
    print_color "Real-time priorities configured" "$GREEN"
}

# Function to check system resources
check_system_resources() {
    log "Checking system resources..."
    
    # Check memory
    MEMORY_FREE=$(free -m | awk 'NR==2{printf "%.1f%%", $4*100/$2}')
    print_color "Memory free: $MEMORY_FREE" "$BLUE"
    
    # Check disk space
    DISK_FREE=$(df -h / | awk 'NR==2{print $4}')
    print_color "Disk free: $DISK_FREE" "$BLUE"
    
    # Check temperature
    if [ -f /sys/class/thermal/thermal_zone0/temp ]; then
        TEMP=$(cat /sys/class/thermal/thermal_zone0/temp)
        TEMP_C=$((TEMP/1000))
        print_color "CPU temperature: ${TEMP_C}°C" "$BLUE"
        
        if [ $TEMP_C -gt 70 ]; then
            print_color "WARNING: High temperature detected" "$YELLOW"
        fi
    fi
    
    # Check battery voltage (if available)
    if [ -f /sys/class/power_supply/battery/voltage_now ]; then
        VOLTAGE=$(cat /sys/class/power_supply/battery/voltage_now)
        VOLTAGE_V=$(echo "scale=2; $VOLTAGE/1000000" | bc)
        print_color "Battery voltage: ${VOLTAGE_V}V" "$BLUE"
    fi
}

# Function to start the bot
start_bot() {
    local LAUNCH_MODE="$1"
    
    log "Starting Spherical Bot in $LAUNCH_MODE mode..."
    
    # Source ROS 2 and workspace
    source /opt/ros/$ROS_DISTRO/setup.bash
    source "$PROJECT_ROOT/../../install/setup.bash"
    
    # Create log directory
    mkdir -p "$LOG_DIR"
    
    # Generate log filename with timestamp
    TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
    LOG_FILE="$LOG_DIR/bot_${TIMESTAMP}.log"
    
    print_color "Starting bot - logs: $LOG_FILE" "$GREEN"
    
    # Set ROS log level
    export RCUTILS_LOGGING_SEVERITY=INFO
    
    # Launch the bot
    case "$LAUNCH_MODE" in
        "normal")
            ros2 launch spherical_bot "$LAUNCH_FILE" > "$LOG_FILE" 2>&1 &
            LAUNCH_PID=$!
            ;;
        "emergency")
            ros2 launch spherical_bot emergency_mode.launch.py >> "$LOG_FILE" 2>&1 &
            LAUNCH_PID=$!
            ;;
        "debug")
            export RCUTILS_LOGGING_SEVERITY=DEBUG
            ros2 launch spherical_bot "$LAUNCH_FILE" debug_mode:=true >> "$LOG_FILE" 2>&1 &
            LAUNCH_PID=$!
            ;;
        *)
            print_color "Unknown launch mode: $LAUNCH_MODE" "$RED"
            exit 1
            ;;
    esac
    
    echo $LAUNCH_PID > "/tmp/spherical_bot_pid"
    print_color "Bot started with PID: $LAUNCH_PID" "$GREEN"
    
    # Monitor the process
    monitor_bot "$LAUNCH_PID"
}

# Function to monitor the bot process
monitor_bot() {
    local PID=$1
    
    log "Monitoring bot process (PID: $PID)..."
    
    while kill -0 $PID 2>/dev/null; do
        sleep 5
        
        # Check system resources every 30 seconds
        if [ $((SECONDS % 30)) -eq 0 ]; then
            check_system_resources
        fi
    done
    
    print_color "Bot process stopped" "$YELLOW"
    
    # Check if it was a clean exit
    wait $PID
    EXIT_CODE=$?
    
    if [ $EXIT_CODE -eq 0 ]; then
        print_color "Bot stopped cleanly" "$GREEN"
    else
        print_color "Bot crashed with exit code: $EXIT_CODE" "$RED"
        print_color "Check logs: $LOG_FILE" "$YELLOW"
        
        # Attempt to restart if not manual stop
        if [ ! -f "/tmp/spherical_bot_stop" ]; then
            print_color "Attempting to restart in 5 seconds..." "$YELLOW"
            sleep 5
            start_bot "normal"
        fi
    fi
}

# Function to stop the bot
stop_bot() {
    log "Stopping Spherical Bot..."
    
    # Create stop flag
    touch "/tmp/spherical_bot_stop"
    
    if [ -f "/tmp/spherical_bot_pid" ]; then
        BOT_PID=$(cat "/tmp/spherical_bot_pid")
        
        if kill -0 $BOT_PID 2>/dev/null; then
            print_color "Stopping bot process (PID: $BOT_PID)..." "$YELLOW"
            
            # Graceful shutdown
            kill -TERM $BOT_PID
            
            # Wait for process to end
            sleep 3
            
            # Force kill if still running
            if kill -0 $BOT_PID 2>/dev/null; then
                print_color "Force stopping bot..." "$RED"
                kill -KILL $BOT_PID
            fi
        fi
        
        rm -f "/tmp/spherical_bot_pid"
    fi
    
    rm -f "/tmp/spherical_bot_stop"
    print_color "Bot stopped" "$GREEN"
}

# Function to show status
show_status() {
    if [ -f "/tmp/spherical_bot_pid" ]; then
        BOT_PID=$(cat "/tmp/spherical_bot_pid")
        if kill -0 $BOT_PID 2>/dev/null; then
            print_color "Spherical Bot is RUNNING (PID: $BOT_PID)" "$GREEN"
            
            # Show recent log entries
            if [ -n "$(ls -At $LOG_DIR/*.log 2>/dev/null)" ]; then
                LATEST_LOG=$(ls -At $LOG_DIR/*.log | head -1)
                print_color "Recent log entries from $LATEST_LOG:" "$BLUE"
                tail -5 "$LATEST_LOG" | while read line; do
                    echo "  $line"
                done
            fi
        else
            print_color "Spherical Bot is NOT RUNNING (stale PID file)" "$RED"
            rm -f "/tmp/spherical_bot_pid"
        fi
    else
        print_color "Spherical Bot is NOT RUNNING" "$YELLOW"
    fi
    
    check_system_resources
}

# Main script
case "${1:-start}" in
    start)
        check_ros2 || exit 1
        check_workspace || exit 1
        set_realtime_priorities
        check_system_resources
        start_bot "normal"
        ;;
    start-debug)
        check_ros2 || exit 1
        check_workspace || exit 1
        set_realtime_priorities
        check_system_resources
        start_bot "debug"
        ;;
    start-emergency)
        check_ros2 || exit 1
        check_workspace || exit 1
        set_realtime_priorities
        check_system_resources
        start_bot "emergency"
        ;;
    stop)
        stop_bot
        ;;
    restart)
        stop_bot
        sleep 2
        check_ros2 || exit 1
        check_workspace || exit 1
        set_realtime_priorities
        start_bot "normal"
        ;;
    status)
        show_status
        ;;
    logs)
        if [ -n "$(ls -At $LOG_DIR/*.log 2>/dev/null)" ]; then
            LATEST_LOG=$(ls -At $LOG_DIR/*.log | head -1)
            less "$LATEST_LOG"
        else
            print_color "No log files found" "$YELLOW"
        fi
        ;;
    *)
        echo "Usage: $0 {start|start-debug|start-emergency|stop|restart|status|logs}"
        echo ""
        echo "Commands:"
        echo "  start           Start the bot in normal mode"
        echo "  start-debug     Start the bot in debug mode"
        echo "  start-emergency Start the bot in emergency mode"
        echo "  stop            Stop the bot"
        echo "  restart         Restart the bot"
        echo "  status          Show bot status and system info"
        echo "  logs            Show latest logs"
        exit 1
        ;;
esac