#!/bin/bash
#
# Spherical Bot - Thermal Monitor Script
# Monitors system temperature and takes action to prevent overheating
#

set -e

# Configuration
CHECK_INTERVAL=5  # seconds
LOG_DIR="$HOME/spherical_bot_data/logs"
STATUS_FILE="/tmp/thermal_status"

# Temperature thresholds (Celsius)
NORMAL_TEMP=65
WARNING_TEMP=75
CRITICAL_TEMP=80
EMERGENCY_TEMP=85

# Cooling control
FAN_GPIO=19
FAN_ENABLED=false

# Colors
RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

# Function to initialize GPIO
init_gpio() {
    if [ ! -d "/sys/class/gpio/gpio$FAN_GPIO" ]; then
        echo "$FAN_GPIO" | sudo tee /sys/class/gpio/export >/dev/null 2>&1 || true
        sleep 1
    fi
    echo "out" | sudo tee /sys/class/gpio/gpio$FAN_GPIO/direction >/dev/null 2>&1 || true
}

# Function to control fan
set_fan() {
    local state=$1
    if [ "$state" = "on" ]; then
        echo "1" | sudo tee /sys/class/gpio/gpio$FAN_GPIO/value >/dev/null 2>&1 || true
        FAN_ENABLED=true
    else
        echo "0" | sudo tee /sys/class/gpio/gpio$FAN_GPIO/value >/dev/null 2>&1 || true
        FAN_ENABLED=false
    fi
}

# Function to read temperature
read_temperature() {
    if [ -f "/sys/class/thermal/thermal_zone0/temp" ]; then
        local temp_raw=$(cat /sys/class/thermal/thermal_zone0/temp)
        echo $((temp_raw / 1000))
    else
        echo "45"  # Default safe temperature
    fi
}

# Function to log message
log_message() {
    local message=$1
    local color=$2
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    
    echo -e "${color}[$timestamp] $message${NC}"
    echo "[$timestamp] $message" >> "$LOG_DIR/thermal_monitor.log"
}

# Function to update status file
update_status() {
    local status=$1
    local temp=$2
    
    cat > "$STATUS_FILE" << EOF
{
    "timestamp": "$(date -Iseconds)",
    "temperature": $temp,
    "status": "$status",
    "fan_enabled": $FAN_ENABLED,
    "thresholds": {
        "normal": $NORMAL_TEMP,
        "warning": $WARNING_TEMP,
        "critical": $CRITICAL_TEMP,
        "emergency": $EMERGENCY_TEMP
    }
}
EOF
}

# Function to check and handle temperature
check_temperature() {
    local temp=$(read_temperature)
    local previous_status=$(cat "$STATUS_FILE" 2>/dev/null | grep -o '"status":"[^"]*"' | cut -d'"' -f4 || echo "unknown")
    
    if [ $temp -ge $EMERGENCY_TEMP ]; then
        # EMERGENCY - Critical overheating
        if [ "$previous_status" != "emergency" ]; then
            log_message "🚨 EMERGENCY: Temperature $temp°C - Taking emergency actions" "$RED"
            set_fan on
            
            # Send emergency signal to bot nodes
            pkill -SIGUSR2 spherical_bot 2>/dev/null || true
            
            # Reduce system load
            echo "performance" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null 2>&1 || true
            echo "1" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_max_freq >/dev/null 2>&1 || true
        fi
        update_status "emergency" $temp
        
    elif [ $temp -ge $CRITICAL_TEMP ]; then
        # CRITICAL - Very high temperature
        if [ "$previous_status" != "critical" ]; then
            log_message "⚠️ CRITICAL: Temperature $temp°C - Reducing system load" "$RED"
            set_fan on
            
            # Send warning to bot nodes
            pkill -SIGUSR1 spherical_bot 2>/dev/null || true
            
            # Throttle CPU
            echo "800000" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_max_freq >/dev/null 2>&1 || true
        fi
        update_status "critical" $temp
        
    elif [ $temp -ge $WARNING_TEMP ]; then
        # WARNING - High temperature
        if [ "$previous_status" != "warning" ]; then
            log_message "🔶 WARNING: Temperature $temp°C - Enabling cooling" "$YELLOW"
            set_fan on
        fi
        update_status "warning" $temp
        
    elif [ $temp -ge $NORMAL_TEMP ]; then
        # NORMAL - Elevated temperature
        if [ "$previous_status" != "normal" ]; then
            log_message "🔵 NORMAL: Temperature $temp°C - Monitoring" "$BLUE"
            set_fan on
        fi
        update_status "normal" $temp
        
    else
        # COOL - Good temperature
        if [ "$previous_status" != "cool" ]; then
            log_message "✅ COOL: Temperature $temp°C - All good" "$GREEN"
            set_fan off
            
            # Restore normal CPU settings
            echo "performance" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null 2>&1 || true
            echo "1200000" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_max_freq >/dev/null 2>&1 || true
        fi
        update_status "cool" $temp
    fi
    
    # Log temperature periodically
    if [ $((RANDOM % 10)) -eq 0 ]; then
        log_message "📊 Temperature: ${temp}°C, Fan: ${FAN_ENABLED}" "$BLUE"
    fi
}

# Function to show status
show_status() {
    if [ -f "$STATUS_FILE" ]; then
        cat "$STATUS_FILE" | python3 -m json.tool 2>/dev/null || cat "$STATUS_FILE"
    else
        echo "No status file found. Thermal monitor may not be running."
    fi
    
    local current_temp=$(read_temperature)
    echo ""
    echo "Current temperature: ${current_temp}°C"
    echo "Fan enabled: $FAN_ENABLED"
}

# Function to clean up
cleanup() {
    log_message "Thermal monitor shutting down..." "$BLUE"
    set_fan off
    rm -f "$STATUS_FILE"
    exit 0
}

# Main script
case "${1:-daemon}" in
    daemon)
        # Create log directory
        mkdir -p "$LOG_DIR"
        
        # Initialize GPIO
        init_gpio
        
        # Set up signal handlers
        trap cleanup SIGTERM SIGINT
        
        log_message "Starting thermal monitor daemon..." "$GREEN"
        log_message "Temperature thresholds: Normal<$NORMAL_TEMP°C, Warning<$WARNING_TEMP°C, Critical<$CRITICAL_TEMP°C, Emergency<$EMERGENCY_TEMP°C" "$BLUE"
        
        # Main monitoring loop
        while true; do
            check_temperature
            sleep $CHECK_INTERVAL
        done
        ;;
    
    status)
        show_status
        ;;
    
    start-fan)
        init_gpio
        set_fan on
        echo "Fan started manually"
        ;;
    
    stop-fan)
        init_gpio
        set_fan off
        echo "Fan stopped manually"
        ;;
    
    test)
        echo "Testing thermal monitoring..."
        init_gpio
        for i in {1..5}; do
            temp=$(read_temperature)
            echo "Temperature reading $i: ${temp}°C"
            check_temperature
            sleep 2
        done
        ;;
    
    *)
        echo "Usage: $0 {daemon|status|start-fan|stop-fan|test}"
        echo ""
        echo "Commands:"
        echo "  daemon     Run thermal monitor in daemon mode"
        echo "  status     Show current thermal status"
        echo "  start-fan  Manually start cooling fan"
        echo "  stop-fan   Manually stop cooling fan"
        echo "  test       Test thermal monitoring functionality"
        exit 1
        ;;
esac