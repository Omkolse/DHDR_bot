#!/bin/bash
while true; do
    MEMORY=$(free -m | awk 'NR==2{printf "%.2f%%", $3*100/$2}')
    TEMP=$(vcgencmd measure_temp | cut -d= -f2)
    echo "$(date): Memory: $MEMORY, Temp: $TEMP"
    
    # Alert if memory > 85%
    if (( $(echo "$MEMORY > 85" | bc -l) )); then
        echo "WARNING: High memory usage!"
    fi
    
    sleep 5
done