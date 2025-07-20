#!/bin/bash

# Output CSV file
LOGFILE="/home/bristol-fsai/logs/io_logs.csv"

# Interval in seconds between samples
INTERVAL=5

# Write CSV header
echo "Timestamp,CPU_Usage(%),Disk_Read(kB/s),Disk_Write(kB/s),GPU_Usage(%),GPU_Memory_Usage(%)" > "$LOGFILE"

while true; do
    TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")

    # CPU Usage: user+system from mpstat
    CPU_USAGE=$(mpstat 1 1 | awk '/Average:/ {print 100 - $NF}')

    # Disk I/O: using iostat
    DISK_IO=($(iostat -d -k 1 2 | grep -A1 "^Device" | tail -n +2 | awk '{read+=$3; write+=$4} END {print read, write}'))

    DISK_READ=${DISK_IO[0]:-0}
    DISK_WRITE=${DISK_IO[1]:-0}

    # GPU Usage and Memory Usage: using nvidia-smi
    if command -v nvidia-smi &> /dev/null; then
        GPU_STATS=$(nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader,nounits)
        GPU_UTIL=$(echo "$GPU_STATS" | awk -F',' '{print $1}')
        GPU_MEM_USED=$(echo "$GPU_STATS" | awk -F',' '{print $2}')
        GPU_MEM_TOTAL=$(echo "$GPU_STATS" | awk -F',' '{print $3}')
        GPU_MEM_USAGE=$(awk "BEGIN {printf \"%.2f\", ($GPU_MEM_USED/$GPU_MEM_TOTAL)*100}")
    else
        GPU_UTIL=0
        GPU_MEM_USAGE=0
    fi

    # Append to log file
    echo "$TIMESTAMP,$CPU_USAGE,$DISK_READ,$DISK_WRITE,$GPU_UTIL,$GPU_MEM_USAGE" >> "$LOGFILE"

    sleep $INTERVAL
done
