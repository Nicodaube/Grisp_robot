#!/bin/bash


#Created with the help of ChatGPT

MAX_TRIES=10
COUNT=0

DEVICE_IP=$(ip route get 1.1.1.1 | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}')
BROADCAST_IP=$(ip -o -f inet addr show | awk '/scope global/ {print $6}' | head -n1)

while [ $COUNT -lt $MAX_TRIES ]; do
    if [ $# -eq 1 ]; then
        FILENAME="$1"
        python3 Controller.py "$DEVICE_IP" "$BROADCAST_IP" "$FILENAME"
    fi

    if [ $# -lt 1 ]; then
        python3 Controller.py "$DEVICE_IP" "$BROADCAST_IP"
    fi
    
    EXIT_CODE=$?
    if [ $EXIT_CODE -eq 0 ]; then
        echo "Controller.py exited successfully."
        exit 0
    else
        COUNT=$((COUNT + 1))
        echo "Controller.py failed (attempt $COUNT/$MAX_TRIES). Retrying in 0.5s..."
        sleep 0.5
    fi
done

echo "Controller.py failed $MAX_TRIES times. Exiting."
exit 1
