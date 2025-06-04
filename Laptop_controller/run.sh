#!/bin/bash

MAX_TRIES=10
COUNT=0

while [ $COUNT -lt $MAX_TRIES ]; do
    python3 Controller.py
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