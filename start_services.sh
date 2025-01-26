#!/bin/bash

# Function to detect the primary network interface and retrieve the IP address
detect_host_ip() {
    HOST_IP=$(ip route get 1 | awk '{print $NF;exit}')
    echo "$HOST_IP"
}

# Function to stop Mosquitto on host if it's running
stop_host_mosquitto() {
    if systemctl is-active --quiet mosquitto; then
        echo "Mosquitto service is running on the host."
        echo "Stopping host Mosquitto service to avoid port conflicts..."
        sudo systemctl stop mosquitto
        if [ $? -eq 0 ]; then
            echo "Host Mosquitto service stopped successfully."
        else
            echo "Failed to stop host Mosquitto service."
            exit 1
        fi
    else
        echo "Mosquitto service is not running on the host."
    fi
}

# Check for "test" argument
if [[ "$1" == "test" ]]; then
    echo "Running in test mode..."
    export TEST_MODE=true
else
    echo "Running in regular mode..."
    export TEST_MODE=false
fi

# Enable X11 forwarding (for GUI-based Docker)
xhost +local:root

# Stop host's Mosquitto if running
stop_host_mosquitto


# Detect host IP
HOST_IP=$(detect_host_ip)
if [ -z "$HOST_IP" ]; then
    echo "Failed to detect host IP address."
    exit 1
fi
echo "Detected Host IP: $HOST_IP"
export HOST_IP


# Start Docker Compose
if [[ "$TEST_MODE" == "true" ]]; then
    echo "Starting Docker Compose services with 'test' profile..."
    docker compose --profile test up
else
    echo "Starting Docker Compose services in regular mode..."
    docker compose up
fi
