#!/bin/bash

# Function to stop Docker Compose services
stop_docker_compose_services() {
    echo "Stopping Docker Compose services..."
    docker compose down
    if [ $? -eq 0 ]; then
        echo "Docker Compose services stopped successfully."
    else
        echo "Failed to stop Docker Compose services."
        exit 1
    fi
}

# Function to stop host Mosquitto if it's running
stop_host_mosquitto() {
    if systemctl is-active --quiet mosquitto; then
        echo "Host Mosquitto service is running. Stopping it..."
        sudo systemctl stop mosquitto
        if [ $? -eq 0 ]; then
            echo "Host Mosquitto service stopped successfully."
        else
            echo "Failed to stop host Mosquitto service."
            exit 1
        fi
    else
        echo "Host Mosquitto service is not running."
    fi
}

# Main script execution
echo "Stopping services..."

# 1. Stop Docker Compose services (includes MQTT container if running via Docker)
stop_docker_compose_services

# 2. Stop host Mosquitto (in case your host runs it outside Docker)
stop_host_mosquitto

echo "All services have been stopped."
