#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# Function to build the Docker image.
init_docker() {
    echo "Building Docker image for MobiFarm Simulation..."
    docker build -t mobisim ./docker
    echo "Docker image built successfully."
}

# Function to start the Docker container using docker-compose.
start_docker() {
    echo "Starting Docker container..."
    # Start docker-compose.yml in the ./docker directory.
    docker-compose -f ./docker/docker-compose.yml up -d
    echo "Docker container started successfully."
}

# Function to stop the Docker container.
stop_docker() {
    echo "Stopping Docker container..."
    # Stop docker-compose.yml in the ./docker directory.
    docker-compose -f ./docker/docker-compose.yml down
    echo "Docker container stopped."
}

# Function to enter the Docker container.
enter_docker() {
    echo "Entering Docker container..."
    # Enter the Docker container.
    docker exec -it mobisim bash
}

# Function to show the usage of the script.
usage() {
    # Display in details the usage of the script.
    echo "MobiFarm Simulation Docker Script"
    echo "Usage: $0 {init-docker|start-docker|stop-docker|enter-docker}"
    echo "  init-docker: Build the Docker image for MobiFarm Simulation."
    echo "  start-docker: Start the Docker container for MobiFarm Simulation."
    echo "  stop-docker: Stop the Docker container for MobiFarm Simulation."
    echo "  enter-docker: Enter the Docker container for MobiFarm Simulation."
    echo ""

    exit 1
}

# Main script logic based on the command line argument.
case "$1" in
    init-docker)
        init_docker
        ;;
    start-docker)
        start_docker
        ;;
    stop-docker)
        stop_docker
        ;;
    enter-docker)
        enter_docker
        ;;
    *)
        usage
        ;;
esac
