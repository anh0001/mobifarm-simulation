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
    echo "Checking if Docker container is already running..."
    # Check if the Docker container is already running.
    if [ "$(docker ps -q -f name=mobisim)" ]; then
        enter_docker
    else
        echo "Starting Docker container..."
        # Start docker-compose.yml in the ./docker directory.
        docker-compose -f ./docker/docker-compose.yml up -d
        echo "Docker container started successfully."
    fi
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
    echo "Checking if Docker container is already running..."
    # Check if the Docker container is already running.
    if [ "$(docker ps -q -f name=mobisim)" ]; then
        echo "Entering Docker container..."
        # Enter the Docker container.
        docker exec -it mobisim bash
    fi
}

# Function to build the project.
build_project() {
    echo "Checking if is running in the Docker container..."
    # Check if the script is running in the Docker container.
    if [ -f /.dockerenv ]; then
        echo "Running in the Docker container."
        echo "Building the project..."
        # Build the project.
        catkin_make
        echo "Project built successfully."
    else
        echo "Not running in the Docker container."
        echo "Please run the script in the Docker container."
        exit 1
    fi
}

# Function to clean the project.
clean_project() {
    echo "Checking if is running in the Docker container..."
    # Check if the script is running in the Docker container.
    if [ -f /.dockerenv ]; then
        echo "Running in the Docker container."
        echo "Cleaning the project..."

        # Clean the project.
        catkin_make clean
        rm -rf build/ devel/
        # Clear the ROS log directory
        rm -rf ~/.ros/log/*

        echo "Project cleaned successfully."
    else
        echo "Not running in the Docker container."
        echo "Please run the script in the Docker container."
        exit 1
    fi
}

# Function to run test displaying a world in Gazebo
run_test_display_world() {
    echo "Checking if is running in the Docker container..."
    # Check if the script is running in the Docker container.
    if [ -f /.dockerenv ]; then
        echo "Running in the Docker container."
        echo "Running test..."
        # Run the test.
        source devel/setup.bash
        rostest mobifarm_gazebo test_world.launch
    else
        echo "Not running in the Docker container."
        echo "Please run the script in the Docker container."
        exit 1
    fi
}

# Function to show the latest ros log.
show_latest_ros_log() {
    echo "Checking if is running in the Docker container..."
    # Check if the script is running in the Docker container.
    if [ -f /.dockerenv ]; then
        echo "Running in the Docker container."
        echo "Showing the latest ROS log..."
        # Find the latest file in the ROS log directory
        latest_file=$(ls -t ~/.ros/log/ | head -n1)

        # View the file using less
        less ~/.ros/log/"$latest_file"
    else
        echo "Not running in the Docker container."
        echo "Please run the script in the Docker container."
        exit 1
    fi
}

# Function to show the usage of the script.
usage() {
    # Display in details the usage of the script.
    echo "MobiFarm Simulation Docker Script"
    echo "Usage: $0 {commands}"
    echo "  init-docker: Build the Docker image for MobiFarm Simulation."
    echo "  start-docker: Start the Docker container for MobiFarm Simulation."
    echo "  stop-docker: Stop the Docker container for MobiFarm Simulation."
    echo "  enter-docker: Enter the Docker container for MobiFarm Simulation."
    echo "  build: Build the project in the Docker container."
    echo "  clean: Clean the project in the Docker container."
    echo "  roslog: Show the latest ROS log in the Docker container."
    echo "  test-display-world: Run test displaying a world in Gazebo."
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
    build)
        build_project
        ;;
    clean)
        clean_project
        ;;
    roslog)
        show_latest_ros_log
        ;;
    test-display-world)
        run_test_display_world
        ;;
    *)
        usage
        ;;
esac
