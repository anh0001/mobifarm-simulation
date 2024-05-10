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
    echo "Checking if the Docker container name mobisim has been created..."
    # Check if the Docker container name mobisim has been created.
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
    # Make sure it is not in a container.
    if [ -f /.dockerenv ]; then
        echo "Already in Docker container."
        exit 1
    fi

    # Enter the Docker container.
    docker exec -it mobisim bash
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
        latest_file=$(find ~/.ros/log/ -type f -printf "%T@ %p\n" | sort -n | tail -1 | cut -f2- -d" ")

        # Check if ii is valid log
        if [ -z "$latest_file" ]; then
            echo "No ROS log found."
            exit 1
        fi

        # View the file using less
        less "$latest_file"
    else
        echo "Not running in the Docker container."
        echo "Please run the script in the Docker container."
        exit 1
    fi
}

# Function to show the katest gazebo log.
show_latest_gazebo_log() {
    echo "Checking if is running in the Docker container..."
    # Check if the script is running in the Docker container.
    if [ -f /.dockerenv ]; then
        echo "Running in the Docker container."
        echo "Showing the latest Gazebo log..."
        # Find the latest file in the Gazebo log directory
        latest_file=$(find ~/.gazebo/log/ -type f -printf "%T@ %p\n" | sort -n | tail -1 | cut -f2- -d" ")

        # Check if ii is valid log
        if [ -z "$latest_file" ]; then
            echo "No Gazebo log found."
            exit 1
        fi 

        # View the file using less
        less "$latest_file"
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
    echo "  gazebo-log: Show the latest Gazebo log in the Docker container."
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
    gazebolog)
        show_latest_gazebo_log
        ;;
    test-display-world)
        run_test_display_world
        ;;
    *)
        usage
        ;;
esac
