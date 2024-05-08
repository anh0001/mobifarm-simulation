
# MoboFarm Simulation

Welcome to the MoboFarm Simulation project repository. This platform uses Docker to encapsulate the development and simulation environment, leveraging ROS Kinetic and Gazebo 7 on Ubuntu, with support for Nvidia GPUs. MoboFarm Simulation is designed to automate tasks in precision agriculture, utilizing a robust simulation framework for development and testing.

## Project Structure

```
mobofarm_simulation/
│
├── docs/                     # Documentation files
│   ├── setup.md
│   ├── usage.md
│   └── contributing.md
│
├── src/                      # Source code
│   ├── controllers/          # Robot control algorithms
│   ├── sensors/              # Sensor interfacing and simulation
│   ├── actuators/            # Actuator control and simulation
│   └── utils/                # Utility scripts and modules
│
├── tests/                    # Test scripts and simulation
│   ├── integration_tests/
│   └── unit_tests/
│
├── simulations/              # Gazebo simulation environments
│   └── scenarios/
│
├── launch/                   # ROS launch files for deployment
│
├── docker/                   # Docker files and setup
│   ├── Dockerfile
│   └── docker-compose.yml
│
├── config/                   # Configuration files and parameters
│   ├── robot_params.yaml
│   └── simulation_params.yaml
│
└── CMakeLists.txt            # Build script
```

## Installation Guide

1. **Clone the repository:**
   ```
   git clone https://github.com/anh0001/mobofarm_simulation.git
   cd mobofarm_simulation
   ```

2. **Build the Docker image:**
   Navigate to the Docker directory inside the MoboFarm project and build the Docker image. This Dockerfile is configured for ROS Kinetic and Gazebo 7 with Nvidia GPU support.
   ```
   cd docker
   docker build -t mobofarm_simulation_image .
   ```

3. **Run the Docker container:**
   Use Docker Compose or a Docker run command to start the container. Ensure to mount the MoboFarm Simulation directory.
   ```
   docker run -v $(pwd)/../:/root/mobofarm_simulation --gpus all -it mobofarm_simulation_image
   ```

4. **Inside the Docker container:**
   Compile the code:
   ```
   cd /root/mobofarm_simulation
   catkin_make
   source devel/setup.bash
   ```

## Usage Instructions

1. **Launch the simulation inside the Docker environment:**
   ```
   roslaunch mobofarm_simulation simulation.launch
   ```

2. **Run the control interface:**
   ```
   rosrun mobofarm_simulation controller.py
   ```

## Contribution Guidelines

- **Reporting issues:** Use the GitHub issues tab to report and track bugs.
- **Submitting Pull Requests:** Ensure you have tested new features inside the Docker environment. Include unit tests if applicable. Update documentation as needed.
- **Coding Standards:** Follow ROS Python style guidelines and maintain readability and modularity of the code.

## Conclusion

The MoboFarm Simulation is designed for growth and collaboration. We encourage contributions from the robotics and agricultural communities to enhance and evolve this platform for advanced agricultural tasks. This setup using Docker ensures a consistent and controlled development environment for all contributors.
