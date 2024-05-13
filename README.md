
# MobiFarm Simulation

Welcome to the MobiFarm Simulation project repository. This platform uses Docker to encapsulate the development and simulation environment, leveraging ROS Kinetic and Gazebo 7 on Ubuntu, with support for Nvidia GPUs. MobiFarm Simulation is designed to automate tasks in precision agriculture, utilizing a robust simulation framework for development and testing.

## Project Structure

```
mobifarm_simulation/
│
├── docs/                     # Documentation files
│   ├── setup.md              # Instructions for setting up the project
│   ├── usage.md              # Instructions for using the project
│   └── contributing.md       # Guidelines for contributing to the project
│
├── src/                      # Source code
│   ├── controllers/          # Robot control algorithms
│   ├── sensors/              # Sensor interfacing and simulation
│   ├── actuators/            # Actuator control and simulation
│   └── utils/                # Utility scripts and modules
│
├── tests/                    # Test scripts and simulation
│   ├── integration_tests/    # Integration tests
│   └── unit_tests/           # Unit tests
│
├── simulations/              # Gazebo simulation environments
│   └── scenarios/            # Different scenarios for simulation
│   └── gazebo_models/        # Gazebo models to be linked to docker's gazebo
│
├── launch/                   # ROS launch files for deployment
│
├── docker/                   # Docker files and setup
│   ├── Dockerfile            # Dockerfile for building the Docker image
│   └── docker-compose.yml    # Docker Compose file for running the Docker container
│
├── config/                   # Configuration files and parameters
│   ├── robot_params.yaml     # Parameters for the robot
│   └── simulation_params.yaml  # Parameters for the simulation
```

## Installation Guide

1. **Clone the repository:**
   ```
   git clone https://github.com/anh0001/mobifarm_simulation.git
   cd mobifarm_simulation
   ```

2. **Build the Docker image:**
   Navigate to the Docker directory inside the MobiFarm project and build the Docker image. This Dockerfile is configured for ROS Kinetic and Gazebo 7 with Nvidia GPU support.
   ```
   ./mobifarm.sh init-docker
   ```

3. **Run or enter the Docker container:**
   Use Docker Compose or a Docker run command to start the container. Ensure to mount the MobiFarm Simulation directory.
   ```
   ./mobifarm.sh start-docker
   ```

4. **Inside the Docker container:**
   Compile the code:
   ```
   ./mobifarm.sh build
   ```

## Usage Instructions

1. **Load your gazebo models in the simulations/gazebo_models folder**
   Gazebo models can downloaded as in the reference.

1. **Launch the simulation inside the Docker environment:**
   ```
   source devel/setup.bash
   roslaunch mobifarm_gazebo mobifarm_world.launch
   roslaunch mobifarm_gazebo load_robot.launch
   ```

1. **Run the control interface:**
   ```
   source devel/setup.bash
   $ cd src/controllers/mobifarm_control/scripts
   $ python keyboard_controller.py
   ```

1. **Load a world (optional):**
   ```
   $ rosrun gazebo_ros gazebo src/mobifarm_gazebo/world/mobifarm.world
   ```

## Contribution Guidelines

- **Reporting issues:** Use the GitHub issues tab to report and track bugs.
- **Submitting Pull Requests:** Ensure you have tested new features inside the Docker environment. Include unit tests if applicable. Update documentation as needed.
- **Coding Standards:** Follow ROS Python style guidelines and maintain readability and modularity of the code.

## Conclusion

The MobiFarm Simulation is designed for growth and collaboration. We encourage contributions from the robotics and agricultural communities to enhance and evolve this platform for advanced agricultural tasks. This setup using Docker ensures a consistent and controlled development environment for all contributors.

## References

1. Gazebo models can be downloaded from https://github.com/aioz-ai/IROS20_NMFNet