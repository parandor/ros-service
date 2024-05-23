# Use the official ROS 2 Jazzy base image
FROM osrf/ros:jazzy-desktop


# Update package lists and install required tools
RUN apt-get update && apt-get install -y \
    python3-venv \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set up a virtual environment for Python packages
RUN python3 -m venv /opt/ros/env

# Activate the virtual environment and install Flask
RUN /bin/bash -c "source /opt/ros/env/bin/activate && pip install --upgrade pip && pip install Flask"

# For testing, expose port
EXPOSE 5000

# Copy your ROS 2 workspace into the container
# Assuming your workspace is named "ros2_ws"
COPY cpp_parameters /root/ros2_ws
COPY flask-ws /root/flask-ws

# Set the working directory to the ROS 2 workspace
WORKDIR /root/ros2_ws

# Install dependencies and build the workspace
RUN . /opt/ros/jazzy/setup.sh && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

# Source the ROS 2 setup script
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ros/env/bin/activate" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Copy the entrypoint script into the container
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint script
ENTRYPOINT ["/entrypoint.sh"]
