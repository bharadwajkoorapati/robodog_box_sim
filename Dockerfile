# Choose the ROS 2 version that matches your development
FROM ros:jazzy-ros-base

# 1. Install Node.js (required by Freebuff) and clean up apt lists correctly
RUN apt-get update && apt-get install -y \
    curl \
    git \
    && curl -fsSL https://deb.nodesource.com/setup_lts.x | bash - \
    && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*

# 2. Install the global Freebuff CLI agent
RUN npm install -g freebuff

# 3. Install missing ROS 2 Navigation packages required by your workspace
RUN apt-get update && apt-get install -y \
    ros-jazzy-nav2-msgs \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory inside the container
WORKDIR /ros2_ws

# Copy your local source code into the container's workspace
COPY src/ src/

# Source the ROS 2 environment and build the package
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build

# Source both ROS 2 and your workspace automatically for interactive sessions
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Default to starting the Freebuff agent when the container runs
CMD ["freebuff"]

