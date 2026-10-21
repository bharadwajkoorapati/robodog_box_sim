# Choose the ROS 2 version that matches your development
FROM ros:humble-ros-base

# 1. Install Node.js (required by Freebuff) and clean up apt lists correctly
RUN apt-get update && apt-get install -y \
    curl \
    git \
    && curl -fsSL https://deb.nodesource.com/setup_lts.x | bash - \
    && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*

# 2. Install the global Freebuff CLI agent
RUN npm install -g freebuff

# Create workspace directory inside the container
WORKDIR /ros2_ws

# Copy your local source code into the container's workspace
COPY src/ src/

# (Optional) If your workspace packages have dependencies, install them here:
# RUN apt-get update && rosdep update && \
#     rosdep install --from-paths src --ignore-src -r -y && \
#     rm -rf /var/lib/apt/lists/*

# Source the ROS 2 environment and build the package
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build

# Source both ROS 2 and your workspace automatically for interactive sessions
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Default to starting the Freebuff agent when the container runs
CMD ["freebuff"]