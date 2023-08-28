FROM osrf/ros:humble-desktop

SHELL [ "/bin/bash", "-c" ]
WORKDIR /ws
COPY . .

# Install dep
RUN rosdep update -y && \
    rosdep install -r --from-paths src -i -y --rosdistro humble

# Build and install
RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source install/setup.bash' >> ~/.bashrc

WORKDIR /ws