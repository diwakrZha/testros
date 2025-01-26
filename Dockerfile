# Dockerfile

# -----------------------------------------------------------------------------
# 1. Use official ROS2 Humble desktop-full image (includes Gazebo + colcon)
# -----------------------------------------------------------------------------
    FROM osrf/ros:humble-desktop-full

    # -----------------------------------------------------------------------------
    # 2. Install base dependencies
    # -----------------------------------------------------------------------------
    RUN apt-get update && apt-get install -y \
        python3-pip \
        curl \
        x11-apps \
        mesa-utils \
        ros-$ROS_DISTRO-gazebo-* \
        ros-$ROS_DISTRO-xacro \
        ros-$ROS_DISTRO-robot-state-publisher \
        ros-$ROS_DISTRO-tf2-ros \
        # If your base image lacks colcon or related packages, uncomment:
        # colcon-common-extensions \
        && rm -rf /var/lib/apt/lists/*
    
    # -----------------------------------------------------------------------------
    # 3. Set environment variables
    # -----------------------------------------------------------------------------
    ENV ROS_DISTRO=humble
    ENV PYTHONUNBUFFERED=1
    ENV DEBIAN_FRONTEND=noninteractive
    
    # Poetry settings
    ENV POETRY_HOME=/opt/poetry
    ENV PATH="$POETRY_HOME/bin:$PATH"
    ENV POETRY_VIRTUALENVS_CREATE=false
    
    # X11 forwarding variables
    ENV DISPLAY=:0
    ENV QT_X11_NO_MITSHM=1
    
    # -----------------------------------------------------------------------------
    # 4. Install Poetry
    # -----------------------------------------------------------------------------
    RUN curl -sSL https://install.python-poetry.org | python3 - \
        && chmod +x $POETRY_HOME/bin/poetry \
        && ln -s $POETRY_HOME/bin/poetry /usr/local/bin/poetry \
        && poetry --version
    
    # -----------------------------------------------------------------------------
    # 5. Set working directory and copy in just the Poetry config first
    # -----------------------------------------------------------------------------
    WORKDIR /app
    
    COPY pyproject.toml README.md ./
    
    # -----------------------------------------------------------------------------
    # 6. Generate poetry.lock if missing, then install dependencies
    #    ( --no-root means it won’t “pip install .” the local package,
    #      which is often fine if colcon is installing it as a ROS package.
    #      If you DO want to treat your package as a standard Python package,
    #      remove the --no-root. )
    # -----------------------------------------------------------------------------
    RUN poetry lock || echo "poetry.lock generated during build"
    RUN poetry install --no-interaction --no-ansi --no-root
    
    # -----------------------------------------------------------------------------
    # 7. Copy the rest of your project (including src/fg_srv/ with CMakeLists.txt)
    # -----------------------------------------------------------------------------
    COPY . .
    
    # (Optional) If you need to ensure these directories are recognized as packages:
    # RUN touch src/fg_srv/fg_srv/__init__.py src/fg_srv/rest_api/__init__.py
    
    # -----------------------------------------------------------------------------
    # 8. Install testing tools (e.g. pytest)
    # -----------------------------------------------------------------------------
    RUN pip install pytest pytest-asyncio
    
    # -----------------------------------------------------------------------------
    # 9. Make the entrypoint script executable
    # -----------------------------------------------------------------------------
    RUN chmod +x /app/docker-entrypoint.sh
    
    # -----------------------------------------------------------------------------
    # 10. Build the ROS2 workspace
    # -----------------------------------------------------------------------------
    # This step calls colcon build, generating your custom action code (Mission.action)
    # and installing the fg_srv Python package in the local workspace.
    # -----------------------------------------------------------------------------
    RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build"
    
    # -----------------------------------------------------------------------------
    # 11. Expose the API port if your mission API uses 8000
    # -----------------------------------------------------------------------------
    EXPOSE 8000
    
    # -----------------------------------------------------------------------------
    # 12. Set the entrypoint
    # -----------------------------------------------------------------------------
    ENTRYPOINT ["/app/docker-entrypoint.sh"]
    