FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Host render/video GIDs so /dev/dri is usable (override at build time).
ARG RENDER_GID=992
ARG VIDEO_GID=44
ARG USERNAME=dev

# --- base + rendering + vnc + python + node ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo git curl wget gnupg lsb-release ca-certificates \
        python3 python3-pip python3-venv \
        xvfb x11-utils x11vnc mesa-utils libgl1-mesa-dri \
        novnc websockify \
        build-essential \
    && rm -rf /var/lib/apt/lists/*

# --- Node + Claude Code CLI (for the Agent SDK / OAuth) ---
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y nodejs \
    && npm install -g @anthropic-ai/claude-code \
    && rm -rf /var/lib/apt/lists/*

# --- Gazebo Harmonic (OSRF apt repo) ---
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# --- non-root user with render/video access ---
RUN groupadd -g ${RENDER_GID} render2 2>/dev/null || true \
    && groupmod -g ${VIDEO_GID} video 2>/dev/null || true \
    && useradd -m -s /bin/bash ${USERNAME} \
    && usermod -aG sudo,video,render2 ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# --- PX4-Autopilot (clone + deps + prebuild SITL target) ---
RUN git clone --depth 1 --recurse-submodules \
        https://github.com/PX4/PX4-Autopilot.git \
    && bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
# Prebuild the gz_x500_depth target so first run is fast (DONT_RUN avoids launch).
RUN cd PX4-Autopilot && DONT_RUN=1 make px4_sitl gz_x500_depth || true

# --- Python deps + dronebot (editable) ---
COPY --chown=${USERNAME}:${USERNAME} . /workspace
WORKDIR /workspace
RUN pip3 install --break-system-packages -e ".[dev]" \
    && pip3 install --break-system-packages fastapi uvicorn

ENV PX4_DIR=/home/${USERNAME}/PX4-Autopilot
ENV DRONEBOT_CONNECTION_URL=udp://:14540
EXPOSE 8000 6080
CMD ["bash", "scripts/start-all.sh"]
