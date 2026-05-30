FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Host render/video GIDs so /dev/dri is usable (override at build time).
ARG RENDER_GID=992
ARG VIDEO_GID=44
ARG USERNAME=dev
# Match the host user's UID/GID so the bind-mounted repo and ~/.claude OAuth
# creds (mode 0600) are readable/writable. ubuntu:24.04 squats on uid 1000,
# so we remove it and recreate our user at the host UID.
ARG USER_UID=1000
ARG USER_GID=1000

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

# --- non-root user at the host UID/GID, with render/video access ---
RUN (userdel -r ubuntu 2>/dev/null || true) \
    && (groupdel ubuntu 2>/dev/null || true) \
    && (groupadd -g ${RENDER_GID} render2 2>/dev/null || true) \
    && (groupmod -g ${VIDEO_GID} video 2>/dev/null || true) \
    && groupadd -g ${USER_GID} ${USERNAME} \
    && useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash ${USERNAME} \
    && usermod -aG sudo,video,render2 ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# --- PX4-Autopilot (clone + deps + prebuild SITL target) ---
RUN git clone --depth 1 --recurse-submodules \
        https://github.com/PX4/PX4-Autopilot.git \
    && bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
# Compile the SITL binary ONLY (this target builds and exits; it does NOT launch
# a simulator). Do NOT run a gz_* run-target here: launching Gazebo at build time
# hangs forever (no display, gz-transport loops on "Interrupted system call").
# The Gazebo launch happens at runtime in scripts/start-all.sh, where Xvfb exists.
RUN cd PX4-Autopilot && make px4_sitl_default

# xdotool: used by start-all.sh to resize the Gazebo GUI to fill the display.
# (Late layer so it doesn't invalidate the PX4 build cache above.)
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends xdotool \
    && sudo rm -rf /var/lib/apt/lists/*

# --- Python deps + dronebot (editable) ---
COPY --chown=${USERNAME}:${USERNAME} . /workspace
WORKDIR /workspace
# uvicorn[standard] pulls a WebSocket backend (websockets) — required for the
# cockpit's /chat and /telemetry WS endpoints; plain `uvicorn` has none.
RUN pip3 install --break-system-packages -e ".[dev]" \
    && pip3 install --break-system-packages fastapi "uvicorn[standard]"

ENV PX4_DIR=/home/${USERNAME}/PX4-Autopilot
ENV DRONEBOT_CONNECTION_URL=udpin://0.0.0.0:14540
# Write runtime flight logs to a container-owned path, not the bind-mounted repo.
ENV DRONEBOT_LOG_DIR=/home/${USERNAME}/flight_logs
EXPOSE 8000 6080
CMD ["bash", "scripts/start-all.sh"]
