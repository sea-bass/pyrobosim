FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    build-essential \
    git \
    libgl1 \
    libglib2.0-0 \
    libxkbcommon-x11-0 \
    libxcb-xinerama0 \
    libxcb-cursor0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-xfixes0 \
    libxcb-xkb1 \
    libx11-xcb1 \
    libxrender1 \
    libxext6 \
    libsm6 \
    libx11-6 \
    libxfixes3 \
    libxi6 \
    libxrandr2 \
    libxcb1 \
    libegl1 \
    libdbus-1-3 \
    libfontconfig1 \
    libfreetype6 \
    libpng16-16 \
    libjpeg-turbo8 \
    libgeos-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/pyrobosim

COPY pyrobosim /workspace/pyrobosim/pyrobosim
COPY pyrobosim/setup.py /workspace/pyrobosim/pyrobosim/setup.py
COPY pyrobosim/README.md /workspace/pyrobosim/pyrobosim/README.md

RUN python3 -m pip install --no-cache-dir -U pip \
    && python3 -m pip install --no-cache-dir -e /workspace/pyrobosim/pyrobosim \
    && python3 -m pip install --no-cache-dir -U py_trees fastmcp

CMD ["bash"]
