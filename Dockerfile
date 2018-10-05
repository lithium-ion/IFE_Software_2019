FROM ubuntu:16.04

ARG user

# Get Basics
RUN apt-get update && apt-get install -y \
    tmux \
    binutils \
    vim \
    make \
    git \
    curl \
    telnet \
    libusb-1.0-0-dev \
 && rm -rf /var/lib/apt/lists/*

# Get ARM Toolchain
RUN apt-get update && apt-get install -y \
    gcc-arm-none-eabi \
    openocd \
    gdb-arm-none-eabi \
 && rm -rf /var/lib/apt/lists/*

# Add me as a user
RUN useradd -u 1000 ${user} 
