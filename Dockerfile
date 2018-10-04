FROM ubuntu:16.04
# Get Basics
RUN apt-get update && apt-get install -y \
    tmux \
    binutils \
    vim \
    gcc-5 \
    g++-5 \
    gdb \
    make \
    git \
    curl \
    software-properties-common \
 && rm -rf /var/lib/apt/lists/*

# Get ARM Toolchain
RUN add-apt-repository ppa:team-gcc-arm-embedded/ppa \
    && apt-get update \
    && apt-get install -y \
    gcc-arm-embedded \
 && rm -rf /var/lib/apt/lists/*
    
