FROM ubuntu:latest

ARG DEBIAN_FRONTEND=noninteractive # ignore user input required

# install required dependencies
# # install required dependencies
RUN apt-get -qq update && apt-get -qq -y install \
    g++ cmake git \
    && rm -rf /var/lib/apt/lists/*

# install tslam deps
RUN apt-get -qq update && apt-get -qq -y install \
    libopencv-dev \
    libgmp-dev \
    libmpfr-dev \
    libeigen3-dev \
    libboost-all-dev\
    && rm -rf /var/lib/apt/lists/*


# COPY . .
# WORKDIR .

# # Config/build cmake
# RUN cmake -S . -B build
# RUN cmake --build build
