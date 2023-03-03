FROM ubuntu:latest

ARG DEBIAN_FRONTEND=noninteractive # ignore user input required

# install required dependencies
RUN apt-get -y update && apt-get install -y
RUN apt-get -y install g++ cmake git
RUN apt-get install -y apt-utils

# install tslam deps
RUN apt-get -y install libopencv-dev=4.5.4+dfsg-9ubuntu4
RUN apt-get -y install libgmp-dev libgmp-dev
RUN apt-get -y install libmpfr-dev
RUN apt-get -y install libeigen3-dev
RUN apt-get -y install libboost-all-dev

COPY . .
WORKDIR .

# Config/build cmake 
RUN cmake -S . -B build
RUN cmake --build build