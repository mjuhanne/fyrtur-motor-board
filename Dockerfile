# Build this container with (you can use docker instead of podman if you'd like):
#
# $ podman build . -t fyrtur-motor-board
#
# To build the firmware with the container image produced above, run:
#
# $ podman run --rm -it -v ${PWD}:/build:Z fyrtur-motor-board make -f STM32Make.make

FROM debian:11-slim

RUN apt-get update -y && apt-get install -y \
    gcc-arm-none-eabi make \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build
