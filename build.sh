#!/bin/bash
ROS_VERSION=noetic

IMAGE_NAME=control-libraries-ros-demos:${ROS_VERSION}

BUILD_FLAGS=()
BRANCH=develop
while [ "$#" -gt 0 ]; do
  case "$1" in
  -r|--rebuild)
    BUILD_FLAGS+=(--no-cache)
    shift 1
    ;;
  -v|--verbose)
    BUILD_FLAGS+=(--progress=plain)
    shift 1
    ;;
  *)
    echo "Unknown option: $1" >&2
    exit 1
    ;;
  esac
done

docker pull ghcr.io/aica-technology/ros-control-libraries:"${ROS_VERSION}"

BUILD_FLAGS+=(--build-arg ROS_VERSION="${ROS_VERSION}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}")

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" .
