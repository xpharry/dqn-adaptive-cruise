#!/bin/bash
set -e

# set environment variables
export ROS_PORT_SIM="11311"

exec "$@"
