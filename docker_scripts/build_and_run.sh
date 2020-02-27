#!/usr/bin/env bash

echo "-- Installing dependencies"

apt update
cd /package_ws
rosdep install --from-paths src --ignore-src -y

echo "-- Building package"
cd /package_ws
catkin_make

echo "-- Running tests"
source /package_ws/devel/setup.bash
PYTHONUNBUFFERED=1 /package_ws/src/staleness_test/docker_scripts/run_tests.py
