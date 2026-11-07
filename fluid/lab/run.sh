#!/bin/sh
# Opens a fluid sample, by default the tank. Settings are saved in build/, not in the Box2D tree.
# For example: fluid/lab/run.sh "Paddle Boat"
cd "$(dirname "$0")/../../build" && SAMPLE_NAME="${1:-Tank}" exec bin/samples
