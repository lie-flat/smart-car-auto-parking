#!/bin/sh

# This is a convenience script to start the client on raspberry pi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

python "$SCRIPT_DIR"/run.py;python "$SCRIPT_DIR"/detect.py