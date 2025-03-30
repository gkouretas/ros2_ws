#!/usr/bin/bash

TARGET=$1
PATH="$(pwd)/src/$TARGET/$TARGET"
PYTHON_UTILS_PATH="../../../python_utils"

# Check if both arguments are provided
if [ -z "$TARGET" ]; then
    echo "Usage: $0 <target>"
    exit 1
fi

# Check if target exists
if [ ! -e "$PATH" ]; then
    echo "Error: Target '$PATH' does not exist."
    exit 1
fi

if [ -L "$PYTHON_UTILS_PATH" ]; then
    echo "Symlink already exists, exiting."
    exit 0
fi

/usr/bin/ln -s "$PYTHON_UTILS_PATH" "$PATH/python_utils"

if [ -L "$PATH/python_utils" ]; then
    echo "Symlink created, exiting."
    exit 0
else
    echo "Failed to create symlink."
    exit 0
fi
