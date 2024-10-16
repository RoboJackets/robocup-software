#!/bin/bash

# Find the binary named simulator-cli somewhere inside the home directory
binary=$(find ~ -type f -name "simulator-cli" -print -quit)

# Check if the binary was found
if [ -z "$binary" ]; then
    echo "Binary 'simulator-cli' not found."
    exit 1
fi

# Run the binary in the background
"$binary" -g 2020B --realism RC2021 &
binary_pid=$!

# Ensure that pressing Ctrl+C kills all subprocesses
trap 'kill $binary_pid; exit' INT

# Run "make run-our-stack" in the foreground
make run-our-stack

# Wait for the background process to complete
wait $binary_pid

