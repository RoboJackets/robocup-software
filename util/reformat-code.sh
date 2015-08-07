#!/bin/bash

# pass the "--check" flag to this script to run in checkstyle mode
FIX_ISSUES=true
if [ "$1" == "--check" ]; then
    FIX_ISSUES=false
    echo "Running checkstyle"
fi

RET=0

find . -path ./build -prune -o -path ./third_party -prune -o -name "*.hpp" -o -name "*.h" -o -name "*.c" -o -name "*.cpp" | while read filename; do
    clang-format -style=file $filename -output-replacements-xml | grep "<replacement " > /dev/null
    rc=$?
    if [[ $rc == 0 ]]; then
        if $FIX_ISSUES; then
            clang-format -style=file $filename -i
            echo "Formatted file: $filename"
        else
            echo "File needs formatting: $filename"
            exit 1
        fi
    fi
done
