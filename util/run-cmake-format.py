#!/usr/bin/env python

import argparse
import json
import multiprocessing
import os
import re
import subprocess
import sys
import threading
from pathlib import Path
import queue as queue


def get_format_invocation(f, cmake_format_binary):
    """Gets a command line for cmake-format."""
    start = [cmake_format_binary, "-i", f]
    return start


def run_format(args, file_queue, lock, return_codes):
    """Takes filenames out of queue and runs clang-format on them."""
    while True:
        name = file_queue.get()
        invocation = get_format_invocation(
            name, args.cmake_format_binary, args.check
        )

        proc = subprocess.Popen(
            invocation, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        output, err = proc.communicate()
        with lock:
            return_codes.append(proc.returncode)
            sys.stdout.write(
                " ".join(invocation) + "\n" + output.decode("utf-8")
            )
            if len(err) > 0:
                sys.stdout.flush()
                sys.stderr.write(err.decode("utf-8"))
        file_queue.task_done()


def main():
    parser = argparse.ArgumentParser(
        description="Runs cmake-format over all CMakeLists.txt"
    )
    parser.add_argument(
        "-cmake-format-binary",
        metavar="PATH",
        default="cmake-format",
        help="path to cmake-format binary",
    )
    parser.add_argument(
        "-i", action="store_true", help="Inplace edit <file>s, if specified"
    )
    parser.add_argument(
        "-j",
        type=int,
        default=0,
        help="number of format instances to be run in parallel.",
    )
    parser.add_argument(
        "files",
        nargs="*",
        default=[".*"],
        help="files to be processed (regex on path)",
    )
    parser.add_argument(
        "-p",
        dest="build_path",
        help="Path used to read a compile command database.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Exit with status code 0 if formatting would not change "
        "file contents, or status code 1 if it would",
    )
    args = parser.parse_args()

    cwd = Path.cwd()

    # Build up a big regexy filter from all command line arguments.
    file_name_re = re.compile("|".join(args.files))

    files = []
    for file in cwd.glob("**/CMakeLists.txt"):
        relative_file = file.relative_to(cwd)
        if file_name_re.search(str(relative_file)):
            files.append(str(file))
            print(relative_file)
            # print(file)

    max_task = args.j
    if max_task == 0:
        max_task = multiprocessing.cpu_count()

    return_codes = []
    try:
        # Spin up a bunch of tidy-launching threads.
        task_queue = queue.Queue(max_task)
        # List of files with a non-zero return code.
        lock = threading.Lock()
        for _ in range(max_task):
            t = threading.Thread(
                target=run_format, args=(args, task_queue, lock, return_codes)
            )
            t.daemon = True
            t.start()

        # Fill the queue with files.
        for name in files:
            task_queue.put(name)

        # Wait for all threads to be done.
        task_queue.join()

    except KeyboardInterrupt:
        # This is a sad hack. Unfortunately subprocess goes
        # bonkers with ctrl-c and we start forking merrily.
        print("\nCtrl-C detected, goodbye.")
        os.kill(0, 9)

    for return_code in return_codes:
        if return_code != 0:
            sys.exit(return_code)

    sys.exit(0)


if __name__ == "__main__":
    main()
