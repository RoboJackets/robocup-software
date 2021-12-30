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


def get_format_invocation(f, clang_format_binary):
    """Gets a command line for clang-tidy."""
    start = [clang_format_binary, "-i", f]
    return start


def run_format(args, file_queue, lock):
    """Takes filenames out of queue and runs clang-tidy on them."""
    while True:
        name = file_queue.get()
        invocation = get_format_invocation(name, args.clang_format_binary)

        proc = subprocess.Popen(
            invocation, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        output, err = proc.communicate()
        with lock:
            sys.stdout.write(" ".join(invocation) + "\n" + output.decode("utf-8"))
            if len(err) > 0:
                sys.stdout.flush()
                sys.stderr.write(err.decode("utf-8"))
        file_queue.task_done()


def main():
    parser = argparse.ArgumentParser(
        description="Runs clang-format over all files " "in a compilation database."
    )
    parser.add_argument(
        "-clang-format-binary",
        metavar="PATH",
        default="clang-format",
        help="path to clang-format binary",
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
    args = parser.parse_args()

    db_path = "compile_commands.json"

    build_path = args.build_path

    # Load the database and extract all files.
    database = json.load(open(os.path.join(build_path, db_path)))

    cwd = Path.cwd()

    # Build up a big regexy filter from all command line arguments.
    file_name_re = re.compile("|".join(args.files))

    files = []
    for entry in database:
        directory = Path(entry["directory"])
        file = directory / entry["file"]
        relative_path = file.relative_to(cwd)

        if file_name_re.search(str(relative_path)):
            files.append(str(file))
            print(relative_path)

    max_task = args.j
    if max_task == 0:
        max_task = multiprocessing.cpu_count()

    try:
        # Spin up a bunch of tidy-launching threads.
        task_queue = queue.Queue(max_task)
        # List of files with a non-zero return code.
        lock = threading.Lock()
        for _ in range(max_task):
            t = threading.Thread(target=run_format, args=(args, task_queue, lock))
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


if __name__ == "__main__":
    main()
