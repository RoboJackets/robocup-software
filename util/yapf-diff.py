#!/usr/bin/env python
#
# ===- clang-format-diff.py - ClangFormat Diff Reformatter ----*- python -*--===#
#
#                     The LLVM Compiler Infrastructure
#
# This file is distributed under the University of Illinois Open Source
# License. See LICENSE.TXT for details.
#
# ===------------------------------------------------------------------------===#
r"""
ClangFormat Diff Reformatter
============================

This script reads input from a unified diff and reformats all the changed
lines. This is useful to reformat all the lines touched by a specific patch.
Example usage for git/svn users:

  git diff -U0 --no-color HEAD^ | clang-format-diff.py -p1 -i
  svn diff --diff-cmd=diff -x-U0 | clang-format-diff.py -i

"""

import argparse
import difflib
import re
import string
import subprocess
import sys
from io import StringIO


def main():
    parser = argparse.ArgumentParser(
        description="Reformat changed lines in diff. Without -i "
        "option just output the diff that would be "
        "introduced."
    )
    parser.add_argument(
        "-i",
        action="store_true",
        default=False,
        help="apply edits to files instead of displaying a diff",
    )
    parser.add_argument(
        "-p",
        metavar="NUM",
        default=0,
        help="strip the smallest prefix containing P slashes",
    )
    parser.add_argument(
        "-regex",
        metavar="PATTERN",
        default=None,
        help="custom pattern selecting file paths to reformat "
        "(case sensitive, overrides -iregex)",
    )
    parser.add_argument(
        "-iregex",
        metavar="PATTERN",
        default=r".*\.(py)",
        help="custom pattern selecting file paths to reformat "
        "(case insensitive, overridden by -regex)",
    )
    parser.add_argument(
        "-style",
        help="specify formatting style: either a style name (for "
        'example "pep8" or "google"), or the name of a file with '
        "style settings. The default is pep8 unless a "
        ".style.yapf or setup.cfg file located in one of the "
        "parent directories of the source file (or current "
        "directory for stdin)",
    )
    parser.add_argument(
        "-binary", default="yapf", help="location of binary to use for yapf"
    )
    args = parser.parse_args()

    # Extract changed lines for each file.
    filename = None
    lines_by_file = {}
    for line in sys.stdin:
        match = re.search(r"^\+\+\+\ (.*?/){%s}(\S*)" % args.p, line)
        if match:
            filename = match.group(2)
        if filename == None:
            continue

        if args.regex is not None:
            if not re.match("^%s$" % args.regex, filename):
                continue
        else:
            if not re.match("^%s$" % args.iregex, filename, re.IGNORECASE):
                continue

        match = re.search(r"^@@.*\+(\d+)(,(\d+))?", line)
        if match:
            start_line = int(match.group(1))
            line_count = 1
            if match.group(3):
                line_count = int(match.group(3))
            if line_count == 0:
                continue
            end_line = start_line + line_count - 1
            lines_by_file.setdefault(filename, []).extend(
                ["--lines", str(start_line) + "-" + str(end_line)]
            )

    # Reformat files containing changes in place.
    for filename, lines in lines_by_file.items():
        command = [args.binary]
        if args.i:
            command.append("-i")
        command.extend(lines)
        if args.style:
            command.extend(["--style", args.style])
        command.append(filename)
        p = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=None, stdin=subprocess.PIPE
        )
        stdout, stderr = p.communicate()
        if p.returncode != 0:
            sys.exit(p.returncode)

        if not args.i:
            with open(filename) as f:
                code = f.readlines()
            formatted_code = StringIO(str(stdout, "utf-8")).readlines()
            diff = difflib.unified_diff(
                code,
                formatted_code,
                filename,
                filename,
                "(before formatting)",
                "(after formatting)",
            )
            diff_string = "".join(diff)
            if len(diff_string) > 0:
                sys.stdout.write(diff_string)


if __name__ == "__main__":
    main()
