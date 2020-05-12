#!/usr/bin/env python3

import os
import sys
import collections
import re
import logging
import itertools
from xml.sax.saxutils import escape

# Create a `ErrorDescription` tuple with all the information we want to keep.
ErrorDescription = collections.namedtuple(
    'ErrorDescription', 'file line column error error_identifier description')


class ClangTidyConverter:
    # All the errors encountered.
    errors = []

    # Parses the error.
    # Group 1: file path
    # Group 2: line
    # Group 3: column
    # Group 4: error message
    # Group 5: error identifier
    error_regex = re.compile(
        r"^([\w\/\.\-\ ]+):(\d+):(\d+): (.+) (\[[\w\-,\.]+\])$")

    # This identifies the main error line (it has a [the-warning-type] at the end)
    # We only create a new error when we encounter one of those.
    main_error_identifier = re.compile(r'\[[\w\-,\.]+\]$')

    def __init__(self, basename):
        self.basename = basename

    def print_junit_file(self, output_file):
        # Write the header.
        output_file.write("""<?xml version="1.0" encoding="UTF-8" ?>
<testsuites>""")

        sorted_errors = sorted(self.errors, key=lambda x: x.file)

        # Iterate through the errors, grouped by file.
        for file, errorIterator in itertools.groupby(sorted_errors,
                                                     key=lambda x: x.file):
            errors = list(errorIterator)
            error_count = len(errors)

            # Each file gets a test-suite
            output_file.write(
                """\n    <testsuite errors="{error_count}" name="{file}" tests="{error_count}" failures="0" time="0">\n"""
                .format(error_count=error_count, file=file))
            for i, error in enumerate(errors):
                if i != 0:
                    output_file.write("\n")
                # Write each error as a test case.
                output_file.write(" " * 8)
                output_file.write("""<testcase id="{id}" name="{id}" time="0">
            <failure message="{message}">
{htmldata}
            </failure>
        </testcase>""".format(id="[{}/{}] {}".format(error.line, error.column,
                                                     error.error_identifier),
                              message=escape(error.error,
                                             entities={"\"": "&quot;"}),
                              htmldata=escape(error.description)))
            output_file.write("\n    </testsuite>\n")
        output_file.write("</testsuites>\n")

    def process_error(self, error_array):
        if len(error_array) == 0:
            return

        result = self.error_regex.match(error_array[0])
        if result is None:
            logging.warning('Could not match error_array to regex: %s',
                            error_array)
            return

        # Remove ending newlines
        for i, line in enumerate(error_array):
            error_array[i] = line.rstrip("\n")

        line = int(result.group(2))
        col = int(result.group(3))
        error = result.group(4)
        error_identifier = result.group(5)
        error_message = "\n".join(error_array[1:]).rstrip()

        # We remove the `basename` from the `file_path` to make prettier filenames in the JUnit file.
        file_path = result.group(1)
        file_path = file_path.replace(self.basename, "")

        # Also remove `basename` from the message if it contains the path
        error_message = error_message.replace(self.basename, "")

        error = ErrorDescription(file_path, line, col, error, error_identifier,
                                 error_message)
        self.errors.append(error)

    def convert(self, input_file, output_file):
        # Collect all lines related to one error.
        current_error = []
        for line in input_file:
            # If the line starts with a `/`, it is a line about a file.
            if line[0] == '/':
                # Look if it is the start of a error
                if self.main_error_identifier.search(line, re.M):
                    # If so, process any `current_error` we might have
                    self.process_error(current_error)
                    # Initialize `current_error` with the first line of the error.
                    current_error = [line]
                else:
                    # Otherwise, append the line to the error.
                    current_error.append(line)
            elif len(current_error) > 0:
                # If the line didn't start with a `/` and we have a `current_error`, we simply append
                # the line as additional information.
                current_error.append(line)
            else:
                pass

        # If we still have any current_error after we read all the lines,
        # process it.
        if len(current_error) > 0:
            self.process_error(current_error)

        # Print the junit file.
        self.print_junit_file(output_file)


if __name__ == "__main__":
    basename = os.getcwd() + "/"
    converter = ClangTidyConverter(basename)
    converter.convert(sys.stdin, sys.stdout)
