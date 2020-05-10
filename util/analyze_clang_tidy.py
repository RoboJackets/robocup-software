import re
from collections import defaultdict
import os
import argparse


def parse_args() -> str:
    parser = argparse.ArgumentParser(
        description="Summarizes the output from clang-tidy to show the number of each warning and locations."
    )
    parser.add_argument("path", help="Path to output of clang-tidy", type=str)
    args = parser.parse_args()
    return args.path


def main():
    log_path = parse_args()
    try:
        log = open(log_path)
    except FileNotFoundError:
        print(f"'{log_path}' is not a valid file.")
        exit(1)

    warning_dict = defaultdict(int)
    locations_dict = defaultdict(list)

    category_re = r"(.*):(\d+):(\d+).*warning\: .+\[(.+)\]"
    total_matches = 0
    for line in log.readlines():
        match = re.search(category_re, line)
        if match:
            total_matches += 1
            filename = match.group(1)
            filename = os.path.basename(os.path.normpath(filename))
            line_number = match.group(2)
            char_pos = match.group(3)
            warning_type = match.group(4)
            warning_dict[warning_type] += 1
            locations_dict[warning_type] += [f"{filename}:{line_number}"]
    log.close()

    longest_key = max([len(k) for k in warning_dict.keys()])

    for k in sorted(warning_dict, key=warning_dict.get, reverse=True):
        locations_str = ", ".join(locations_dict[k])
        max_length = 80
        if len(locations_str) > max_length:
            locations_str = locations_str[:max_length] + "..."
        justified_key = k.ljust(longest_key, '·')
        print(f"{justified_key}: {warning_dict[k]:3} - {locations_str}")
    print("--------------------------------")
    print(f"Total warnings: {total_matches}")


if __name__ == '__main__':
    main()
