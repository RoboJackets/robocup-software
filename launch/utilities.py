#
# Utility functions for the various launch files
#

import os

from typing import Optional

def find_simulator_cli() -> Optional[str]:
    for root, _, filenames in os.walk("~"):
        if "simulator-cli" in filenames:
            return os.path.join(root, "simulator-cli")
    return None
