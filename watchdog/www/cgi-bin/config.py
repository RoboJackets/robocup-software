# -*- coding: utf-8 -*-
# Defaults

import socket
hostname = socket.gethostname()

# Tuple of all hosts that are listed on the status page.
# Also used for the host pulldown when adding.
hosts = [hostname]

# Path to the stylesheet
stylesheet = '/default.css'

# Process groups
#
# Format:
#   Dictionary: group name -> [(host name, [process name, ...]), ...]
process_groups = {}

process_groups['all'] = [
        (hostname,      ['Radio',
                         'Vision',
                         'Soccer'
                        ]
        )
]

process_groups['support'] = [
        (hostname,      ['Radio',
                         'Vision'
                        ]
        )
]

# Order in which groups are shown.
# Since the order of items in a dictionary can't be controlled, this can be used to make
# the most important group show up on top.
group_order = ['all']

