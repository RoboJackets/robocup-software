# Tuple of all hosts that are listed on the status page.
# Also used for the host pulldown when adding.
#hosts = ['localhost', '127.0.0.1']
hosts = ['localhost']

# Path to the stylesheet
stylesheet = '/default.css'

# Process groups
#
# Format:
#   Dictionary: group name -> [(host name, [process name, ...]), ...]

process_groups = {
  #  'test_stopped': [('sting1', ['not_here'])]
  #  'test group': [('localhost', ['xeyes', 'test 1'])],

    'general': [('localhost', [
        'Simulator',
        'Vision (right)',
        'Vision (full)'])
    ],
    
    'yellow comp': [('localhost', [
        'Radio Yellow',
        'Motion Yellow (comp)',
        'Skills Yellow',
        'Logger Yellow',
        'Referee Yellow',
        'Soccer Yellow'])
	],
    
    'blue comp': [('localhost', [
        'Radio Blue',
        'Motion Blue (comp)',
        'Skills Blue',
        'Logger Blue',
        'Referee Blue',
        'Soccer Blue'])
    ],
    
    'yellow sim': [('localhost', [
        'Motion Yellow (sim)',
        'Skills Yellow',
        'Logger Yellow',
        'Referee Yellow',
        'Soccer Yellow'])
    ],
    
    'blue sim': [('localhost', [
        'Motion Blue (sim)',
        'Skills Blue',
        'Logger Blue',
        'Referee Blue',
        'Soccer Blue'])
    ],
}
group_order = ['general', 'yellow comp', 'blue comp', 'yellow sim', 'blue sim']
