
## Load and save playbook files
# A playbook is a simple text file with a '.pbk' extension that lists
# plays to enable.  The entries are the paths to the play's python module,
# NOT the actual play name.  See playbooks/example.pbk for an example.

def load_from_file(playbook_file):
    plays = []

    with open(playbook_file, 'r') as f:
        for play in f:
            play = play.strip()
            if not play:
                plays.append(play.split('/'))

    return plays
