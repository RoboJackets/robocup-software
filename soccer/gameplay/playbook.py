def load_from_file(playbook_file):
    plays = []

    with open(playbook_file, 'r') as f:
        for play in f:
            plays.append(play.strip().split('/'))

    return plays

def save_to_file(playbook_file, list_of_plays):
    with open(playbook_file, 'w') as f:
        for play in list_of_plays:
            f.write('/'.join(play) + '\n')
