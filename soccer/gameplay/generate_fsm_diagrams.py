import logging
import class_import
import fsm
import os, errno
import sys
import traceback

sys.path.append('../../run')


def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as e:
        if e.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise


logging.getLogger().setLevel(logging.INFO)

import main
main.init()

for behavior_type in ['skills', 'tactics', 'plays']:
    entries = class_import.recursive_import_classes('.', [behavior_type],
                                                    fsm.StateMachine)

    for entry in entries:
        try:
            klass = entry[1]
            module_path = entry[0]
            dirpath = 'diagrams/' + '/'.join(module_path[:-1])
            mkdir_p(dirpath)
            filepath = dirpath + "/" + klass.__name__
            klass().write_diagram_png(filepath)
            print("generated " + filepath)
        except Exception as e:
            logging.error("Error generating fsm diagram for behavior '" +
                          klass.__name__ + "':" + str(e))
            traceback.print_exc()
