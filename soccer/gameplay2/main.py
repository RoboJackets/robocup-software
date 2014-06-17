import root_play as root_play_module
import play_registry as play_registry_module
import play, skill
import fs_watcher
import recursive_import
import logging

import plays.line_up    # FIXME: remove


GAMEPLAY_DIR = '../soccer/gameplay2'



# main init method for the python side of things
_has_initialized = False
def init():
    global _has_initialized
    if _has_initialized:
        logging.warn("main robocoup python init() method called twice - ignoring")
        return

    # init root play
    global _root_play
    _root_play = root_play_module.RootPlay()

    # init play registry
    global _play_registry
    _play_registry = play_registry_module.PlayRegistry()

    # load all plays
    play_classes = recursive_import.recursive_import_classes(GAMEPLAY_DIR, ['plays'], play.Play)
    for entry in play_classes:
        # keep in mind that @entry is a tuple
        mod_path = entry[0][1:]
        _play_registry.insert(mod_path, entry[1])


    # this callback lets us do cool stuff when our python files change on disk
    def fswatch_callback(event_type, module_path):
        print('.'.join(module_path) + " " + event_type)
        # TODO: implement for real

    # start up filesystem-watching
    watcher = fs_watcher.FsWatcher(GAMEPLAY_DIR)
    watcher.subscribe(fswatch_callback)
    watcher.start()

    _has_initialized = True


def run():
    global _has_initialized
    if not _has_initialized:
        raise AssertionError("Error: must call init() before run()")

    if root_play() != None:
        root_play().run()


_root_play = None
def root_play():
    return _root_play


_play_registry = None
def play_registry():
    global _play_registry
    return _play_registry
