import root_play as root_play_module
import play_registry
import fs_watcher
import logging


# main init method for the python side of things
_has_initialized = False
def init():
    global _has_initialized
    if _has_initialized:
        logging.warn("main robocoup python init() method called twice - ignoring")
        return

    global _root_play
    _root_play = root_play_module.RootPlay()

    # TODO: setup play registry, fs watching

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
