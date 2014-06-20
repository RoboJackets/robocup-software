import root_play as root_play_module
import play_registry as play_registry_module
import play, skill
import fs_watcher
import recursive_import
import logging
import importlib
import imp
import sys


# soccer is run from the `run` folder, so we provide a relative path to where the python files live
GAMEPLAY_DIR = '../soccer/gameplay2'


# main init method for the python side of things
_has_initialized = False
def init():
    # by default, the logger only shows messages at the WARNING level or greater
    logging.getLogger().setLevel(logging.INFO)

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
        # the top-level folders we care about watching
        autoloadables = ['plays', 'skills', 'tactics', 'evaluation']

        if module_path[0] in autoloadables:
            logging.info('.'.join(module_path) + " " + event_type)

            is_play = module_path[0] == 'plays'

            if event_type == 'created':
                if is_play:
                    # we load the module and register the play class it contains with the play registry
                    # this makes it automatically show up in the play config tab in the gui
                    module = importlib.import_module('.'.join(module_path))
                    try:
                        play_class = recursive_import.find_subclasses(module, play.Play)[0]
                        _play_registry.insert(module_path[1:], play_class) # note: skipping index zero of module_path cuts off the 'plays' part
                    except IndexError as e:
                        # we'll get an IndexError exception if the module didn't contain any Plays
                        # FIXME: instead, we should unload the module and just log a warning
                        raise Exception("Error: python files within the plays directory must contain a subclass of play.Play")
            elif event_type == 'modified':
                try:
                    # reload the module
                    containing_dict = sys.modules
                    for modname in module_path[:-1]:
                        containing_dict = containing_dict[modname].__dict__
                    module = containing_dict[module_path[-1]]
                    module = imp.reload(module)

                    # re-register the new play class
                    # FIXME: this logic should go inside the play_registry
                    play_reg_node = _play_registry.node_for_module_path(module_path[1:])
                    play_reg_node.play_class = recursive_import.find_subclasses(module, play.Play)[0]
                    # _play_registry.modelReset.emit()

                    logging.info("reloaded module '" + '.'.join(module_path) + "'")

                    # kill currently-running stuff if needed
                    if not is_play:
                        _root_play.drop_current_play()
                        _root_play.drop_goalie_behavior()
                    elif is_play and root_play != None and root_play.__class__.__name__ == play_reg_node.play_class.__name__:
                        _root_play.drop_current_play()

                except Exception as e:
                    logging.error("EXCEPTION in file modified event: " + repr(e))
                    raise e
            elif event_type == 'deleted':
                if is_play:
                    node = _play_registry.node_for_module_path(module_path[1:])
                    if _root_play.play != None and _root_play.play.__class__.__name__ == node.play_class.__name__:
                        _root_play.drop_current_play()

                    _play_registry.delete(module_path[1:])
                else:
                    _root_play.drop_current_play()
                    _root_play.drop_goalie_behavior()
            else:
                raise AssertionError("Unknown FsWatcher event type: '" + event_type + "'")


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



# set by the C++ GameplayModule
############################################################

_game_state = None
@property
def game_state(self):
    return self._game_state
@game_state.setter
def game_state(self, value):
    self._game_state = value

_ball = None
@property
def ball(self):
    return self._ball
@ball.setter
def ball(self, value):
    self._ball = value


