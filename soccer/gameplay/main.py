import play_registry as play_registry_module
import playbook
import play
import fs_watcher
import class_import
import logging
import importlib
import traceback
import imp
import sys
import os
import constants

## soccer is run from the `run` folder, so we have to make sure we use the right path to the gameplay directory
GAMEPLAY_DIR = os.path.dirname(os.path.realpath(__file__))
PLAYBOOKS_DIR = GAMEPLAY_DIR + '/playbooks'


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
    import root_play as root_play_module
    _root_play = root_play_module.RootPlay()

    # init play registry
    global _play_registry
    _play_registry = play_registry_module.PlayRegistry()

    # load all plays
    play_classes = class_import.recursive_import_classes(GAMEPLAY_DIR, ['plays'], play.Play)
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
                    try:
                        module = importlib.import_module('.'.join(module_path))
                    except:
                        logging.error("Error reloading module '" + '.'.join(module_path) + "': e")
                        traceback.print_exc()
                        return

                    try:
                        play_class = class_import.find_subclasses(module, play.Play)[0]
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
                    try:
                        module = imp.reload(module)
                    except:
                        logging.error("Error reloading module '" + '.'.join(module_path) + "': e")
                        traceback.print_exc()
                        return

                    logging.info("reloaded module '" + '.'.join(module_path) + "'")

                    if is_play:
                        # re-register the new play class
                        # FIXME: this logic should go inside the play_registry
                        play_reg_node = _play_registry.node_for_module_path(module_path[1:])
                        play_reg_node.play_class = class_import.find_subclasses(module, play.Play)[0]
                        # _play_registry.modelReset.emit()

                    # kill currently-running stuff if needed
                    if not is_play:
                        _root_play.drop_current_play()
                        _root_play.drop_goalie_behavior()
                    elif is_play and root_play != None and root_play.__class__.__name__ == play_reg_node.play_class.__name__:
                        _root_play.drop_current_play()

                except Exception as e:
                    logging.error("EXCEPTION in file modified event: " + repr(e))
                    traceback.print_exc()
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

#loads the specified file_name from the playbooks folder
#isAbsolute should be passed as True if the file_name is an absolute path
def load_playbook(file_name, isAbsolute=False):
    global _play_registry
    _play_registry.load_playbook(playbook.load_from_file((PLAYBOOKS_DIR + '/' if not isAbsolute else '') + file_name))

#saves the playbook into the specified file_name in the playbooks folder
#isAbsolute should be passed as True if the file_name is an absolute path
def save_playbook(file_name, isAbsolute=False):
    global _play_registry
    playbook.save_to_file((PLAYBOOKS_DIR + '/' if not isAbsolute else '') + file_name, _play_registry.get_enabled_plays_paths());

## Called ~60times/sec by the C++ GameplayModule
def run():
    global _has_initialized
    if not _has_initialized:
        raise AssertionError("Error: must call init() before run()")

    try:
        if root_play() != None:
            root_play().spin()
    except:
        exc = sys.exc_info()[0]
        logging.error("Exception occurred in main.run(): " + str(exc) + "ignoring for now")
        traceback.print_exc()


_root_play = None
def root_play():
    return _root_play


_play_registry = None
def play_registry():
    global _play_registry
    return _play_registry

# returns the first robot in our robots with matching ID,
# or None if no robots have the given ID
def our_robot_with_id(ID):
    return next(iter([r for r in _our_robots if r.shell_id is ID]), None)

# set by the C++ GameplayModule
############################################################

_game_state = None
def game_state():
    global _game_state
    return _game_state
def set_game_state(value):
    global _game_state
    _game_state = value

_ball = None
def ball():
    global _ball
    return _ball
def set_ball(value):
    global _ball
    _ball = value

_our_robots = None
def our_robots():
    global _our_robots
    return _our_robots
def set_our_robots(value):
    global _our_robots
    root_play().robots = value
    _our_robots = value

_their_robots = None
def their_robots():
    global _their_robots
    return _their_robots
def set_their_robots(value):
    global _their_robots
    _their_robots = value

_system_state = None
def system_state():
    global _system_state
    return _system_state
def set_system_state(value):
    global _system_state
    _system_state = value

def set_field_constants(value):
    constants.setFieldConstantsFromField_Dimensions(value)
