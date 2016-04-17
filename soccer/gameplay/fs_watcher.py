from watchdog.observers import *
from watchdog.events import *
import time
import os.path
import imp


## Watches the filesytem for changes and executes any registered callbacks
class FsWatcher(Observer):
    def __init__(self, path):
        super().__init__()
        handler = FsWatcher.FsEventHandler(self)
        self._subscribers = []
        self.schedule(handler, path, recursive=True)
        self.root_path = path

    ## the callback is passed event_type, module_path
    # where event_type is a string with the following possible values: 'modified', 'created', 'deleted'
    def subscribe(self, callback):
        self._subscribers.append(callback)

    ## removes a given subscriber
    def unsubscribe(self, callback):
        idx = self._subscribers.index(callback)
        del self._subscribers[idx]

    ## The directory to watch recursively
    @property
    def root_path(self):
        return self._root_path

    @root_path.setter
    def root_path(self, value):
        self._root_path = value

    # the handler calls _notify on its parent FsWatcher
    def _notify(self, event_type, path):
        if not isinstance(path, str):
            path = path.decode('utf-8')

        path = os.path.abspath(path)

        name, file_ext = os.path.splitext(path)
        if file_ext == '.py':
            # remove the prefix @root from @subpath
            root = os.path.abspath(self.root_path)
            subpath = name[len(root):]
            if len(subpath) > 0 and subpath[0] == '/': subpath = subpath[1:]

            # if we're watching /robocup/soccer/gameplay and within that, plays/my_play.py changes,
            # we extract ['plays', 'my_play'] into the @modpath list
            modpath = []
            while True:
                subpath, last_piece = os.path.split(subpath)
                modpath.insert(0, last_piece)
                if subpath == '' or subpath == '/': break

            # ignore changes to __init__.py files
            if modpath[-1] == '__init__':
                return

            logging.debug("module '" + '.'.join(modpath) + "' " + event_type)

            # call all callbacks
            for subscriber in self._subscribers:
                subscriber(event_type, modpath)

    class FsEventHandler(FileSystemEventHandler):
        def __init__(self, watcher):
            super().__init__()
            self._watcher = watcher

        def on_modified(self, event):
            if isinstance(event, FileModifiedEvent):
                self._watcher._notify('modified', event.src_path)

        def on_created(self, event):
            if isinstance(event, FileCreatedEvent):
                self._watcher._notify('created', event.src_path)

        def on_deleted(self, event):
            if isinstance(event, FileDeletedEvent):
                self._watcher._notify('deleted', event.src_path)


if __name__ == '__main__':

    def watcher_callback(event_type, modpath):
        print("CALLBACK")
        is_play = modpath[0] == 'plays'

        if event_type == 'modified':
            # imp.reload(parent_mod_dict[modpath[-1]])
            print('reloaded module: ' + '.'.join(modpath))

            if is_play:
                # FIXME: if it's the currently-running play, restart it
                pass

        elif event_type == 'deleted':
            if is_play:
                # FIXME: unload it from the play registry, otherwise, ignore it
                pass

        elif event_type == 'created':
            if is_play:
                __import__('.'.join(modpath))
                # FIXME: add it to the play registry
                # if it's not a play, we just ignore it (no need to load it, a play could load it if it wants to)
        else:
            raise AssertionError("Unknown event_type")

    watcher = FsWatcher(path='./')
    watcher.subscribe(watcher_callback)
    watcher.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        watcher.stop()
    watcher.join()
