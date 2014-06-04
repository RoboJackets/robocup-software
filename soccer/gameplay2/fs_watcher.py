from watchdog.observers import *
from watchdog.events import *
import time
import os.path
import imp


class FsWatcher(Observer):

    class FsEventHandler(FileSystemEventHandler):

        def __init__(self, root_path):
            super().__init__()
            self.root_path = root_path


        @property
        def root_path(self):
            return self._root_path
        @root_path.setter
        def root_path(self, value):
            self._root_path = value


        def notify(self, event_type, path):
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

                print("module '" + '.'.join(modpath) + "' " + event_type)

                parent_mod_dict = globals()
                for modname in modpath[:-1]:
                    parent_mod_dict = parent_mod_dict[modname].__dict__


                is_play = modpath[0] == 'plays'


                if event_type == 'modified':
                    imp.reload(parent_mod_dict[modpath[-1]])
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


        def on_modified(self, event):
            if isinstance(event, FileModifiedEvent):
                self.notify('modified', event.src_path)


        def on_created(self, event):
            if isinstance(event, FileCreatedEvent):
                self.notify('created', event.src_path)


        def on_deleted(self, event):
            if isinstance(event, FileDeletedEvent):
                self.notify('deleted', event.src_path)


    def __init__(self, path):
        super().__init__()
        handler = FsWatcher.FsEventHandler(path)
        self.schedule(handler, path, recursive=True)



if __name__ == '__main__':
    watcher = FsWatcher(path='./')
    watcher.start()

    try:
        while True:
            # print('loop')
            time.sleep(1)
    except KeyboardInterrupt:
        watcher.stop()
    watcher.join()
