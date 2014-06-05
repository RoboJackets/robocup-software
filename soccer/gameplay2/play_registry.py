
# The play registry keeps a tree of all plays in the 'plays' folder (and its subfolders)
# Our old system required programmatically registering plays into categories, but
# the new system just uses the filesystem hierarchy for this.
#
# The registry has methods for loading and unloading plays (for when files change on disk)
#
# It also tracks which plays are enabled
class PlayRegistry():

    def __init__(self):
        super().__init__()
        self._root = dict()


    @property
    def root(self):
        return self._root
    

    # returns a list of all plays in the tree that are currently enabled
    def get_enabled_plays(self):
        raise NotImplementedError()


    # pass in a module object that contains a Play subclass to add it to the registry
    def register_module(self, module):
        raise NotImplementedError()


    def __str__(self):
        raise NotImplementedError()



    class Node():

        @property
        def module_name(self):
            return self._module_name
        @module_name.setter
        def module_name(self, value):
            self._module_name = value

        @property
        def play_class(self):
            return self._play_class
        @play_class.setter
        def play_class(self, value):
            self._play_class = value
        
        @property
        def enabled(self):
            return self._enabled
        @enabled.setter
        def enabled(self, value):
            self._enabled = value


