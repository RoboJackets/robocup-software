
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
    

    # the module path is a list
    # for a demo play called RunAround, module_path = ['demo', 'run_around']
    # (note that we left out 'plays' - every play is assumed to be in a descendent module of it)
    def insert(self, module_path, play_class):
        category = self.root

        # iterate up to the last one (the last one is just an underscored,
        # lowercased version of the play's name and we don't display it in the tree)
        for module in module_path[:-1]:
            if 


    # returns a list of all plays in the tree that are currently enabled
    def get_enabled_plays(self):
        raise NotImplementedError()


    # pass in a module object that contains a Play subclass to add it to the registry
    def register_module(self, module):
        raise NotImplementedError()


    def __str__(self):
        raise NotImplementedError()



    class Node():

        def __init__(self, module_name, play_class):
            self._module_name = module_name
            self._play_class = play_class
            self._enabled = True


        @property
        def module_name(self):
            return self._module_name

        @property
        def play_class(self):
            return self._play_class
        
        @property
        def enabled(self):
            return self._enabled
        @enabled.setter
        def enabled(self, value):
            self._enabled = value
