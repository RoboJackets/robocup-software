
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
        categories = self.root

        # iterate up to the last one (the last one is just an underscored,
        # lowercased version of the play's name and we don't display it in the tree)
        for module in module_path[:-1]:
            if module not in categories:
                categories[module] = dict()
            categories = categories[module]

        categories[play_class.__name__] = PlayRegistry.Node(module_path[-1], play_class)


    def delete(self, module_path, play_class):
        dictStack = [self.root]
        try:
            for module in module_path[:-1]:
                dictStack.append(dictStack[-1][module])
            
            # remove the play
            del dictStack[-1][play_class.__name__]

            # remove any categories where this play was the only entry
            dictStack.reverse()
            for idx, aDict in enumerate(dictStack[:-1]):
                if len(aDict) == 0:
                    del dictStack[idx+1][module_path[-2 - idx]]
        except KeyError:
            raise KeyError("Unable to find the specified play")


    # returns a list of all plays in the tree that are currently enabled
    def get_enabled_plays(self):
        return [node.play_class for node in self if node.enabled]


    # iterates over all of the Nodes registered in the tree
    def __iter__(self):
        def _recursive_iter(aDict):
            for key, value in aDict.items():
                if isinstance(value, PlayRegistry.Node):
                    yield value
                else:
                    yield from _recursive_iter(value)
        return _recursive_iter(self.root)


    def __contains__(self, play_class):
        for node in self:
            if node.play_class == play_class:
                return True
        return False


    def __str__(self):
        def _dict_str(aDict, indent):
            desc = ""
            for key, value in aDict.items():
                if isinstance(value, PlayRegistry.Node):
                    desc += "    " * indent
                    desc += str(value)
                else:
                    desc += "    " * indent + key + ':' + '\n'
                    desc += _dict_str(value, indent + 1)
                desc += '\n'
            return desc[:-1]    # delete trailing newline

        return "PlayRegistry:\n-------------\n" + _dict_str(self.root, 0)


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


        def __str__(self):
            return self.play_class.__name__ + " " + ("[ENABLED]" if self.enabled else "[DISABLED]")
