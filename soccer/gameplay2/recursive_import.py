import pkgutil
import importlib
import inspect


# Recursively imports all subclasses of a given class from a given directory
# 
# module_path is a list of package names (in order) and must have 1+ items
# to import plays.offense and everything in it, call `recursive_import_classes(['plays', 'offense'])`
#
# Returns a list of tuples of the form (module_path, class) where class
# is a subclass of parent_class and module_path is a
# list of module names telling where @class came from.
# This is used in the system for importing plays and behaviors.
# An example entry in the returned list could be: `(['plays', 'offense', 'mighty_might'], plays.offense.MightyMight)`
def recursive_import_classes(module_path, parent_class):
    classes = []

    if not isinstance(module_path, list):
        raise AssertionError('module_path must be a list of strings')

    path = '/'.join(module_path)
    for loader, module_name, is_pkg in pkgutil.walk_packages([path]):
        new_module_path = module_path + [module_name]

        if is_pkg:
            classes.append(recursive_import_classes(new_module_path, parent_class))
        else:
            module = importlib.import_module('.'.join(new_module_path))
            for name, obj in inspect.getmembers(module):
                if inspect.isclass(obj) and issubclass(obj, parent_class) and obj != parent_class:
                    print('module: ' + str(module))
                    classes.append((new_module_path, obj))

    return classes


if __name__ == '__main__':
    # import play
    # results = recursive_import_classes(['plays'], play.Play)
    import skill
    results = recursive_import_classes(['skills'], skill.Skill)
    print("imports:")
    for res in results:
        print("\t" + str(res))

