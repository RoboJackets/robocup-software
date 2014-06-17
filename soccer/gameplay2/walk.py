import pkgutil
import importlib
import inspect

import plays


# module_path is a list of package names (in order) and must have 1+ items
# to import plays.offense and everything in it, call `recursive_import(['plays', 'offense'])`
def recursive_import(module_path):
    if not isinstance(module_path, list):
        raise AssertionError('module_path must be a list of strings')

    path = '/'.join(module_path)
    for loader, module_name, is_pkg in pkgutil.walk_packages([path]):
        new_module_path = module_path + [module_name]

        if is_pkg:
            recursive_import(new_module_path)
        else:
            module = __import__('.'.join(new_module_path))  # FIXME: module is the toplevel module, not the one we want

            # FIXME: this doesn't work sinc we don't import module contents yet
            for name, obj in inspect.getmembers(module):
                print('inspecting obj: ' + name)
                if inspect.isclass(obj) and issubclass(obj, plays.Play):
                    print("\tfound play class: " + obj.__name__)

            print('importing ' + '.'.join(new_module_path))


if __name__ == '__main__':
    recursive_import(['plays'])
