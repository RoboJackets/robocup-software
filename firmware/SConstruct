# Remove the build directory when cleaning
Clean('.', 'build-scons')

# Where to put all executables
exec_dir = Dir('#/run')
Export('exec_dir')

build_dir = Dir('#/build')
Export('build_dir')

env = Environment(tools=['default', 'textfile'])  

# http://www.scons.org/wiki/GoFastButton
env.Decider('MD5-timestamp')
SetOption('max_drift', 1)
#SetOption('implicit_cache', 1)
env.SourceCode(".", None)

# Keep a plain environment for cross-compiling later
env_base = env.Clone()
Export('env_base')

# C++ compiler
env.MergeFlags('-Wall -g -Wno-unused-function -Wno-reorder -Wno-gnu -std=c++11')  # debug version - don't use for competition use

# Newer compilers have deprecated the 'register' keyword
# To suppress warnings about 'register' being used in some of our dependencies, we pass this flag
# However, older compilers don't recognize this flag and give us warnings for trying to suppress non-existant warnings
# we'll leave this commented out for now and add it back once we're all using a newer compiler
# env.MergeFlags('-Wno-deprecated-register')

Export({'env': env, 'cross_32bit': False})

SConscript('./SConscript', variant_dir=build_dir, duplicate=0)
