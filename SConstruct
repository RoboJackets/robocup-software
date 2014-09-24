# Remove the build directory when cleaning
Clean('.', 'build')

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

# env.MergeFlags('-O2 -g3 -Wall -DNDEBUG') # optimized version
env.Append(CPPPATH = [Dir('#/common'), Dir('/usr/include/eigen3')])

# Variables
# Can't put this in build_dir because scons wants to read it before build_dir exists.
# I don't want to create build_dir for the sole purpose of hiding that error...
var_file = File('#/.scons_variables').abspath
vars = Variables(var_file)
vars.AddVariables(
	BoolVariable('profile', 'build for profiling', False)
)
vars.Update(env)
vars.Save(var_file, env)
Help('Variables (specify as name=value on the command line):')
Help('\n'.join(['  ' + x for x in vars.GenerateHelpText(env).splitlines()]))
Help('\n\nAvailable targets (specify on the command line):\n\n')

# All subsequent help text should describe targets

# Profiling
if env['profile']:
	print '*** Profiling enabled'
	env.Append(CPPFLAGS='-pg ', LINKFLAGS='-pg ')

# Qt
env['QT4DIR'] = '/usr'
env.Tool('qt4')
env.EnableQt4Modules(['QtCore', 'QtGui', 'QtNetwork', 'QtXml', 'QtOpenGL'])

# All executables need to link with the common library, which depends on protobuf
env.Append(LIBS=['common', 'protobuf', 'pthread', 'libGL'])

# Make a new environment for code that must be 32-bit
#env32 = env.Clone()

# Search paths for native code
env.Append(LIBPATH=[build_dir.Dir('common')])
env.Append(CPPPATH=[build_dir.Dir('common')])

# use clang++ instead of g++
# clang tends to have more friendly error messages and faster compile times
env.Replace(CXX='clang++')

# Fix to allow clang to show messages in color
import os
env['ENV']['TERM'] = os.environ['TERM']

Export({'env': env, 'cross_32bit': False})

def do_build(dir, exports={}):
	SConscript(dir + '/SConscript', exports=exports, variant_dir=build_dir.Dir(dir), duplicate=0)

do_build('common')

# Build everything else
for dir in ['logging', 'soccer', 'firmware', 'test']:
	do_build(dir)

