# Remove the build directory when cleaning
Clean('.', 'build')

# Where to put all executables
exec_dir = Dir('#/run')
Export('exec_dir')

env = Environment()

# C++ compiler
env.MergeFlags('-O2 -g3 -Wall -DQT_NO_KEYWORDS')
env.Append(CPPPATH = [Dir('#/common')])

# Qt
env['QT4DIR'] = '/usr'
env.Tool('qt4')
env.EnableQt4Modules(['QtCore', 'QtGui', 'QtNetwork', 'QtXml', 'QtOpenGL'])

# All executables need to link with the common library, which depends on protobuf
env.Append(LIBS=['common', 'protobuf'])

env32 = env.Clone()

# SoccSim must build 32-bit code, even on a 64-bit system, because PhysX is only
# available as 32-bit binaries.
env32.Append(CPPFLAGS='-m32')
env32.Append(LINKFLAGS='-m32')
env32.Append(LIBPATH=[Dir('build/common32'), Dir('SoccSim/lib/32-on-64/lucid_amd64')])
env32.Append(CPPPATH=[Dir('#/build/common32')])

# Ubuntu 10.04 32-bit compatibility:
# Copy the 32-bit libprotobuf to the run directory and add let the dynamic linker find it.
env32.Append(RPATH=[Literal('\\$$ORIGIN')])
env32.Install(exec_dir, 'SoccSim/lib/32-on-64/lucid_amd64/libprotobuf.so.5')

Export({'env': env32})
SConscript('common/SConscript', variant_dir='build/common32', duplicate=0)
SConscript('SoccSim/SConscript', variant_dir='build/SoccSim', duplicate=0)

# Build native code
env.Append(LIBPATH=[Dir('build/common')])
env.Append(CPPPATH=[Dir('#/build/common')])

Export({'env': env})
SConscript('common/SConscript', variant_dir='build/common', duplicate=0)

for dir in ['logging', 'radio', 'soccer']:
	SConscript('%s/SConscript' % dir, variant_dir='build/%s' % dir, duplicate=0)

# Build sslrefbox with its original makefile (no dependency checking)
env.Command('sslrefbox/sslrefbox', 'sslrefbox/Makefile', 'make -C sslrefbox')
