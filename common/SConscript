Import('*')

SConscript('protobuf/SConscript')

srcs = []
srcs.extend(Glob('protobuf/*.pb.cc'))
srcs.extend(Glob('*.cpp'))
srcs.extend(Glob('Geometry2d/*.cpp'))

env.Library('common', srcs)
