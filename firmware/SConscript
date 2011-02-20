Import('*')

env.AppendENVPath('PATH', Dir('install/bin').abspath)

# variant_dir is not necessary because it is inherited from the top-level SConstruct
SConscript('arm_toolchain/SConscript')
SConscript('base_2009/SConscript')

