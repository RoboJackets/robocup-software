Import('*')

# AVR cross-compiler
avr = Environment()
avr.Replace(CC='avr-gcc')
avr.Append(CFLAGS='-Os -Wall -std=gnu99')
avr.Append(BUILDERS={'Binary': Builder(
	action='avr-objcopy -O binary $SOURCE $TARGET',
	suffix='.bin',
	src_suffix='.elf')})

Export('avr')

# ARM cross-compiler
arm = SConscript('arm_toolchain/SConscript', {'env': env_base})
Export('arm')

SConscript('base_2009/SConscript')
SConscript('speedgate/SConscript')
SConscript('robot_2011/SConscript')
