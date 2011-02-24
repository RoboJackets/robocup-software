Import('*')

# AVR cross-compiler
avr = Environment()
avr.Replace(CC='avr-gcc')
avr.Append(CFLAGS='-Os -Wall -std=gnu99')

def generate_bin(source, target, env, for_signature):
	return 'avr-objcopy -O binary %s %s'%(source[0], target[0])

avr.Append(BUILDERS={'Binary': Builder(
	generator=generate_bin,
	suffix='.bin',
	src_suffix='.elf')})

Export('avr')

# ARM cross-compiler
#
# Use the Robocup toolchain.
# We can't use Maverick's arm-linux-gnueabi toolchain because it expects glibc and
# an armv7-a CPU.
arm = env_base.Clone()
arm.AppendENVPath('PATH', Dir('arm_toolchain/install/bin').abspath)
arm.Append(CROSS='arm-elf-')
arm.Replace(CC='${CROSS}gcc')

def generate_bin(source, target, env, for_signature):
	return '${CROSS}objcopy -O binary %s %s'%(source[0], target[0])

arm.Append(BUILDERS={'Binary': Builder(
	generator=generate_bin,
	suffix='.bin',
	src_suffix='.elf')})

Export('arm')

# variant_dir is not necessary because it is inherited from the top-level SConstruct
SConscript('arm_toolchain/SConscript')
SConscript('base_2009/SConscript')
SConscript('speedgate/SConscript')
SConscript('robot_2011/SConscript')

