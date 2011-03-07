Import('*')

Help('\nFirmware targets:\n')

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

# Find the Xilinx tools
import os
from os.path import *
home = os.getenv('HOME')
xilinx_versions = {}
paths = ['/opt', home]
for start in paths:
	# Look for a Xilinx directory
	path = join(start, 'Xilinx')
	if isdir(path):
		# Find all versions installed in this directory
		for entry in os.listdir(path):
			try:
				ver = float(entry)
			except ValueError:
				continue
			ver_path = join(path, entry)
			xpath = join(ver_path, 'ISE_DS')
			if isdir(ver_path) and isdir(xpath):
				xilinx_versions[xpath] = ver

xilinx = Environment()
if xilinx_versions:
	# Find the highest of all installed versions
	xilinx_path = max(xilinx_versions, key=xilinx_versions.get)

	import platform
	machine = platform.machine()
	if machine == 'x86_64':
		platform_dir = 'lin64'
	elif machine == 'x86':
		platform_dir = 'lin'
	else:
		raise NotImplementedError('Xilinx tools only supported for 32/64-bit x86 Linux')

	xilinx.AppendENVPath('PATH', join(xilinx_path, 'ISE/bin/', platform_dir))
	xilinx.AppendENVPath('XILINXD_LICENSE_FILE', join(home, '.Xilinx/Xilinx.lic'))

# Scanner to find Verilog includes
import re
include_re = re.compile(r'^`include\s+"(\S+)"', re.M)

def vfile_scan (node, env, path):
	contents = node.get_contents()
	return include_re.findall(contents)

xilinx.Append(SCANNERS=Scanner(function=vfile_scan, skeys=['.v'], recursive=True))

Export('xilinx')

# Subdirectories
SConscript('base_2009/SConscript')
SConscript('speedgate/SConscript')
SConscript('robot_2011/SConscript')