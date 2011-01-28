Import('env', 'exec_dir', 'cross_32bit')

SConscript('protobuf/SConscript')

srcs = []
srcs.extend(Glob('protobuf/*.pb.cc'))
srcs.extend(Glob('*.cpp'))
srcs.extend(Glob('Geometry2d/*.cpp'))

env.Library('common', srcs)

# Make common a Python package
#env.Textfile('__init__.py', source=[''])

# Write the git commit hash to a source file for versioning logs
import git

repo = git.Repo(Dir('#').path)

env.Textfile('git_version.h',
	['static const char git_version_hash[] = "@hash@";',
	 'static bool git_version_dirty = @dirty@;',
	 ''],
	SUBST_DICT=(
		('@hash@', repo.head.commit.hexsha),
		('@dirty@', str(repo.is_dirty()).lower())
	)
)

env.Textfile('git_version.py',
	["git_version_hash = '@hash@'",
	 'git_version_dirty = @dirty@',
	 ''],
	SUBST_DICT=(
		('@hash@', repo.head.commit.hexsha),
		('@dirty@', repo.is_dirty())
	)
)
