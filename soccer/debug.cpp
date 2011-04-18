#include "debug.hpp"

#include <stdlib.h>
#include <assert.h>
#include <execinfo.h>
#include <bfd.h>

#define HAVE_DECL_BASENAME 1
#include <demangle.h>

static bfd *abfd;
static asymbol **syms;
static asection *section;

void debugInit(const char *filename)
{
	bfd_init();
	
	abfd = bfd_openr(filename, "elf64-x86-64");
	if (!abfd)
	{
		fprintf(stderr, "bfd_openr failed: %m\nSelf-debugging not available\n\n");
		return;
	}
	
	assert(bfd_check_format(abfd, bfd_object));
	assert(bfd_get_file_flags(abfd) & HAS_SYMS);
	
	long storage = bfd_get_symtab_upper_bound(abfd);
	assert(storage > 0);
	syms = (asymbol **)malloc(storage);
	assert(syms);
	int symcount = bfd_canonicalize_symtab(abfd, syms);
	assert(symcount > 0);
	printf("Debug: %d symbols\n", symcount);
	
	// Assume all backtraces will stay in .text
	// This is true as long as we don't use self-modifying or generated code...
	section = bfd_get_section_by_name(abfd, ".text");
}

QStringList debugTrace(const google::protobuf::RepeatedField<google::protobuf::uint64> &trace)
{
	QStringList strs;
	
	if (!abfd)
	{
		return strs;
	}
	
	for (int i = 0; i < trace.size(); ++i)
	{
		bfd_vma pc = (bfd_vma)trace.Get(i);
		QString s;
		s.sprintf("0x%lx", pc);
		
		bfd_vma vma = bfd_get_section_vma(abfd, section);
		bfd_size_type size = bfd_get_section_size(section);
		if (pc < vma || pc >= vma + size)
		{
			continue;
		}

		const char *filename = 0;
		const char *function = 0;
		unsigned int line = 0;
		if (!bfd_find_nearest_line(abfd, section, syms, pc - vma, &filename, &function, &line) ||
			!filename || !function)
		{
			continue;
		}
		
		bool found;
		do
		{
			found = bfd_find_inliner_info(abfd, &filename, &function, &line);
		} while (found);
		
		if (function)
		{
			char *demangled = bfd_demangle(abfd, function, DMGL_ANSI | DMGL_PARAMS);
			strs.append(demangled ? demangled : function);
			if (demangled)
			{
				free(demangled);
			}
		} else {
			const char *file = strrchr(filename, '/');
			if (file)
			{
				++file;
			} else {
				file = filename;
			}
			strs.append(QString("%1:%2").arg(QString(file), QString::number(line)));
		}
	}
	
	return strs;
}
