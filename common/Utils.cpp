#include "Utils.hpp"

#include <stdlib.h>

using namespace std;

#ifdef __GNUC__
// This function uses type_info::name which returns an implementation-dependent
// string.
// This implementation expects gcc's convention as of 4.3.3:
//   A class not in a namespace is <n><name> where <n> in the number of
//   characters in <name>.
//   A class in a namespace is N<a><namespace><b><namespace>...<n><classname>E,
//   where each namespace name is preceeded by the number of characters in that
//   name.
QString typeName(const std::type_info& info) {
    const char* name = info.name();
    QString out;

    if (name[0] == 'N') {
        // In a namespace: format is N<n><n characters>...
        ++name;
    }

    while (*name && *name != 'E') {
        char* next;
        int len = strtol(name, &next, 10);
        if (!out.isEmpty()) {
            out += "::";
        }
        out += QString::fromLatin1(next, len);
        name = next + len;
    }

    return out;
}
#endif

QString className(const std::type_info& info) {
    // Use the class name, which is the part after the last colon.
    QString fullName = typeName(info);
    int pos = fullName.lastIndexOf(':');
    if (pos >= 0) {
        return fullName.mid(pos + 1);
    } else {
        // Not in a namespace
        return fullName;
    }
}

QDir ApplicationRunDirectory() {
    QDir runDir(qApp->applicationDirPath());

// cd up out of the application bundle on OS X
#if defined(__APPLE__)
    runDir.cdUp();
    runDir.cdUp();
    runDir.cdUp();
#endif

    return runDir;
}
