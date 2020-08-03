#include <QtCore/QCoreApplication>
#include <rj_common/qt_utils.hpp>

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
