#include <QtCore/QCoreApplication>

#include <rj_common/qt_utils.hpp>

QDir application_run_directory() {
    QDir run_dir(qApp->applicationDirPath());

// cd up out of the application bundle on OS X
#if defined(__APPLE__)
    runDir.cdUp();
    runDir.cdUp();
    runDir.cdUp();
#endif

    return run_dir;
}
