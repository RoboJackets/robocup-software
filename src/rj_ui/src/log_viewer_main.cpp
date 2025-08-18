#include <rj_ui/log_viewer.hpp>

#include <algorithm>
#include <cstdio>

#include <rj_protos/LogFrame.pb.h>
#include <ui_LogViewer.h>

#include <QTime>
#include <QTimer>
#include <vector>

#include <QApplication>
#include <QFile>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include <fcntl.h>

using namespace std;
using namespace boost;
using namespace Packet;
using namespace google::protobuf::io;

void usage(const char* prog) {
    fprintf(stderr, "Usage: %s <filename.log>\n", prog);
    exit(1);
}

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    if (argc != 2) {
        usage(argv[0]);
    }

    LogViewer win;

    win.readFrames(argv[1]);
    win.showMaximized();

    return QApplication::exec();
}