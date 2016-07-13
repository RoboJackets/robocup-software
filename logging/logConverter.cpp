#include <RJLog.hpp>
#include <SSLLog.hpp>
#include <BSVLog.hpp>

#include <google/protobuf/io/zero_copy_stream_impl.h>

#include <QFile>
#include <QTextStream>

#include <vector>

#include <fcntl.h>
#include <time.h>
#include <iostream>

using namespace Packet;
using namespace std;

/**
 * Defines usage information for launching the Log-Viewer application
 * @param prog The name of the program
 */
void usage(const char* prog) {
    fprintf(stderr,
            "Usage: %s -i <filetype>%%<filepath> -o <filetype>%%<filepath>\n",
            prog);
    exit(1);
}

int main(int argc, char* argv[]) {
    if (argc > 5 || argc < 2) {
        usage(argv[0]);
    }

    string input;
    string output;

    for (int i = 1; i < argc; ++i) {
        if (i < argc - 1 && strcmp(argv[i], "-i") == 0) {
            input = argv[i + 1];
            ++i;
        } else if (i < argc - 1 && strcmp(argv[i], "-o") == 0) {
            output = argv[i + 1];
            ++i;
        } else {
            usage(argv[0]);
        }
    }

    cout << "input: " << input << " output: " << output << endl;

    if (input.empty() || output.empty()) {
        cerr << "Missing input or output arguments" << endl;
        usage(argv[0]);
    }

    int delimeter = input.find("%");

    if (delimeter == string::npos) usage(argv[0]);

    string inputType = input.substr(0, delimeter);
    string inputPath = input.substr(delimeter + 1, string::npos);

    delimeter = output.find("%");

    if (delimeter == string::npos) usage(argv[0]);

    string outputType = output.substr(0, delimeter);
    string outputPath = output.substr(delimeter + 1, string::npos);

    RJLog intermediate;

    if (inputType.compare("ssl") == 0) {
        cout << "Reading SSL Log" << endl;
        SSLLog ssl = SSLLog(inputPath);
        intermediate = ssl.toRJ();
    } else if (inputType.compare("rj") == 0) {
        cout << "Reading RoboJackets Log" << endl;
        intermediate = RJLog(inputPath);
    } else {
        cerr << "Input log format not supported" << endl;
        usage(argv[0]);
    }

    if (outputType.compare("bsv") == 0) {
        cout << "Writing BSV Log" << endl;
        BSVLog bsv(intermediate);

        string robotPath = "-robot.bsv";
        string framePath = "-frame.bsv";
        robotPath.insert(0,outputPath);
        framePath.insert(0,outputPath);

        cout << "writing output to " << robotPath << " and " << framePath << endl;

        bsv.write(robotPath, framePath);
    }
};