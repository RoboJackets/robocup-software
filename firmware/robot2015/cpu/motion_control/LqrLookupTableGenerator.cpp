#include "OfflineLqrController.hpp"
#include "TunableRobotParameters.hpp"
#include "RobotModel.hpp"

#include <iostream>
#include <sstream>

using namespace std;

// In order to pass matrices from C++ code into an octave script, they have to
// be formatted appropriately.
template<typename Matrix>
ostream &printOctaveFormattedMatrix(ostream &stream, const Matrix &matrix) {
    stream << "[";
    for (int i = 0; i < Matrix::RowsAtCompileTime; ++i) {
        for (int j = 0; j < Matrix::ColsAtCompileTime; ++j) {
            if (j != 0) stream << ", ";
            stream << matrix(i, j);
        }
        if (i != Matrix::RowsAtCompileTime-1) {
            stream << ";" << endl;
        }
    }
    stream << "]";
    return stream;
}

// This program generates a lookup table of LQR gains according to the
// parameters in TunableRobotParameters.cpp.  Unfortunately, we couldn't find a
// native C++ library that does lqr(), so we call out to octave (sort of an
// open-source MATLAB) to do the actual work.  Eventually, we'd like to redo
// this using just C++ so the implementation is cleaner and doesn't depend on
// octave.
int main(int argc, char **argv) {
    if (argc < 2) {
        cerr << "The first parameter to '" << argv[0] << "' should be the output .cpp file path" << endl;
        return 1;
    }
    string outputFilename(argv[1]);

    // make sure the output directory exists
    string mkdirCmd = "mkdir -p $(dirname '" + outputFilename + "')";
    int mkdirStatus = system(mkdirCmd.c_str());
    if (mkdirStatus != 0) {
        cerr << "Unable to create lookup table output directory, aborting..." << endl;
        return 1;
    }

    // Build an octave script to generate the LQR gains and write them to a file
    // clang-format off
    stringstream octaveScript;
    octaveScript << "octave --quiet --eval \"" << endl;
    octaveScript << "pkg load control;" << endl;
    octaveScript << "fid = fopen('" << outputFilename << "', 'w');" << endl << endl;

    octaveScript << "A1 = ";
    printOctaveFormattedMatrix(octaveScript, Robot2015SystemModel.A1);
    octaveScript << ";" << endl << endl;

    octaveScript << "A2 = ";
    printOctaveFormattedMatrix(octaveScript, Robot2015SystemModel.A2);
    octaveScript << ";" << endl << endl;

    octaveScript << "B = ";
    printOctaveFormattedMatrix(octaveScript, Robot2015SystemModel.B);
    octaveScript << ";" << endl << endl;

    octaveScript << "Q = ";
    printOctaveFormattedMatrix(octaveScript, Robot2015LqrQ);
    octaveScript << ";" << endl << endl;

    octaveScript << "R = ";
    printOctaveFormattedMatrix(octaveScript, Robot2015LqrR);
    octaveScript << ";" << endl << endl;

    octaveScript << "MinRotVel = " << Robot2015LqrLookupTableMinRotVel << ";" << endl;
    octaveScript << "MaxRotVel = " << Robot2015LqrLookupTableMaxRotVel << ";" << endl;
    octaveScript << "NumEntries = " << Robot2015LqrLookupTableNumEntries << ";" << endl;
    octaveScript << endl;

    octaveScript << "fprintf(fid, '// Do not modify this file, it is automatically generated at compile time\\n');" << endl;
    octaveScript << "fprintf(fid, '#include \\\"Robot2015LqrLookupTable.hpp\\\"\\n');" << endl;
    octaveScript << "fprintf(fid, '\\n');" << endl;
    octaveScript << "fprintf(fid, 'const float Robot2015LqrLookupTableValues[] = {\\n');" << endl;

    octaveScript << "for entryNum=0:(NumEntries-1)" << endl;
    octaveScript << "  rotVel = MinRotVel + entryNum*(MaxRotVel - MinRotVel)/(NumEntries-1);" << endl;
    octaveScript << "  fprintf(fid, '    // rot vel = %f\\n', rotVel);" << endl;
    octaveScript << "  A = A1 + A2*rotVel;" << endl;
    octaveScript << "  [K, S, E] = lqr(A, B, Q, R);" << endl;
    octaveScript << "  for i=1:rows(K)" << endl;
    octaveScript << "    fprintf(fid, '   ');" << endl;
    octaveScript << "    for j=1:columns(K)" << endl;
    octaveScript << "      fprintf(fid, ' %f,', K(i, j));" << endl;
    octaveScript << "    end" << endl;
    octaveScript << "    fprintf(fid, '\\n');" << endl;
    octaveScript << "  end" << endl;
    octaveScript << "end" << endl;
    octaveScript << "fprintf(fid, '};\\n');" << endl;

    octaveScript << "fclose(fid);" << endl;
    octaveScript << "\"" << endl;
    // clang-format on

    // cout << "Octave script:" << endl << octaveScript.str() << endl;

    // Execute the octave script and return 0 on success
    int status = system(octaveScript.str().c_str());
    if (status != 0) {
        cerr << "Error running octave script to generate lqr lookup table" << endl;
    }
    return status;
}
