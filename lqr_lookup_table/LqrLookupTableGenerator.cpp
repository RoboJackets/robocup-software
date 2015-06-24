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




    // cout <<"K = " << endl << K << endl;


    // A and B from MATLAB:
    // A =
    //    -8.5237         0         0
    //          0   -8.5237         0
    //          0         0   -8.5237
    //
    // B =
    //    -1.5874   -1.5874    1.8232    1.8232
    //     2.0318   -2.0318   -1.8232    1.8232
    //    32.2867   32.2867   32.2867   32.2867


    // Expected values from Simulink:
    // dPhiDt = 0
    // K =
    //    -0.4095    0.4918    0.1059
    //    -0.4095   -0.4918    0.1059
    //     0.4446   -0.4413    0.1047
    //     0.4446    0.4413    0.1047


    // Defines the maximum and minimum angular speeds the table should be capable of looking up
    // constexpr float MaxAngularSpeed = M_PI*3;
    // constexpr size_t LookupTableNumEntries = 500;

    // if (argc < 2) {
    //     cerr << "First param should be the output file path, aborting..." << endl;
    //     exit(1);
    // }

    // string filename(argv[1]);
    // ofstream outfile(filename);

    // cout << "Opened lookup table file: '" << filename << "'" << endl;
    // cout << "Writing entries...";

    // outfile << "// WARNING: This file is autogenerated." << endl;
    // outfile << "// Any changes you make will be overwritten next build." << endl << endl;
    // outfile << "#include \"LqrLookupTable.hpp\"" << endl;
    // outfile << endl;
    // outfile << endl;
    // outfile << "// Evenly spaced values corresponding to angular speeds in the range: [" << -MaxAngularSpeed << ", " << MaxAngularSpeed << "]" << endl;
    // outfile << "const float LqrLookupTable[" << LookupTableNumEntries << "][9] = {" << endl;

    // for (int i = 0; i < LookupTableNumEntries; ++i) {
    //     float dPhiDt = -MaxAngularSpeed + i*(2*MaxAngularSpeed)/LookupTableNumEntries;

    //     // Linearize the model by evaluating A with the given angular speed.
    //     RobotModel::AType A = model.A1 + model.A2*dPhiDt;

    //     // Calculate K using lqr
    //     KType K;
    //     constexpr float precision = 0.01;
    //     lqr<RobotModel::AType, RobotModel::BType, QType, RType, KType>(
    //         A, model.B, Q, Qf, R, precision, K);

    //     //  write this value of K to the file - formatted nicely and valid c with commas
    //     for (int r = 0; r < KType::RowsAtCompileTime; ++r) {
    //         outfile << "    ";
    //         for (int c = 0; c < KType::ColsAtCompileTime; ++c) {
    //             outfile << K.row(r).col(c) << ",";
    //         }
    //         outfile << endl;
    //     }
    //     outfile << endl;

    //     // cout << ".";

    //     // outfile << K << "," << endl << endl;
    // }
    // cout << endl;

    // outfile << "};" << endl;

    // outfile.close();
    // cout << "Done." << endl;
}
