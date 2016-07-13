#include <SSLLog.hpp>
#include <RJLog.hpp>

#include <iostream>
#include <iomanip>
#include <fstream>

using namespace std;

SSLLog::SSLLog(string filename) {
    // ER-Force Example code

    ifstream in(filename.c_str(), ios_base::in | ios_base::binary);

    if (!in.is_open()) {
        cerr << "Error opening log file \"" << filename << "\"!" << endl;
    }

    FileHeader fileHeader;
    in.read((char*)&fileHeader, sizeof(fileHeader));
    // Log data is stored big endian, convert to host byte order
    fileHeader.version = be32toh(fileHeader.version);

    if (strncmp(fileHeader.name, DEFAULT_FILE_HEADER_NAME,
                sizeof(fileHeader.name)) == 0) {
        cout << "File format version " << fileHeader.version << " detected."
             << endl;

        if (fileHeader.version == DEFAULT_FILE_VERSION) {
            unsigned long refereePackets = 0;
            unsigned long visionPackets = 0;

            while (!in.eof()) {
                DataHeader dataHeader;
                in.read((char*)&dataHeader, sizeof(dataHeader));
                // Log data is stored big endian, convert to host byte order
                dataHeader.timestamp = be64toh(dataHeader.timestamp);
                dataHeader.messageType = be32toh(dataHeader.messageType);
                dataHeader.messageSize = be32toh(dataHeader.messageSize);

                char* data = new char[dataHeader.messageSize];
                in.read(data, dataHeader.messageSize);

                if (dataHeader.messageType == MESSAGE_SSL_VISION_2014) {
                    SSL_WrapperPacket packet;
                    if (packet.ParseFromArray(data, dataHeader.messageSize)) {
                        visionPackets++;
                        vision2014.push_back(
                            make_shared<SSL_WrapperPacket>(packet));
                        // cout << setiosflags(ios::fixed) << setprecision(3) <<
                        // (double) dataHeader.timestamp / 1e9
                        //          << ": Read " << visionPackets << " vision
                        //          packets and " << refereePackets << " referee
                        //          packets!" << endl;
                    } else {
                        cerr << "Error parsing 2014 vision packet!" << endl;
                    }
                } else if (dataHeader.messageType == MESSAGE_SSL_VISION_2010) {
                    // Legacy packets not currently supported
                    /*
                    SSL_WrapperPacket packet;
                    if (packet.ParseFromArray(data, dataHeader.messageSize)) {
                        visionPackets++;
                        //TODO: Push Packet onto stack
                        cout << setiosflags(ios::fixed) << setprecision(3) <<
                    (double) dataHeader.timestamp / 1e9
                                  << ": Read " << visionPackets << " vision
                    packets and " << refereePackets << " referee packets!" <<
                    endl;
                    } else {
                        cerr << "Error parsing 2010 vision packet!" << endl;
                    }
                    */
                } else if (dataHeader.messageType == MESSAGE_SSL_REFBOX_2013) {
                    SSL_Referee packet;
                    if (packet.ParseFromArray(data, dataHeader.messageSize)) {
                        refereePackets++;
                        referee.push_back(make_shared<SSL_Referee>(packet));
                        // cout << setiosflags(ios::fixed) << setprecision(3) <<
                        // (double) dataHeader.timestamp / 1e9
                        //          << ": Read " << visionPackets << " vision
                        //          packets and " << refereePackets << " referee
                        //          packets!" << endl;
                    } else {
                        cerr << "Error parsing vision packet!" << endl;
                    }
                }

                delete data;
            }

            cout << "Vision 2014 packets read: " << vision2014.size() << endl;
            cout << "Referee packets read: " << referee.size() << endl;
        }
    } else {
        cerr << "Corrupted Log File" << endl;
    }
}

SSLLog::SSLLog(RJLog rjLog) {}

RJLog SSLLog::toRJ() { return RJLog(); }
