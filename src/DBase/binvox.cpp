// adapted from code from http://www.cs.princeton.edu/~min/binvox/binvox.html
// http://www.cs.princeton.edu/~min/binvox/read_binvox.cc

#include "debug.h"

#include "binvox.h"

Binvox::Binvox(QObject *parent) :
    QObject(parent)
{
     binvoxLoaded = false;
}

int Binvox::read_binvox(std::string filespec)
{

    std::ifstream *input = new std::ifstream(filespec.c_str(), std::ios::in | std::ios::binary);

    //
    // read header
    //
    std::string line;
    *input >> line;  // #binvox
    if (line.compare("#binvox") != 0) {
        DBGA("Binvox: Error reading binvox. First line reads [" << line << "] instead of [#binvox]");
        delete input;
        return 0;
    }
    *input >> version;
    DBGA("Binvox: .binvox using version: " << version);

    depth = -1;

    int done = 0;
    while(input->good() && !done) {
        *input >> line;
        if (line.compare("data") == 0) {
            done = 1;
        } else if (line.compare("dim") == 0) {
            *input >> depth >> height >> width;
        } else if (line.compare("translate") == 0) {
            *input >> tx >> ty >> tz;
        } else if (line.compare("scale") == 0) {
            *input >> scale;
        } else {
            DBGA("BinvoxUtil: Unrecognized keyword in binvox file [" << line << "], skipping...");
            char c;
            do { c = input->get(); } while(input->good() && (c != '\n')); // skip until end of line
        }
    }

    if (!done) {
        DBGA("Binvox: Error reading header");
        return 0;
    }
    if (depth == -1) {
        DBGA("Binvox: Missing dimensions in header");
        return 0;
    }

    size = width * height * depth;
    voxels = new byte[size];
    if (!voxels) {
        DBGA("Binvox: Error allocating memory to read voxel data");
        return 0;
    }

    //
    // read voxel data
    //
    byte value;
    byte count;
    int index = 0;
    int end_index = 0;
    int nr_voxels = 0;

    input->unsetf(std::ios::skipws);  // need to read every byte now (!)
    *input >> value;  // read the linefeed char

    while((end_index < size) && input->good()) {
        *input >> value >> count;

        if (input->good()) {
            end_index = index + count;
            if (end_index > size) {
                DBGA("Binvox: Mismatch between binvox dimensions and binary voxel data");
                return 0;
            }

            for (int i=index; i < end_index; i++) {
                voxels[i] = value;
            }

            if (value) {
                nr_voxels += count;
            }

            index = end_index;
        }
    }

    input->close();
    DBGA("Binvox: Read " << nr_voxels << " voxels. Done loading .binvox file");

    binvoxLoaded = true;
    return 1;

}
