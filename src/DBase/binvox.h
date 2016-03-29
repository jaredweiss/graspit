// adapted from code from http://www.cs.princeton.edu/~min/binvox/binvox.html
// http://www.cs.princeton.edu/~min/binvox/read_binvox.cc

#ifndef BINVOX_H
#define BINVOX_H

#include <QObject>

#include <string>
#include <fstream>
#include <iostream>
#include <stdlib.h>


class Binvox : public QObject
{
    Q_OBJECT


private:
    bool binvoxLoaded;

public:
    Binvox(QObject *parent = 0);
    ~Binvox(){}

    typedef unsigned char byte;

    int version;
    int depth, height, width;
    int size;
    byte *voxels;
    float tx, ty, tz;
    float scale;

    bool isBinvoxLoaded() { return binvoxLoaded; }
    int read_binvox(std::string filespec);

signals:
    
public slots:
    
};


#endif // BINVOX_H
