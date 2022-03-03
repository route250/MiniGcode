/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GReader.h
 * Author: maeda
 *
 * Created on 2022年2月24日, 6:26
 */

#ifndef GREADER_H
#define GREADER_H

#include <fstream>
#include <string>

#include "Gunit.h"
#include "GBlock.h"

class GReader {
private:
    std::string mFileName;
    bool is_open = false;
    std::ifstream mIn;
    std::string mBuffer;
    int mPos;
    int mLineNo;
    pos_t mNextPos[3];
    pos_t mNextCenter[3];
    pos_t mFeed;
public:
    GReader();
    GReader( std::string aFileName );
    //GReader(const GReader& orig);
    virtual ~GReader();
public:
    bool open();
    int read();
    void close();
    GBlock nextBlock( pos_vct& aCurrentPos );
private:
    void stripLine();
    void readLine();
    int readInt(int p);
    pos_t readPosRef( pos_t i, int p);
    pos_t readPos( int p);
};

#endif /* GREADER_H */

