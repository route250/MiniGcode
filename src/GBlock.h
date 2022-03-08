/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GBlock.h
 * Author: maeda
 *
 * Created on 2022年2月24日, 7:34
 */

#ifndef GBLOCK_H
#define GBLOCK_H

#include <stdio.h>
#include <iostream>

#include "Gunit.h"
#include "MGmath.h"

struct MGcommand {
    int no;
    string cmd;
};
    #define CMD_NOT_FOUND 0
    #define CMD_HELP 1
    #define CMD_EXEC 2
    #define CMD_SCAN 3
    #define CMD_SET_HOME 4
    #define CMD_SET_LIMIT 5
    #define CMD_SET_ZERO 6
    #define CMD_SHOW 7
    #define CMD_MOVE 8
    #define CMD_G02TOG01 10
    #define CMD_CCW 11
    #define CMD_DBGWAIT 12
    #define CMD_PON 13
    #define CMD_POFF 14
    #define CMD_CLEAR_ABORT 15
    #define CMD_ADJUST 16
    #define CMD_SCALE 118
    #define CMD_OFFSET 119

    const struct MGcommand MG_CMD_MAP[] = {
        {CMD_HELP,      "help" },
        {CMD_EXEC,      "exec" },
        {CMD_SCAN,      "scan" },
        {CMD_SET_HOME,  "sethome" },
        {CMD_SET_LIMIT, "setlimit" },
        {CMD_SET_ZERO,  "setzero" },
        {CMD_SHOW,      "show" },
        {CMD_MOVE,      "move" },
        {CMD_SCALE,     "scale" },
        {CMD_OFFSET,    "offset" },
        {CMD_G02TOG01,  "g02tog01" },
        {CMD_CCW,       "ccw" },
        {CMD_DBGWAIT,   "dbgwait" },
        {CMD_PON,       "pon" },
        {CMD_POFF,      "poff" },
        {CMD_CLEAR_ABORT,"clearabort" },
        {CMD_ADJUST,    "adjust" },
        { -1, "" }
    };
    
    int to_cmd( string aCmd );
    string from_cmd( int aCmd );
    
class GBlock {
private:
    int mLineNo;
    std::string mRawLine;
    address_t mAddress;
    code_t  mCode;
    pos_t mCurrentPos[NUM_MOTORS];
    pos_t mXYZ[NUM_MOTORS];
    feed_t mFeed;
    pos_t mIJK[NUM_MOTORS];
public:
    GBlock();
    GBlock( int aLineNo, std::string aRawLine, address_t aAddress,code_t aCode, pos_t aCurrentPos[] );
    GBlock( int aLineNo, std::string aRawLine, address_t aAddress,code_t aCode, pos_t aCurrentPos[], feed_t aFeed );
    GBlock( int aLineNo, std::string aRawLine, address_t aAddress,code_t aCode, pos_t aCurrentPos[], pos_t aXYZ[], feed_t aFeed );
    GBlock( int aLineNo, std::string aRawLine, address_t aAddress,code_t aCode, pos_t aCurrentPos[], pos_t aXYZ[], pos_t aIJK[], feed_t aFeed );
    GBlock(const GBlock& orig);
    virtual ~GBlock();
public:
    inline char getAddress() {
        return mAddress;
    }
    inline int getCode() {
        return mCode;
    }
    inline pos_t getXYZ( int idx ) {
        return mXYZ[idx];
    }
    inline pos_t getIJK( int idx ) {
        return mIJK[idx];
    }
    inline feed_t getFeed() {
        return mFeed;
    }
    inline pos_vct getCurrent() {
        pos_vct zPos(NUM_MOTORS);
        for( int i=0; i<NUM_MOTORS; i++ ) {
            zPos[i] = mCurrentPos[i];
        }
        return zPos;
    }
    inline pos_vct getTarget() {
        pos_vct zPos(NUM_MOTORS);
        for( int i=0; i<NUM_MOTORS; i++ ) {
            zPos[i] = mXYZ[i];
        }
        return zPos;
    }
    inline pos_vct getCenter() {
        pos_vct zPos(NUM_MOTORS);
        for( int i=0; i<NUM_MOTORS; i++ ) {
            zPos[i] = mIJK[i];
        }
        return zPos;
    }
    inline void getCurrent( pos_t aPos[] ) {
        for( int i=0; i<NUM_MOTORS; i++ ) {
            aPos[i] = mCurrentPos[i];
        }
    }
    inline void getCenter( pos_t aPos[] ) {
        for( int i=0; i<NUM_MOTORS; i++ ) {
            aPos[i] = mIJK[i];
        }
    }
    inline void getTarget( pos_t aPos[] ) {
        for( int i=0; i<NUM_MOTORS; i++ ) {
            aPos[i] = mXYZ[i];
        }
    }
    vector<string> getArgs();
    inline string getLine() {
        return mRawLine;
    }
    void out( std::ostream &aOut );
private:

public:


};

#endif /* GBLOCK_H */

