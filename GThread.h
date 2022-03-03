/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GThread.h
 * Author: maeda
 *
 * Created on 2022年2月26日, 0:11
 */

#ifndef GTHREAD_H
#define GTHREAD_H

#include <condition_variable>
#include <pthread.h>

#include "Gunit.h"
#include "MGmath.h"
#include "config.h"
#include "MotorCtl.h"

#define THCMD_00 0
#define CMD_START_BLOCK 1
#define CMD_TICK 2
#define CMD_END_BLOCK 9
#define THCMD_99 99

class GThread {
private:
    bool mDbg;
    long mCostTime_nanosec;
    long mCostWait_nanosec;
    long mMinimumWait_usec;

    std::mutex mMutex;
    std::condition_variable mCV;

    bool mRun;
    bool mActive;
    pthread_t mThread;

    pos_t mOffsetPos[NUM_MOTORS];
    
    string mCurrentLine;
    usec_t mCurrentTime;
    step_t mCurrentStep[NUM_MOTORS];
    
    bool mNext;
    string mNextLine;
    int mNextCmd;
    usec_t mNextTime;
    step_t mNextStep[NUM_MOTORS];
    
    MotorCtl mMotors[NUM_MOTORS];
    double mScale = 1.0;
    bool mAbort;
    usec_t mDbgWait = 5000;
    
public:
    GThread();
    GThread(const GThread& orig);
    virtual ~GThread();
public:
    bool configure( config& aConfig );
    void setDebug( bool b );
    void adjust(std::ostream& aOut);
    void start();
    void stop();
    bool isAbort();

    void poff();
    void pon();
    bool is_power();
    bool isDoneHoming();
    void scan_origin( int no, std::ostream& aOut );
    bool set_home( int no, std::ostream& aOut );
    bool set_limit( int no, std::ostream& aOut );
    bool set_zero( int no, std::ostream& aOut );
    bool move( int no, step_t aStep, std::ostream& aOut );

    bool send00( step_vct& aStep );
    bool send_start_block( string& aLine, step_vct& aStep );
    void send_step( usec_t aUsec, step_vct& aStep );
    bool send_end_block( step_vct& aStep );
    bool send99( step_vct& aStep );
    
    void show( std::ostream &aOut, double aScale, pos_vct& aOffset );
    pos_vct step_to_pos( step_vct& aStep, double aScale, pos_vct& aOffset );
    step_vct pos_to_step( pos_vct& aPos, double aScale, pos_vct& aOffset );
    step_t pos_to_step( int aNo, pos_t aPos, double aScale, pos_t aOffset );
    step_t getCurrentStep( int aNo );
    pos_t getCurrentPos( int aNo, double aScale, pos_t aOffset );
public:
    void thread_loop();
    bool put( int aCmd, string& aLine, usec_t aUsec, step_vct& aStep );
    bool putW( int aCmd, string& aLine, usec_t aUsec, step_vct& aStep );
    bool putW( int aCmd, usec_t aUsec, step_vct& aStep );
    bool get( int *aCmd, string *aLine, usec_t *aUsec, step_t aStep[] );
    void setScale( double aScale );
    void setDbgWait( usec_t aWait );
    usec_t getDbgWait();
};

#endif /* GTHREAD_H */

