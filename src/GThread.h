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
#include "MGaxis.h"

#define QLEN 100
#define THCMD_00 0
#define CMD_START_BLOCK 1
#define CMD_TICK 2
#define CMD_END_BLOCK 9
#define THCMD_99 99

class GThread {
private:
    bool mDbg = false;
    long mCostTime_nanosec = 0;
    long mCostWait_nanosec = 0;
    long mMinimumWait_nanosec = 0;

    std::mutex mMutex;
    std::condition_variable mCV;

    bool mRun = false;
    bool mActive = false;
    pthread_t mThread;

    string mCurrentLine;
    usec_t mCurrentTime;
    
    bool mNext;
    string mNextLine;
    int mNextCmd = 0;
    usec_t mNextTime = 0;
    step_t mNextStep[NUM_MOTORS];
    
    int mQ1len = 0;
    string mQ1_NextLine[QLEN];
    int mQ1_NextCmd[QLEN];
    usec_t mQ1_NextTime[QLEN];
    step_t mQ1_NextStep[QLEN*NUM_MOTORS];
    
    int mCmdSend_length = 0;
    string mCmdSend_line[QLEN];
    int mCmdSend_cmd[QLEN];
    usec_t mCmdSend_time[QLEN];
    step_t mCmdSend_step[QLEN*NUM_MOTORS];

    MGaxis mAxis[NUM_MOTORS];
    double mScale = 1.0;
    bool mAbort = false;
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
    
    void show( std::ostream &aOut );
    pos_vct step_to_pos( step_vct& aStep );
    step_vct pos_to_step( pos_vct& aPos );
    step_t pos_to_step( int aNo, pos_t aPos );
    double getPosPerStep( int aNo );
    double getPosPerStep2( int aNo );
    step_t getCurrentStep( int aNo );
    pos_t getCurrentPos( int aNo );
public:
    void thread_loop();
    bool put( int aCmd, string& aLine, usec_t aUsec, step_vct& aStep );
    bool putW( int aCmd, string& aLine, usec_t aUsec, step_vct& aStep );
    bool putW( int aCmd, usec_t aUsec, step_vct& aStep );
    void setScale( double aScale );
    void setDbgWait( usec_t aWait );
    usec_t getDbgWait();
private:
    bool __queue_copy();
    bool recieve_cmd(int *aCmd, string *aLine, usec_t *aUsec, step_t aStep[]);
    bool get( int *aCmd, string *aLine, usec_t *aUsec, step_t aStep[] );
    bool send_cmd( int aCmd, string& aLine, usec_t aUsec, step_vct& aStep );
};

#endif /* GTHREAD_H */

