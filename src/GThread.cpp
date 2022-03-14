/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GThread.cpp
 * Author: maeda
 * 
 * Created on 2022年2月26日, 0:11
 */

#include <wiringPi.h>

#include "GThread.h"

using std::cout;
using std::endl;
using namespace std::chrono;

#ifdef __i386
__inline__ uint64_t rdtsc() {
    uint64_t x;
    __asm__ volatile ("rdtsc" : "=A" (x));
    return x;
}
#elif __amd64
__inline__ uint64_t rdtsc() {
    uint64_t a, d;
    __asm__ volatile ("rdtsc" : "=a" (a), "=d" (d));
    return (d<<32) | a;
}
#endif

using std::endl;

GThread::GThread() {
    mDbg = false;
    mRun = false;
    mCurrentLine = "";
    mCurrentTime = 0;
    for( int i=0; i<NUM_MOTORS; i++ ) {
        mCurrentStep[i] = 0;
        mOffsetPos[i] = 0;
    }
    mNext = false;
    mNextTime = 0;
    mAbort = false;
}

GThread::GThread( const GThread& orig ) {
}

GThread::~GThread( ) {
}
//------------------------------------------------------------------------------
// getter/setter
//------------------------------------------------------------------------------
void GThread::setDebug( bool b ) {
    mDbg = b;
}
void GThread::setDbgWait( usec_t aWait ) {
    mDbgWait = aWait;
}
usec_t GThread::getDbgWait() {
    return mDbgWait;
}

void GThread::setScale( double aScale ) {
    mScale = aScale;
}

pos_vct GThread::step_to_pos( step_vct& aStep ) {
    pos_vct aResult(NUM_MOTORS);
    for( int i=0; i<NUM_MOTORS; i++ ) {
        aResult[i] = mAxis[i].step_to_pos( aStep[i] ) + mOffsetPos[i];
    }
    return aResult;
}

step_vct GThread::pos_to_step( pos_vct& aPos ) {
    step_vct aResult(NUM_MOTORS);
    for( int i=0; i<NUM_MOTORS; i++ ) {
        aResult[i] = mAxis[i].pos_to_step( aPos[i] - mOffsetPos[i] );
    }
    return aResult;
}

double GThread::getPosPerStep( int aNo ) {
    return mAxis[aNo].getPosPerStep();
}

step_t GThread::pos_to_step( int aNo, pos_t aPos ) {
    return mAxis[aNo].pos_to_step( aPos - mOffsetPos[aNo] );
}
step_t GThread::getCurrentStep( int aNo ) {
    return mAxis[aNo].getCurrentStep();
}
pos_t GThread::getCurrentPos( int aNo ) {
    return mAxis[aNo].getCurrentPos() + mOffsetPos[aNo];
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

bool GThread::configure( config& aConfig ) {
    for( int i=0; i<NUM_MOTORS; i++ ) {
        mAxis[i].setNo(i);
        string zPath;
        zPath.push_back( 'x'+i );

        int zPich = aConfig.getNumber( zPath + "/pich", -1 );
        int zLength = aConfig.getNumber( zPath + "/length", -1 );
        double zStepAngle = aConfig.getNumber( zPath + "/motor/stepangle", -1 );
        int zExcitation = aConfig.getNumber( zPath + "/motor/excitationmethod", -1 );
        if( zStepAngle > 0 ) {
            int n = (360.0/zStepAngle)+0.5;
            if( n> 0 ) {
                zStepAngle = 360.0 / n;
            } else {
                zStepAngle = 0;
            }
        }
        int zSteps = 0;
        if( zStepAngle > 0 && zPich > 0 && zLength > 0 ) {
            int n = 360.0/zStepAngle;
            if( zExcitation == 12 || zExcitation == 21 ) {
                n = n * 2;
            }
            double d = ((double)zPich) / (double)n;
            zSteps = ((double)zLength) / d;
            zLength = d * zSteps;
        }
        if( zPich > 0 ) {
            mPich[i] = zPich;
        } else {
            mPich[i] = 0;
        }
        mAxis[i].setStepAngle( zStepAngle );
        mAxis[i].setExcitationMethod( zExcitation );
        mAxis[i].step_size_length( zSteps, zLength );

        int zOffset = aConfig.getNumber( zPath + "/offset", 0 );
        bool zReverse = aConfig.getBool( zPath + "/reverse", false );

        mAxis[i].setReverse( zReverse );
        mOffsetPos[i] = zOffset;
        std::cout << "[config]" << zPath <<" steps:" << zSteps << " length:" << zLength << std::endl;

        int pinA1 = aConfig.getNumber( zPath + "/motor/pinA1", 0 );
        int pinA2 = aConfig.getNumber( zPath + "/motor/pinA2", 0 );
        int pinB1 = aConfig.getNumber( zPath + "/motor/pinB1", 0 );
        int pinB2 = aConfig.getNumber( zPath + "/motor/pinB2", 0 );
        std::cout<<"[DBG] A1:" <<pinA1<<std::endl;
        mAxis[i].setMotorPin( pinA1, pinA2, pinB1, pinB2 );

        int min = aConfig.getNumber( zPath + "/motor/minpps", -1 );
        if( min > 0 ) {
            mAxis[i].setMinSpeed(min);
        }
        int max = aConfig.getNumber( zPath + "/motor/maxpps", -1 );
        if( max > 0 ) {
            mAxis[i].setMaxSpeed(max);
        }
        

        {
            int zPin = aConfig.getNumber( zPath + "/home/pin", 0 );
            std::string v = str::tolower( aConfig.getText( zPath + "/home/pud" ) );
            int zPullUD = PUD_OFF;
            if( v == "dn" ) {
                zPullUD = PUD_DOWN;
            } else if( v == "up" ) {
                zPullUD = PUD_UP;
            }
            bool zLogic = aConfig.getBool( zPath + "/home/logic", false );
            mAxis[i].setHomePin( zPin, zLogic, zPullUD);
        }

    }
    for( int i=0; i<NUM_MOTORS; i++ ) {
        if( mAxis[i].is_availavle() ) {
            mAxis[i].init();
        }
    }
    return true;
}

void GThread::adjust(std::ostream &aOut) {

    std::ios_base::fmtflags original_flags = cout.flags();
    long zCostTime_nanosec = 0;
    long zCostWait_nanosec = 0;
    long zMinmumWait_usec = 0;
    // 現在時間取得にかかる時間を計測
    {
        aOut<<"(1)test high_resolution_clock"<<endl;
        int max = 100000;
        high_resolution_clock::time_point start_time = high_resolution_clock::now();
        long dummy_long = 0;
        microseconds dummy_ms(0);
        nanoseconds t0_nanosec = duration_cast<nanoseconds>(start_time.time_since_epoch());
        long zBefore=t0_nanosec.count(), zTotal = 0;

        high_resolution_clock::time_point zNow;
        for( int i=0; i<max; i++ ) {
            zNow = high_resolution_clock::now();
            nanoseconds t_nanosec = duration_cast<nanoseconds>(zNow.time_since_epoch());
            long nanosec = t_nanosec.count();
            zTotal += (nanosec - zBefore);
            microseconds u = duration_cast<microseconds>(start_time-zNow);
            dummy_ms = dummy_ms + u;
            long dummy_count = u.count();
            dummy_long = dummy_long + dummy_count;
            zBefore = nanosec;
        }
        high_resolution_clock::time_point end_time = high_resolution_clock::now();
        nanoseconds t_last_nanosec = duration_cast<nanoseconds>(end_time-start_time);
        long elaps_nanosec = t_last_nanosec.count();
        long ave_long = elaps_nanosec / max;
        aOut<<"    run:"<<max;
        aOut<<" total:"<<elaps_nanosec<<","<<zTotal<<"(nanosec) diff:"<<(zTotal-elaps_nanosec);
        aOut<<" ave:"<<ave_long<<"(nanosec) "<<(zTotal/max)<<endl;
        zCostTime_nanosec = ave_long;
    }
    
    // sleepにかかる時間
    {
        int max = 200;
        aOut<<"(2)test nano_sleep run:" << max << endl;
        struct timespec req_time, remaining_time ;
        long zMaxCost = 0;
        for( int sleep_usec = 0; sleep_usec<=500; sleep_usec += sleep_usec < 50 ? 10 : 50 ) {
            high_resolution_clock::time_point t0 = high_resolution_clock::now();
            long dummy = 0;
            for( int i=0; i<max; i++ ) {
                req_time.tv_sec  = 0;
                req_time.tv_nsec = sleep_usec * 1000;
                remaining_time.tv_sec = 0;
                remaining_time.tv_nsec = 0;
                while( nanosleep (&req_time, &remaining_time) != 0 ) {
                    switch( errno ) {
                        case EINTR:
                            req_time.tv_sec = remaining_time.tv_sec;
                            req_time.tv_nsec = remaining_time.tv_nsec;
                            continue;
                        default:
                            std::cerr << "ERROR:nanosleep return " << errno <<" : ";
                            std::perror(NULL);
                            break;
                    }
                    break;
                }
                dummy = dummy + 1;
            }
            high_resolution_clock::time_point t9 = high_resolution_clock::now();
            nanoseconds t_last_nanosec = duration_cast<nanoseconds>(t9-t0);
            long elaps_nanosec = t_last_nanosec.count();
            long ave_long = elaps_nanosec/max;
            long diff = elaps_nanosec - (sleep_usec * 1000 * max );
            long zCost_nanosec = diff/max;
            aOut<<"    sleep:"<<sleep_usec<<"(usec)";
            aOut<<" total:"<<elaps_nanosec<<"(nanosec)";
            aOut<<" ave:"<<ave_long<<"(nanosec)";
            aOut<<"    over:"<<diff<<"(nanosec)";
            aOut<<" ave:"<<zCost_nanosec<<"(nanosec)";
            aOut<<endl;
            if( zCostWait_nanosec<zCost_nanosec ) {
                zCostWait_nanosec = zCost_nanosec;
            }
            if( zMinmumWait_usec == 0 && (sleep_usec/2)>(zCost_nanosec/1000) ) {
                zMinmumWait_usec = sleep_usec;
            }
        }
    }
    
    // sleepにかかる時間
    {
        int max = 200;
        aOut<<"(3)test nano_sleep run:" << max << endl;
        struct timespec req_time, remaining_time ;
        long zMaxCost = 0;
        int sleep_usec = zMinmumWait_usec;
        {
            high_resolution_clock::time_point t0 = high_resolution_clock::now();
            long dummy = 0;
            for( int i=0; i<max; i++ ) {
                req_time.tv_sec  = 0;
                req_time.tv_nsec = sleep_usec * 1000 - zCostWait_nanosec - zCostTime_nanosec;
                remaining_time.tv_sec = 0;
                remaining_time.tv_nsec = 0;
                while( nanosleep (&req_time, &remaining_time) != 0 ) {
                    switch( errno ) {
                        case EINTR:
                            req_time.tv_sec = remaining_time.tv_sec;
                            req_time.tv_nsec = remaining_time.tv_nsec;
                            continue;
                        default:
                            std::cerr << "ERROR:nanosleep return " << errno <<" : ";
                            std::perror(NULL);
                            break;
                    }
                    break;
                }
                high_resolution_clock::time_point t_now = high_resolution_clock::now();
                nanoseconds t_last_nanosec = duration_cast<nanoseconds>(t_now-t0);
                long now_nanosec = t_last_nanosec.count();
                long diff = now_nanosec - sleep_usec * 1000;
                dummy = dummy + diff;
            }
            high_resolution_clock::time_point t9 = high_resolution_clock::now();
            nanoseconds t_last_nanosec = duration_cast<nanoseconds>(t9-t0);
            long elaps_nanosec = t_last_nanosec.count();
            long ave_long = elaps_nanosec/max;
            long diff = elaps_nanosec - (sleep_usec * 1000 * max );
            long zCost_nanosec = diff/max;
            aOut<<"    sleep:"<<sleep_usec<<"(usec)";
            aOut<<" total:"<<elaps_nanosec<<"(nanosec)";
            aOut<<" ave:"<<ave_long<<"(nanosec)";
            aOut<<"    over:"<<diff<<"(nanosec)";
            aOut<<" ave:"<<zCost_nanosec<<"(nanosec)";
            aOut<<endl;
        }
    }
    {
        long pps = 1000000 / zMinmumWait_usec;
        aOut << "---" <<endl;
        aOut << "  max pps:" << pps << "(pps)"<<endl;
    }
    aOut.flags( original_flags );
    mCostTime_nanosec = zCostTime_nanosec;
    mCostWait_nanosec = zCostWait_nanosec;
    mMinimumWait_nanosec = zMinmumWait_usec * 1000;
}

bool GThread::isAbort() {
    for( int i=0; i<NUM_MOTORS; i++ ) {
        if( mAxis[i].isAbort() ) {
            return true;
        }
    }
    return mAbort;
}

void GThread::poff() {
    for( int i = 0;i<NUM_MOTORS; i++ ) {
        mAxis[i].poff();
    }
}
void GThread::pon() {
    for( int i = 0;i<NUM_MOTORS; i++ ) {
        mAxis[i].pon();
    }
}
bool GThread::is_power() {
    for( int i = 0;i<NUM_MOTORS; i++ ) {
        if( !mAxis[i].is_power() ) {
            return false;
        }
    }
    return true;
}

bool GThread::isDoneHoming() {
    for( int i = 0;i<NUM_MOTORS; i++ ) {
        if( !mAxis[i].isDoneHoming() ) {
            return false;
        }
    }
    return true;
}

void GThread::scan_origin( int aNo, std::ostream &aOut ) {
    if( 0<=aNo && aNo<NUM_MOTORS ) {
        mAxis[aNo].homing(aOut);
        mCurrentStep[aNo] = 0;
    }
}
bool GThread::set_home( int aNo, std::ostream &aOut ) {
    if( 0<=aNo && aNo<NUM_MOTORS ) {
        return mAxis[aNo].set_home();
    }
    return false;
}
bool GThread::set_limit( int aNo, std::ostream &aOut ) {
    aOut<<"not implemented"<<endl;
    return false;
}
bool GThread::set_zero( int aNo, std::ostream &aOut ) {
    if( 0<=aNo && aNo<NUM_MOTORS ) {
        pos_t zCurrentPos = mAxis[aNo].getCurrentPos();
        pos_t zOff = -zCurrentPos;
        aOut << "offset "<<mOffsetPos[aNo]<<" to "<<zOff<<endl;
        mOffsetPos[aNo] = zOff;
        return true;
    }
    return false;
}
bool GThread::move( int aNo, step_t aStep, std::ostream &aOut ) {
    if( 0<=aNo && aNo<NUM_MOTORS ) {
        return mAxis[aNo].move(aStep);
    }
    return false;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

void *thread_engry( void *p ) {
    ((GThread*)p)->thread_loop();
    return NULL;
}

void GThread::thread_loop() {

    mActive = true;
    std::cout << "...started thread\n" << std::endl;
    if( mCostTime_nanosec == 0 || mCostWait_nanosec == 0 || mMinimumWait_nanosec == 0 ) {
        adjust(std::cout);
    }
    high_resolution_clock::time_point start = high_resolution_clock::now();

    int zIdleWait = 200 * 1000000;
    timespec zInterval, zWait, zPausess, remaining_time;
    zInterval.tv_sec = 0;
    zInterval.tv_nsec = 0;
    zWait.tv_sec = 0;
    zWait.tv_nsec = mDbgWait * 1000;

    int zCmd;
    string zLine;
    usec_t zTargetTime_usec;
    step_t zStep[NUM_MOTORS];
    long zCmdCount = 0, zShortIdleCount = 0, zLongIdleCount = 0;
    long zSlipCountA = 0, zSlipMinA = 0, zSlipMaxA = 0;
    long zSlipCountB = 0, zSlipMinB = 0, zSlipMaxB = 0;
    
    high_resolution_clock::time_point zBlockStartTime = high_resolution_clock::now();
    high_resolution_clock::time_point zNow = high_resolution_clock::now();
    high_resolution_clock::time_point zLastTime = high_resolution_clock::now();
    
    while( mRun ) {

        bool ret = recieve_cmd( &zCmd, &zLine, &zTargetTime_usec, zStep );
        zNow = high_resolution_clock::now();
        if( !ret ) {
            nanoseconds t_xtime = duration_cast<nanoseconds>( zNow-zLastTime);
            if( t_xtime.count() > 1000000000 ) {
                zLongIdleCount++;
                zInterval.tv_nsec = zIdleWait;
                nanosleep( &zInterval, NULL );
            } else {
                zShortIdleCount++;
            }
            continue;
        }
        zLastTime = zNow;
        
        if( zCmd == THCMD_00 ) {
            mCurrentLine = "";
            std::cout<<"[Thread] Start00"<<std::endl;
            zIdleWait = 500 * 1000000;
            zBlockStartTime = zNow;
            continue;
        } else if( zCmd == CMD_START_BLOCK ) {
            //std::cout<<"[Thread] Start Block"<<std::endl;
            mCurrentLine = zLine;
            zIdleWait = 20 * 1000000;
            zBlockStartTime = zNow;
            zCmdCount = 0;
            zSlipCountA = 0; zSlipMinA = 0; zSlipMaxA = 0;
            zSlipCountB = 0; zSlipMinB = 0; zSlipMaxB = 0;
            zShortIdleCount = 0;
            zLongIdleCount = 0;
            continue;
        } else if( zCmd == CMD_END_BLOCK ) {
            //std::cout<<"[Thread] End Block"<<std::endl;
            if( zSlipCountA > 0 || zSlipCountB > 0 ) {
                cout<<"[Thread] End Block count:"<<zCmdCount<<" idle:"<<zShortIdleCount<<","<<zLongIdleCount<<endl;
                cout<<"                   slipA:"<<zSlipCountA<<" "<<zSlipMinA<<" - "<<zSlipMaxA<<endl;
                cout<<"                   slipB:"<<zSlipCountB<<" "<<zSlipMinB<<" - "<<zSlipMaxB<<endl;
            }
            mCurrentLine = "";
            continue;
        } else if( zCmd == THCMD_99 || zCmd != CMD_TICK ) {
            mCurrentLine = "";
            std::cout<<"[Thread] End99"<<std::endl;
            zIdleWait = 500 * 1000000;
            mRun = false;
            continue;
        }
       
        zCmdCount++;
        // 実時間で経過時間を計算
        nanoseconds t_blocktime = duration_cast<nanoseconds>(zNow-zBlockStartTime);
        // 待機時間を計算
        long wait_nanosec = (zTargetTime_usec*1000) - t_blocktime.count() - mCostTime_nanosec;

        if( wait_nanosec > 0 ) {
            // 余裕がある
            if( wait_nanosec < mCostWait_nanosec ) {
                // nanosleepでは間に合わない
                zSlipCountA++;
                if( zSlipMinA == 0 ) {
                    zSlipMinA = wait_nanosec;
                    zSlipMaxA = wait_nanosec;
                } else if( wait_nanosec > zSlipMaxA ) {
                    zSlipMaxA = wait_nanosec;
                } else if( wait_nanosec < zSlipMinA ) {
                    zSlipMinA = wait_nanosec;
                }
                // 指定時間までループで待機
                while( mRun && wait_nanosec > 0 ) {
                    high_resolution_clock::time_point zNow2 = high_resolution_clock::now();
                    t_blocktime = duration_cast<nanoseconds>(zNow2-zBlockStartTime);
                    wait_nanosec = (zTargetTime_usec*1000) - t_blocktime.count() - mCostTime_nanosec;
                }
            } else {
                // 指定時間までnanosleepで待機
                wait_nanosec = wait_nanosec - mCostWait_nanosec;
                zPausess.tv_sec = wait_nanosec / 1000000000;
                zPausess.tv_nsec = ( wait_nanosec % 1000000000 );
                while( mRun && nanosleep (&zPausess, &remaining_time) != 0 ) {
                    switch( errno ) {
                        case EINTR:
                            zPausess.tv_sec = remaining_time.tv_sec;
                            zPausess.tv_nsec = remaining_time.tv_nsec;
                            continue;
                        default:
                            std::cerr << "ERROR:nanosleep return " << errno <<" : ";
                            std::perror(NULL);
                            break;
                    }
                    break;
                }
            }
        } else if( wait_nanosec < 0 ) {
            // 時間が過ぎてる！
            zSlipCountB++;
            if( zSlipMinB == 0 ) {
                zSlipMinB = wait_nanosec;
                zSlipMaxB = wait_nanosec;
            } else if( wait_nanosec > zSlipMaxB ) {
                zSlipMaxB = wait_nanosec;
            } else if( wait_nanosec < zSlipMinB ) {
                zSlipMinB = wait_nanosec;
            }
        }

        if( !mRun ) {
            break;
        }
        
        // 移動する
        if( mDbg ) {
            std::cout <<"...steps " << (zTargetTime_usec/1000) << "(msec) ";
            for( int i=0; i<NUM_MOTORS; i++ ) {
                std::cout <<" "<< zStep[i];
            }
            std::cout << std::endl;
        }
        bool update = true;
        while( update ) {
            // 移動する
            for( int i=0; i<NUM_MOTORS; i++ ) {
                int d = zStep[i] - mCurrentStep[i];
                if( d < 0 ) {
                    mAxis[i].out_step( -1, false );
                    mCurrentStep[i]--;
                } else if( d > 0 ) {
                    mAxis[i].out_step( 1, false );
                    mCurrentStep[i]++;
                }
            }
            // 追加の移動がある？
            update = false;
            for( int i=0; i<NUM_MOTORS; i++ ) {
                int d = zStep[i] - mCurrentStep[i];
                if( d < 0 ) {
                    update = true;
                } else if( d > 0 ) {
                    update = true;
                }
            }
            
            if( !update ) {
                // 移動終了
                break;
            }
            nanosleep( &zWait, NULL );
        }

    }
    std::cout << "...exit thread\n" << std::endl;
    mActive = false;
}

void GThread::start() {
    mRun = true;
    mActive = false;
    pthread_create( &mThread, NULL, thread_engry, (void*)this );

    timespec zInterval = {0, 200*1000*1000};
    while( !mActive ) {
        nanosleep( &zInterval, NULL );
    }
}

void GThread::stop() {
    timespec zInterval = {0, 200*1000*1000};
    while( mActive ) {
        mRun = false;
        nanosleep( &zInterval, NULL );
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

bool GThread::__queue_copy() {
    std::lock_guard<std::mutex> lock(mMutex);
    if( (QLEN - mQ1len) >= mCmdSend_length ) {
        for( int dst=mQ1len,src=0; src<mCmdSend_length; dst++,src++ ) {
            mQ1_NextCmd[dst] = mCmdSend_cmd[src];
            mQ1_NextLine[dst] = mCmdSend_cmd[src];
            mQ1_NextTime[dst] = mCmdSend_time[src];
        }
        for( int dst=mQ1len*NUM_MOTORS,src=0; src<mCmdSend_length*NUM_MOTORS; dst++,src++ ) {
            mQ1_NextStep[dst] = mCmdSend_step[src];
        }
        mQ1len+=mCmdSend_length;
        mCmdSend_length = 0;
    }
    return false;
}

bool GThread::recieve_cmd(int *aCmd, string *aLine, usec_t *aUsec, step_t aStep[]) {
    if( mQ1len <= 0 ) {
        __queue_copy();
    }
    if( mQ1len <= 0 ) {
        return false;
    }
    *aCmd = mQ1_NextCmd[0];
    *aLine = mQ1_NextLine[0];
    *aUsec = mQ1_NextTime[0];
    for( int m=0; m<NUM_MOTORS; m++ ) {
        aStep[m] = mQ1_NextStep[m];
    }
    for( int dst=0,src=1; src<mQ1len; dst++,src++ ) {
        mQ1_NextCmd[dst] = mQ1_NextCmd[src];
        mQ1_NextLine[dst] = mQ1_NextCmd[src];
        mQ1_NextTime[dst] = mQ1_NextTime[src];
    }
    for( int dst=0,src=NUM_MOTORS; src<mQ1len*NUM_MOTORS; dst++,src++ ) {
        mQ1_NextStep[dst] = mQ1_NextStep[src];
    }
    mQ1len--;
    return true;
}
bool GThread::get( int *aCmd, string *aLine, usec_t *aUsec, step_t aStep[]) {
    std::lock_guard<std::mutex> lock(mMutex);
    if( mNext ) {
        *aCmd = mNextCmd;
        *aLine = mNextLine;
        *aUsec = mNextTime;
        for( int i=0; i<NUM_MOTORS; i++ ) {
            aStep[i] = mNextStep[i];
        }
        mNextTime = 0;
        mNext = false;
        return true;
    } else {
        return false;
    }
}

bool GThread::send_cmd( int aCmd, string& aLine, usec_t aUsec, step_vct& aStep ) {
    std::lock_guard<std::mutex> lock(mMutex);
    if( mCmdSend_length < QLEN ) {
        for( int src=0,dst=mCmdSend_length*NUM_MOTORS; src<NUM_MOTORS; src++,dst++ ) {
            mCmdSend_step[dst] = aStep[src];
        }
        mCmdSend_time[mCmdSend_length] = aUsec;
        mCmdSend_line[mCmdSend_length] = aLine;
        mCmdSend_cmd[mCmdSend_length] = aCmd;
        mCmdSend_length++;
        return true;
    } else {
        return false;
    }
}

bool GThread::put( int aCmd, string& aLine, usec_t aUsec, step_vct& aStep ) {
    std::lock_guard<std::mutex> lock(mMutex);
    if( !mNext ) {
        for( int i=0; i<NUM_MOTORS; i++ ) {
            mNextStep[i] = aStep[i];
        }
        mNextTime = aUsec;
        mNextLine = aLine;
        mNextCmd = aCmd;
        mNext = true;
        return true;
    } else {
        return false;
    }
}

bool GThread::putW( int aCmd, string& aLine, usec_t aUsec, step_vct& aStep ) {
     if( !mRun ) {
         start();
     }
    timespec zInterval = {0, 300*1000*1000 };
     while( mRun && !send_cmd( aCmd, aLine, aUsec, aStep ) ) {
        nanosleep( &zInterval, NULL );
     }
     return mRun;
}
bool GThread::putW( int aCmd, usec_t aUsec, step_vct& aStep ) {
     if( !mRun ) {
         start();
     }
    string BLANK = "";
    timespec zInterval = {0, 200*1000 };
     while( !send_cmd( aCmd, BLANK, aUsec, aStep ) ) {
        nanosleep( &zInterval, NULL );
     }
     return true;
}

bool GThread::send00( step_vct& aStep ) {
    string BLANK = "";
    return putW( THCMD_00, BLANK, 0, aStep );
}

bool GThread::send_start_block( string& aLine, step_vct& aStep ) {
     return putW( CMD_START_BLOCK, aLine, 0, aStep );
}

void GThread::send_step( usec_t aUsec, step_vct& aStep ) {
     putW( CMD_TICK, aUsec, aStep );
     //printf("send %6ld(msec) %d,%d,%d\n",aUsec/1000,aStep[0],aStep[1],aStep[2]);
 }

bool GThread::send_end_block( step_vct& aStep ) {
     return putW( CMD_END_BLOCK, 0, aStep );
}

bool GThread::send99( step_vct& aStep ) {
     return putW( THCMD_99, 0, aStep );
}


//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void GThread::show( std::ostream &aOut ) {
    aOut << "Abort:" << isAbort();
    aOut << " Power:" << is_power();
    aOut << " Homeing:" <<isDoneHoming();
    aOut << endl;
    for( int i=0; i<NUM_MOTORS; i++ ) {
        mAxis[i].show( aOut );
        //
        aOut << "  pich:" <<mPich[i];
        aOut << " offset:" <<mOffsetPos[i];
        pos_t zC = mAxis[i].getCurrentPos();
        pos_t zL = mAxis[i].getPosLength();
        aOut << " cur:" << (zC + mOffsetPos[i]);
        aOut << " max:" << (zL + mOffsetPos[i]);
        aOut << endl;
    }
}

