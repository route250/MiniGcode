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
        aResult[i] = mMotors[i].step_to_pos( aStep[i] ) + mOffsetPos[i];
    }
    return aResult;
}

step_vct GThread::pos_to_step( pos_vct& aPos ) {
    step_vct aResult(NUM_MOTORS);
    for( int i=0; i<NUM_MOTORS; i++ ) {
        aResult[i] = mMotors[i].pos_to_step( aPos[i] - mOffsetPos[i] );
    }
    return aResult;
}

step_t GThread::pos_to_step( int aNo, pos_t aPos ) {
    return mMotors[aNo].pos_to_step( aPos - mOffsetPos[aNo] );
}
step_t GThread::getCurrentStep( int aNo ) {
    return mMotors[aNo].getCurrentStep();
}
pos_t GThread::getCurrentPos( int aNo ) {
    return mMotors[aNo].getCurrentPos() + mOffsetPos[aNo];
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

bool GThread::configure( config& aConfig ) {
    for( int i=0; i<NUM_MOTORS; i++ ) {
        mMotors[i].setNo(i);
        string zPath;
        zPath.push_back( 'x'+i );

        int zPich = aConfig.getNumber( zPath + "/pich", -1 );
        int zLength = aConfig.getNumber( zPath + "/length", -1 );
        double zStepAngle = aConfig.getNumber( zPath + "/motor/stepangle", -1 );
        int zExcitation = aConfig.getBool( zPath + "/motor/excitationmethod", false );
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
        mMotors[i].setStepAngle( zStepAngle );
        mMotors[i].setExcitationMethod( zExcitation );
        mMotors[i].step_size_length( zSteps, zLength );

        int zOffset = aConfig.getNumber( zPath + "/offset", 0 );
        bool zReverse = aConfig.getBool( zPath + "/reverse", false );

        mMotors[i].setReverse( zReverse );
        mOffsetPos[i] = zOffset;
        std::cout << "[config]" << zPath <<" steps:" << zSteps << " length:" << zLength << std::endl;

        int pinA1 = aConfig.getNumber( zPath + "/motor/pinA1", 0 );
        int pinA2 = aConfig.getNumber( zPath + "/motor/pinA2", 0 );
        int pinB1 = aConfig.getNumber( zPath + "/motor/pinB1", 0 );
        int pinB2 = aConfig.getNumber( zPath + "/motor/pinB2", 0 );
        std::cout<<"[DBG] A1:" <<pinA1<<std::endl;
        mMotors[i].setMotorPin( pinA1, pinA2, pinB1, pinB2 );

        int min = aConfig.getNumber( zPath + "/motor/minpps", -1 );
        if( min > 0 ) {
            mMotors[i].setMinSpeed(min);
        }
        int max = aConfig.getNumber( zPath + "/motor/maxpps", -1 );
        if( max > 0 ) {
            mMotors[i].setMaxSpeed(max);
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
            mMotors[i].setHomePin( zPin, zLogic, zPullUD);
        }

    }
    for( int i=0; i<NUM_MOTORS; i++ ) {
        if( mMotors[i].is_availavle() ) {
            mMotors[i].init();
        }
    }
    return true;
}

void GThread::adjust(std::ostream &aOut) {

    long zCostTime_nanosec = 0;
    long zCostWait_nanosec = 0;
    long zMinmumWait_usec = 0;
    // 現在時間取得にかかる時間を計測
    {
        aOut<<"(1)test high_resolution_clock"<<endl;
        int max = 100000;
        high_resolution_clock::time_point t0 = high_resolution_clock::now();
        long l = 0, ll = 0;
        microseconds x(0);
        nanoseconds t0_nanosec = duration_cast<nanoseconds>(t0.time_since_epoch());
        long zBefore=t0_nanosec.count(), zTotal = 0;

        high_resolution_clock::time_point t;
        for( int i=0; i<max; i++ ) {
            t = high_resolution_clock::now();
            nanoseconds t_nanosec = duration_cast<nanoseconds>(t.time_since_epoch());
            long nanosec = t_nanosec.count();
            zTotal += (nanosec - zBefore);
            microseconds u = duration_cast<microseconds>(t0-t);
            x = x + u;
            long lu = u.count();
            ll = ll + lu;
            zBefore = nanosec;
        }
        high_resolution_clock::time_point t9 = high_resolution_clock::now();
        nanoseconds t_last_nanosec = duration_cast<nanoseconds>(t9-t0);
        long elaps_nanosec = t_last_nanosec.count();
        long ave_long = elaps_nanosec / max;
        aOut<<"    run:"<<max;
        aOut<<" total:"<<elaps_nanosec<<","<<zTotal<<"(nanosec) diff:"<<(zTotal-elaps_nanosec);
        aOut<<" ave:"<<ave_long<<"(nanosec) "<<(zTotal/max)<<endl;
        zCostTime_nanosec = ave_long;
    }
    
    // sleepにかかる時間
    {
        aOut<<"(2)test nano_sleep"<<endl;
        int max = 200;
        struct timespec req_time, remaining_time ;
        long zMaxCost = 0;
        for( int sleep_usec = 50; sleep_usec<=500; sleep_usec += 50 ) {
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
            aOut<<"    run:"<<max;
            aOut<<" sleep:"<<sleep_usec<<"(usec)";
            aOut<<endl;
            aOut<<"    total:"<<elaps_nanosec<<"(nanosec)";
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
        aOut<<"(3)test nano_sleep"<<endl;
        int max = 200;
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
            aOut<<"    run:"<<max;
            aOut<<" sleep:"<<sleep_usec<<"(usec)";
            aOut<<endl;
            aOut<<"    total:"<<elaps_nanosec<<"(nanosec)";
            aOut<<" ave:"<<ave_long<<"(nanosec)";
            aOut<<"    over:"<<diff<<"(nanosec)";
            aOut<<" ave:"<<zCost_nanosec<<"(nanosec)";
            aOut<<endl;
        }
    }
    mCostTime_nanosec = zCostTime_nanosec;
    mCostWait_nanosec = zCostWait_nanosec;
    mMinimumWait_usec = zMinmumWait_usec;
}

bool GThread::isAbort() {
    for( int i=0; i<NUM_MOTORS; i++ ) {
        if( mMotors[i].isAbort() ) {
            return true;
        }
    }
    return mAbort;
}

void GThread::poff() {
    for( int i = 0;i<NUM_MOTORS; i++ ) {
        mMotors[i].poff();
    }
}
void GThread::pon() {
    for( int i = 0;i<NUM_MOTORS; i++ ) {
        mMotors[i].pon();
    }
}
bool GThread::is_power() {
    for( int i = 0;i<NUM_MOTORS; i++ ) {
        if( !mMotors[i].is_power() ) {
            return false;
        }
    }
    return true;
}

bool GThread::isDoneHoming() {
    for( int i = 0;i<NUM_MOTORS; i++ ) {
        if( !mMotors[i].isDoneHoming() ) {
            return false;
        }
    }
    return true;
}

void GThread::scan_origin( int aNo, std::ostream &aOut ) {
    if( 0<=aNo && aNo<NUM_MOTORS ) {
        mMotors[aNo].homing(aOut);
        mCurrentStep[aNo] = 0;
    }
}
bool GThread::set_home( int aNo, std::ostream &aOut ) {
    aOut<<"not implemented"<<endl;
    return false;
}
bool GThread::set_limit( int aNo, std::ostream &aOut ) {
    aOut<<"not implemented"<<endl;
    return false;
}
bool GThread::set_zero( int aNo, std::ostream &aOut ) {
    if( 0<=aNo && aNo<NUM_MOTORS ) {
        pos_t zCurrentPos = mMotors[aNo].getCurrentPos();
        pos_t zOff = -zCurrentPos;
        aOut << "offset "<<mOffsetPos[aNo]<<" to "<<zOff<<endl;
        mOffsetPos[aNo] = zOff;
        return true;
    }
    return false;
}
bool GThread::move( int aNo, step_t aStep, std::ostream &aOut ) {
    if( 0<=aNo && aNo<NUM_MOTORS ) {
        return mMotors[aNo].move(aStep);
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
    if( mCostTime_nanosec == 0 || mCostWait_nanosec == 0 || mMinimumWait_usec == 0 ) {
        adjust(std::cout);
    }
    high_resolution_clock::time_point start = high_resolution_clock::now();
    int w = 200 * 1000;
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
    long zSlipCount = 0, zCmdCount = 0;
    
    high_resolution_clock::time_point zBlockStartTime = high_resolution_clock::now();
    while( mRun ) {

        zInterval.tv_nsec = zIdleWait;
        while( !get( &zCmd, &zLine, &zTargetTime_usec, zStep ) ) {
            nanosleep( &zInterval, NULL );
            if( !mRun ) {
                break;
            }
        }
        
        if( zCmd == THCMD_00 ) {
            mCurrentLine = "";
            std::cout<<"[Thread] Start00"<<std::endl;
            zIdleWait = 500 * 1000000;
            zBlockStartTime = high_resolution_clock::now();
            continue;
        } else if( zCmd == CMD_START_BLOCK ) {
            //std::cout<<"[Thread] Start Block"<<std::endl;
            mCurrentLine = zLine;
            zIdleWait = 20 * 1000000;
            zBlockStartTime = high_resolution_clock::now();
            zCmdCount = 0;
            zSlipCount = 0;
            continue;
        } else if( zCmd == CMD_END_BLOCK ) {
            //std::cout<<"[Thread] End Block"<<std::endl;
            if( zSlipCount>0 ) {
                cout<<"[Thread] End Block Slip:"<<zSlipCount<<"/"<<zCmdCount<<endl;
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
        // 指定時間まで待機
        high_resolution_clock::time_point now = high_resolution_clock::now();
        nanoseconds t_blocktime = duration_cast<nanoseconds>(now-zBlockStartTime);
        long wait_nanosec = zTargetTime_usec*1000 - t_blocktime.count() - mCostTime_nanosec;

        if( wait_nanosec > 0 ) {
            if( wait_nanosec < mMinimumWait_usec ) {
                wait_nanosec = mMinimumWait_usec * 1000 - mCostWait_nanosec;
                zSlipCount++;
            } else {
                wait_nanosec = wait_nanosec - mCostWait_nanosec;
            }
            zPausess.tv_sec = wait_nanosec / 1000000000;
            zPausess.tv_nsec = (wait_nanosec%1000000000);
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
        } else if( wait_nanosec < 0 ) {
            zSlipCount++;
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
            update = false;
            for( int i=0; i<NUM_MOTORS; i++ ) {
                int d = zStep[i] - mCurrentStep[i];
                if( d < 0 ) {
                    update = true;
                    mMotors[i].out_step( -1, false );
                    mCurrentStep[i]--;
                } else if( d > 0 ) {
                    update = true;
                    mMotors[i].out_step( 1, false );
                    mCurrentStep[i]++;
                }
            }
//            std::cout<<"seek to ";
//            for( int i=0; i<NUM_MOTORS;i++) {
//                std::cout<<" ";
//                out_step( std::cout, zStep[i]);
//            }
//            std::cout<<" current ";
//            for( int i=0; i<NUM_MOTORS;i++) {
//                std::cout<<" ";
//                out_step( std::cout, mCurrentStep[i]);
//            }
//            std::cout<<std::endl;
            if( update ) {
                nanosleep( &zWait, NULL );
            } else {
                break;
            }
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
     while( !put( aCmd, aLine, aUsec, aStep ) ) {
        timespec zInterval = {0, 200*1000 };
        nanosleep( &zInterval, NULL );
     }
     return true;
}
bool GThread::putW( int aCmd, usec_t aUsec, step_vct& aStep ) {
     if( !mRun ) {
         start();
     }
    string BLANK = "";
     while( !put( aCmd, BLANK, aUsec, aStep ) ) {
        timespec zInterval = {0, 20*1000 };
        nanosleep( &zInterval, NULL );
     }
     return true;
}
bool GThread::send00( step_vct& aStep ) {
    return putW( THCMD_00, 0, aStep );
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
        mMotors[i].show( aOut );
        //
        aOut << "  pich:" <<mPich[i];
        aOut << " offset:" <<mOffsetPos[i];
        pos_t zC = mMotors[i].getCurrentPos();
        pos_t zL = mMotors[i].getPosLength();
        aOut << " cur:" << (zC + mOffsetPos[i]);
        aOut << " max:" << (zL + mOffsetPos[i]);
        aOut << endl;
    }
}

