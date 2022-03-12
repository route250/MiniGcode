/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   smotor.h
 * Author: maeda
 *
 * Created on 2022年2月17日, 0:55
 */

#ifndef MG_AXIS_H
#define MG_AXIS_H

#include <wiringPi.h>

#include "Gunit.h"
#include "MGmath.h"
#include "MGaxis.h"
#include "MGstepper.h"

#define NUM_PHASE 8

class MGaxis {
private:
    int mNo = 0;
    deg_t mStepAngle = 0;
    step_t mMinSpeed = 0;
    step_t mMaxSpeed = 0;
    step_t mIncSpeed = 0;
    
    bool mReverse = false;
    step_t mStep_max = 0;
    pos_t mPos_length = 0;
    double mPosPerStep = 0;
    double mStepPerPos = 0;

    MGstepper mStepper;
    int mExcitationMethod = 0;
    int mAssignA1 = 0;
    int mAssignA2 = 0;
    int mAssignB1 = 0;
    int mAssignB2 = 0;
    
    int mPinA1;
    int stateA1;
    int mPinA2;
    int stateA2;
    int mPinB1;
    int stateB1;
    int mPinB2;
    int stateB2;

    int mOrgPin;
    int mOrgLogic;
    int mOrgPin_pullUpDn;
    bool mDoneHoming;
    bool mAbort;//停止信号？

public:

    int enable_power = 0;
    int mCurrentPhase;
    long pulse_elapsed_time_usec = 0;
    long pulse_length_usec = 0;
//    long pulse_min_length_usec = 200 * 1000;
//    long pulse_max_length_usec = 500 * 1000;
//    long pulse_inc_usec = ( pulse_max_length_usec - pulse_min_length_usec ) / 3;

    // モータの位置
    step_t current_step = 0;
    step_t start_step;
    step_t target_step = 0;
    
    // コンストラクタ
    MGaxis();
    MGaxis( int no );
    virtual ~MGaxis();
    // メソッド
    void setNo( int aNo );
    int no();

    void setMotorPin( int a, int b, int c, int d );
    void setHomePin( int no, bool logic, int pull_updn );
    void pin_reset();
    void init();
    void swapAB();
    void swapA();
    void swapB();
    void setMinSpeed( step_t aSpeed );
    void setMaxSpeed( step_t aSpeed );
    void setStepAngle( deg_t aAngle );
    void setExcitationMethod( int aMethod );

    bool isReverse();
    void setReverse( bool b );
    void step_size_length( step_t size, pos_t length );
    step_t getStepMax();
    pos_t getPosLength();

    bool is_availavle();

    bool isHome();
    bool haveHomingPin();
    void homing(std::ostream &aOut);
    bool isDoneHoming();
    bool set_home();
    bool move( step_t aStep );

    void poff();
    void pon();
    bool is_power();
    // ログ出力用
    void out_phase( int A1, int A2, int B1, int B2 );
    void out_phase_no( int ph );
    void out_step( int direction, bool aForce );
    void signal( long time_nonsec );
    int getPinA1();
    int getPinB1();
    int getPinA2();
    int getPinB2();
    int getStateA1();
    int getStateA2();
    int getStateB1();
    int getStateB2();
    deg_t getStepAngle();
    step_t getMinSpeed();
    step_t getMaxSpeed();
    step_t getIncSpeed();
    void show( std::ostream& aOut );
    bool isAbort();
    
    step_t pos_to_step( pos_t aPos );
    pos_t step_to_pos( step_t aStep );
    double getPosPerStep();
    step_t getCurrentStep();
    pos_t getCurrentPos();
private:
    void operator =(const MGaxis &src) {}
    MGaxis(const MGaxis &src) {}
private:
    void update_config();

};

extern bool is_availavle( MGaxis *m );
extern void print_motor( MGaxis *m );

void print_pin( std::ostream &aOut, int pin );

#endif /* MG_AXIS_H */

