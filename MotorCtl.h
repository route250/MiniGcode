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

#ifndef SMOTOR_H
#define SMOTOR_H

#include <wiringPi.h>

#include "Gunit.h"

#define NUM_PHASE 8

class MotorCtl {
private:
    int mNo;

    bool mReverse;
    step_t mStep_max;
    pos_t mPos_length;
    double mPosPerStep;
    double mStepPerPos;

    int use_half_step;
    int mAssignA1;
    int mAssignA2;
    int mAssignB1;
    int mAssignB2;
    
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
    long pulse_min_length_usec = 200 * 1000;
    long pulse_max_length_usec = 500 * 1000;
    long pulse_inc_usec = ( pulse_max_length_usec - pulse_min_length_usec ) / 3;

    // モータの位置
    step_t current_step = 0;
    step_t start_step;
    step_t target_step = 0;
    
    // コンストラクタ
    MotorCtl();
    MotorCtl( int no );
    virtual ~MotorCtl();
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
    void pulse_min( usec_t min );
    void pulse_max( usec_t usec );
    void pulse_min_max( usec_t min, usec_t max );
    bool isReverse();
    void setReverse( bool b );
    void step_size_length( step_t size, pos_t length );
    step_t getStepMax();
    pos_t getPosLength();
    void set_use_half_step( bool f );

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
    void show( std::ostream& aOut );
    bool isAbort();
    
    step_t pos_to_step( pos_t aPos );
    pos_t step_to_pos( step_t aStep );
    
    step_t getCurrentStep();
    pos_t getCurrentPos();
private:
    void operator =(const MotorCtl &src) {}
    MotorCtl(const MotorCtl &src) {}

};

extern bool is_availavle( MotorCtl *m );
extern void print_motor( MotorCtl *m );

void print_pin( std::ostream &aOut, int pin );

#endif /* SMOTOR_H */

