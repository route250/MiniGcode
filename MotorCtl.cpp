/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   smotor.cpp
 * Author: maeda
 * 
 * Created on 2022年2月17日, 0:55
 */

#include <stdio.h>
#include "MotorCtl.h"

MotorCtl::MotorCtl() {
    mPinA1 = mAssignA1 = -1;
    mPinA2 = mAssignA2 = -1;
    mPinB1 = mAssignB1 = -1;
    mPinB2 = mAssignB2 = -1;

    mOrgPin = -1;
    mOrgLogic = LOW;
    mOrgPin_pullUpDn = 0;

    enable_power = -1;
    mCurrentPhase = 0;
    mDoneHoming = false;
    mAbort = false;
}

MotorCtl::~MotorCtl( ) {
}

/**
 * 
 * @return 
 */
int MotorCtl::no( ) {
    return mNo;
}
void MotorCtl::setNo( int aNo ) {
    mNo = aNo;
}

/**
 * 
 */
void MotorCtl::init( ) {
    if( is_availavle( ) ) {
        if( mAssignA1 >= 0 ) {
            pinMode( mAssignA1, OUTPUT );
        }
        if( mAssignA2 >= 0 ) {
            pinMode( mAssignA2, OUTPUT );
        }
        if( mAssignB1 >= 0 ) {
            pinMode( mAssignB1, OUTPUT );
        }
        if( mAssignB2 >= 0 ) {
            pinMode( mAssignB2, OUTPUT );
        }
        out_phase( LOW, LOW, LOW, LOW );
        mCurrentPhase = 9999;
        if( mOrgPin >= 0 ) {
            pinMode( mOrgPin, INPUT );
            if( mOrgPin_pullUpDn == PUD_DOWN ) {
                pullUpDnControl( mOrgPin, PUD_DOWN );
            } else if( mOrgPin_pullUpDn == PUD_UP ) {
                pullUpDnControl( mOrgPin, PUD_UP );
            } else {
                pullUpDnControl( mOrgPin, PUD_OFF );
            }
        }
    }

}

void MotorCtl::setMotorPin( int a, int b, int c, int d ) {
        std::cout<<"[DBG]set A1:" <<a<<std::endl;
    mAssignA1 = a;
    mAssignA2 = b;
    mAssignB1 = c;
    mAssignB2 = d;
    pin_reset();
}

void MotorCtl::pin_reset( ) {
    mPinA1 = mAssignA1;
    mPinA2 = mAssignA2;
    mPinB1 = mAssignB1;
    mPinB2 = mAssignB2;
}

void MotorCtl::swapAB( ) {
    swapi( &mPinA1, &mPinB1 );
    swapi( &stateA1, &stateB1 );
    swapi( &mPinA2, &mPinB2 );
    swapi( &stateA2, &stateB2 );
}

void MotorCtl::swapA( ) {
    swapi( &mPinA1, &mPinA2 );
    swapi( &stateA1, &stateA2 );
}

void MotorCtl::swapB( ) {
    swapi( &mPinB1, &mPinB2 );
    swapi( &stateB1, &stateB2 );
}

int MotorCtl::getPinA1() { return mAssignA1; }
int MotorCtl::getPinA2() { return mAssignA2; }
int MotorCtl::getPinB1() { return mAssignB1; }
int MotorCtl::getPinB2() { return mAssignB2; }

int MotorCtl::getStateA1() { return stateA1; }
int MotorCtl::getStateA2() { return stateA2; }
int MotorCtl::getStateB1() { return stateB1; }
int MotorCtl::getStateB2() { return stateB2; }

void MotorCtl::pulse_min( usec_t usec ) {
    pulse_min_length_usec = usec;
    if( pulse_min_length_usec > pulse_min_length_usec ) {
        pulse_max_length_usec = usec;
    }
    pulse_inc_usec = ( pulse_max_length_usec - pulse_min_length_usec ) / 3;
}

void MotorCtl::pulse_max( usec_t usec ) {
    pulse_max_length_usec = usec;
    if( pulse_max_length_usec < pulse_min_length_usec ) {
        pulse_min_length_usec = usec;
    }
    pulse_inc_usec = ( pulse_max_length_usec - pulse_min_length_usec ) / 3;
}

void MotorCtl::pulse_min_max( usec_t min, usec_t max ) {
    if( min < max ) {
        pulse_min( min );
    } else {
        pulse_min( max );
    }
    pulse_max( max );
}
void MotorCtl::setReverse( bool aReverse ) {
    mReverse = aReverse;
}
bool MotorCtl::isReverse() {
    return mReverse;
}
void MotorCtl::step_size_length( step_t size, pos_t length ) {
    mStep_max = size;
    mPos_length = length;
    mPosPerStep = (double)length / (double)size;
    mStepPerPos = (double)size / (double)length;
}
step_t MotorCtl::getStepMax() {
    return mStep_max;
}
pos_t MotorCtl::getPosLength() {
    return mPos_length;
}

step_t MotorCtl::getCurrentStep() {
    return current_step;
}

step_t MotorCtl::pos_to_step( pos_t aPos ) {
    double v = ((double)aPos) * mStepPerPos;
    step_t zStep = (step_t)v;
    if( mReverse ) {
        return mStep_max - zStep;
    } else {
        return zStep;
    }
}

pos_t MotorCtl::step_to_pos( step_t aStep ) {
    if( mReverse ) {
        double v = ((double)aStep) * mPosPerStep;
        return mPos_length -(pos_t)v;
    } else {
        double v = ((double)aStep) * mPosPerStep;
        return (pos_t)v;
    }
}

pos_t MotorCtl::getCurrentPos() {
    return step_to_pos( getCurrentStep() );
}

void MotorCtl::set_use_half_step( bool f ) {
    use_half_step = f;
}

bool MotorCtl::is_availavle( ) {
    if( mPinA1<0 || mPinA2<0 || mPinB1<0 || mPinB2<0 ) {
        return false;
    }
    if( pulse_max_length_usec == 0 || pulse_min_length_usec == 0 ) {
        return false;
    }
    return true;
}

bool MotorCtl::haveHomingPin() {
    return mOrgPin>=0;
}
void MotorCtl::setHomePin( int no, bool logic, int pull_updn ) {
    mOrgPin = no;
    mOrgLogic = logic ? HIGH : LOW;
    mOrgPin_pullUpDn = pull_updn;
}

bool MotorCtl::isHome( ) {
    if( mOrgPin >= 0 ) {
        if( digitalRead( mOrgPin ) == mOrgLogic ) {
            return true;
        } else {
            return false;
        }
    }
    return true;
}
bool MotorCtl::isDoneHoming() {
    if( mOrgPin >= 0 ) {
        return mDoneHoming && mCurrentPhase>=0;
    } else {
        return mCurrentPhase>=0;
    }
}
bool MotorCtl::isAbort() {
    return mAbort;
}

void MotorCtl::poff( ) {
    if( !is_availavle( ) ) {
        return;
    }
    if( enable_power>= 0 ) {
        printf( "[m%d]power_off\n", mNo );
    }
    enable_power = -1;
    stateA1 = LOW;
    stateA2 = LOW;
    stateB1 = LOW;
    stateB2 = LOW;
    digitalWrite( mPinA1, LOW );
    digitalWrite( mPinA2, LOW );
    digitalWrite( mPinB1, LOW );
    digitalWrite( mPinB2, LOW );
    mCurrentPhase = -1;
    mDoneHoming = false;
}

void MotorCtl::pon() {
    if( !is_availavle( ) ) {
        return;
    }
    if( enable_power<= 1 ) {
        printf( "[m%d]power_on\n", mNo );
    }
    enable_power = 2;
}

bool MotorCtl::is_power() {
    return enable_power>0;
}

bool MotorCtl::set_home() {
    if( is_power() && 0 <= mCurrentPhase && mCurrentPhase <= NUM_PHASE ) {
        pulse_elapsed_time_usec = 0;
        current_step = 0;
        target_step = current_step;
        start_step = current_step;
        mDoneHoming = true;
        return true;
    } else {
        return false;
    }
}

bool MotorCtl::move( step_t aStep ) {
    if( aStep != 0 ) {
        if( mCurrentPhase < 0 || NUM_PHASE <= mCurrentPhase ) {
            mCurrentPhase = 0;
        }
        step_t zDirection = aStep>0 ? 1:-1;
        int delay_msec = pulse_max_length_usec/1000;
        if( delay_msec < 1 ) {
            delay_msec = 1;
        } else if( delay_msec > 100 ) {
            delay_msec = 100;
        }

        while( aStep != 0 ) {
            out_step( zDirection, true );
            aStep -= zDirection;
            sleep_msec( delay_msec );
        }
    }
    return true;
}
void MotorCtl::out_phase( int A1, int A2, int B1, int B2 ) {
    if( enable_power <=0 ) {
        return;
    }
    if( stateA1 != A1 ) {
        digitalWrite( mPinA1, A1 );
        stateA1 = A1;
    }
    if( stateA2 != A2 ) {
        digitalWrite( mPinA2, A2 );
        stateA2 = A2;
    }
    if( stateB1 != B1 ) {
        digitalWrite( mPinB1, B1 );
        stateB1 = B1;
    }
    if( stateB2 != B2 ) {
        digitalWrite( mPinB2, B2 );
        stateB2 = B2;
    }
}

void MotorCtl::out_phase_no( int ph ) {

    switch( ph ) {
        case 0:
            out_phase( LOW, HIGH, LOW, HIGH );
            break;
        case 1:
            out_phase( LOW, HIGH, LOW, LOW );
            break;
        case 2:
            out_phase( LOW, HIGH, HIGH, LOW );
            break;
        case 3:
            out_phase( LOW, LOW, HIGH, LOW );
            break;
        case 4:
            out_phase( HIGH, LOW, HIGH, LOW );
            break;
        case 5:
            out_phase( HIGH, LOW, LOW, LOW );
            break;
        case 6:
            out_phase( HIGH, LOW, LOW, HIGH );
            break;
        case 7:
            out_phase( LOW, LOW, LOW, HIGH );
            break;
    }

    mCurrentPhase = ph;

}

void MotorCtl::out_step( int direction, bool aForce ) {
    int next_phase;
    int step;
    if( direction > 0 ) {
        if ( aForce || 0<= current_step && current_step < mStep_max ) {
            next_phase = ( mCurrentPhase + 1 ) % NUM_PHASE;
        }
        step = 1;
    } else if( direction < 0 ) {
        if( aForce || 0<current_step && current_step <=mStep_max ) {
            next_phase = ( mCurrentPhase + NUM_PHASE - 1 ) % NUM_PHASE;
        }
        step = -1;
    } else {
        next_phase = mCurrentPhase;
        step = 0;
    }
    out_phase_no( next_phase );
    current_step = current_step + step;
}

void MotorCtl::signal( long time_usec ) {

    if( !is_availavle( ) ) {
        return;
    }

    if( current_step == target_step && target_step == start_step && enable_power == 2 || enable_power == -1 ) {
        pulse_elapsed_time_usec = 0;
        pulse_length_usec = 0;
        return;
    }
    if( pulse_length_usec <= 0 ) {
        pulse_elapsed_time_usec = 0;
        pulse_length_usec = pulse_max_length_usec;
    }
    pulse_elapsed_time_usec = pulse_elapsed_time_usec + time_usec;
    if( pulse_elapsed_time_usec < pulse_length_usec ) {
        return;
    }
    pulse_elapsed_time_usec = 0;

    switch( enable_power ) {
        case -1:
            return;
        case 1:
            //printf( "[m%d]power_on\n", mNo );
            enable_power = 2;
        case 2:
            break;
        default:
            //printf( "[m%d]power_off\n", mNo );
            enable_power = -1;
            out_phase( LOW, LOW, LOW, LOW );
            return;
    }

    int remaining_length, moved_length, total_length;
    if( target_step > current_step ) {
        // ＋移動
        out_step( 1, false );
        remaining_length = target_step - current_step;
        total_length = target_step - start_step;
        moved_length = current_step - start_step;
    } else if( target_step < current_step ) {
        // ー移動
        out_step( -1, false );
        remaining_length = current_step - target_step;
        total_length = start_step - target_step;
        moved_length = start_step - current_step;
    } else {
        // 移動終了
        start_step = current_step;
        return;
    }
    // 加減速にかかる時間
    long delta_usec;
    if( pulse_max_length_usec == pulse_min_length_usec || pulse_inc_usec == 0 ) {
        delta_usec = 0;
    } else {
        delta_usec = ( pulse_max_length_usec - pulse_min_length_usec ) / pulse_inc_usec + 1;
    }
    if( ( total_length - 2 ) < delta_usec * 2 ) {
        delta_usec = ( total_length - 2 ) / 2;
    }
    if( moved_length == 0 ) {
        // 始点
        pulse_length_usec = pulse_max_length_usec;
    } else if( remaining_length == 0 ) {
        // 終点
        pulse_length_usec = pulse_max_length_usec;
    } else if( moved_length <= delta_usec ) {
        // 加速
        pulse_length_usec = pulse_length_usec - pulse_inc_usec;
        if( pulse_length_usec < pulse_min_length_usec ) {
            pulse_length_usec = pulse_min_length_usec;
        }
    } else if( remaining_length <= delta_usec ) {
        // 減速
        pulse_length_usec = pulse_length_usec + pulse_inc_usec;
        if( pulse_length_usec > pulse_max_length_usec ) {
            pulse_length_usec = pulse_max_length_usec;
        }
    }


}

void MotorCtl::homing( std::ostream &aOut ) {

    if( !is_availavle() ) {
        //printf( "(seek_origin)invalid motor no %d\n", motor_no );
        return;
    }

    int delay_msec = pulse_max_length_usec * 3 /1000;
    int delay2_msec = 200;
    if( delay_msec < 20 ) {
        delay_msec = 20;
        delay2_msec = 100;
    } else if( delay_msec > 200 ) {
        delay_msec = 200;
        delay2_msec = 1000;
    } else {
        delay2_msec = delay_msec * 5;
    }
    //poff();
    if( haveHomingPin() ) {
        aOut << "["<<mNo<<"]scan origin 1"<<std::endl;
        sleep_msec( 500 );

        step_t zStepLength = getStepMax();
        step_t ss = zStepLength < 100 ? 10 : zStepLength/10;
        // 原点を探す
        mCurrentPhase = 0;
        if( !isHome() ) {
            aOut << " 001 detect origin signal"<<std::endl;
            for( int i=0 ; i<=zStepLength && !isHome() && !isAbort(); i++ ) {
                out_step( -1, true );
                sleep_msec( delay_msec );
            }
            if( !isHome() ) {
                aOut << "ERROR: can not detect org signal" << std::endl;
                return;
            }
            sleep_msec( 200 );
        }

        mCurrentPhase = 0;
        if( isHome() ) {
            aOut << " 002 detect origin signal "<<ss <<std::endl;
            for( int i=0 ; i<ss && isHome() && !isAbort(); i++ ) {
                out_step( 1, true );
                sleep_msec( delay_msec );
            }
            if( isHome() ) {
                aOut << "ERROR: can not detect org signal" << std::endl;
                return;
            }
            sleep_msec( 200 );
        }
        if( !isHome() ) {
            printf(" 003 seek to origin\n");
            for( int i=0 ; i<ss && !isHome() && !isAbort(); i++ ) {
                out_step( -1, true );
                sleep_msec( delay_msec );
            }
            if( !isHome() ) {
                printf("ERROR: can not seek to origin\n");
                return;
            }
            sleep_msec( 200 );
        }
        
        // 確認する
        printf(" 005 check origin\n");
        step_t xx = 0;
        if( isHome() ) {
            sleep_msec( 100 );
            for( int i=0 ; i<ss && isHome() && !isAbort(); i++ ) {
                out_step( 1, true );
                xx++;
                sleep_msec( delay_msec );
            }
            if( isHome() ) {
                printf("ERROR: can not seek to origin\n");
                return;
            }
            sleep_msec( 100 );
            for( int i=0 ; i<ss && !isHome() && !isAbort(); i++ ) {
                out_step( -1, true );
                xx--;
                sleep_msec( delay_msec );
            }
            if( !isHome() ) {
                printf("ERROR: can not seek to origin\n");
                return;
            }
        }
        
    } else {
        aOut << "["<<mNo<<"]scan origin 1"<<std::endl;
        sleep_msec( 500 );

        for( int i = 0; i < 400; i++ ) {
            out_step( -1, true );
            sleep_msec( 1 );
            out_phase( LOW, LOW, LOW, LOW );
            sleep_msec( 10 );
        }
        out_phase( LOW, LOW, LOW, LOW );
        mCurrentPhase = 0;
        for( int i = 0; i < NUM_PHASE; i++ ) {
            out_step( 1, true );
            sleep_msec( 1 );
        }
    }
    pulse_elapsed_time_usec = 0;
    current_step = 0;
    target_step = current_step;
    start_step = current_step;
    mDoneHoming = true;
}


bool is_availavle( MotorCtl *m ) {
    if( m != NULL ) {
        return m->is_availavle();
    }
    return false;
}

void MotorCtl::show( std::ostream &aOut ) {
    aOut << "motor no:" << no() << std::endl;
    
    aOut << " ";
    aOut << " "; print_pin( aOut, getPinA1() );
    aOut << " "; print_pin( aOut, getPinA2() );
    aOut << " "; print_pin( aOut, getPinB1() );
    aOut << " "; print_pin( aOut, getPinB2() );
    aOut << std::endl;
    aOut << "  phase:" << mCurrentPhase << " ";
    aOut << " " << getStateA1();
    aOut << " " << getStateA2();
    aOut << " " << getStateB1();
    aOut << " " << getStateB2();
    aOut << std::endl;
    aOut << "  speed(msec):";
    aOut << " " << (float)(pulse_min_length_usec)/1000.0;
    aOut << "-" << (float)(pulse_max_length_usec)/1000.0;
    aOut << "/" << (float)(pulse_inc_usec)/1000.0;
    if( haveHomingPin() ) {
        aOut << "  home:"; print_pin( aOut, mOrgPin );
        int ah = isHome() ? HIGH : LOW;
        print_HL( aOut, ah );
        if( mOrgLogic == HIGH ) {
            aOut <<" logic:H";
        } else if( mOrgLogic == LOW ) {
            aOut <<" logic:L";
        } else {
            aOut <<"/logic:?";
        }
        if( mOrgPin_pullUpDn == PUD_OFF ) {
            aOut << " P_OFF";
        } else if( mOrgPin_pullUpDn == PUD_UP ) {
            aOut << " P_UP";
        } else if( mOrgPin_pullUpDn == PUD_DOWN ) {
            aOut << " P_DN";
        } else {
            aOut << " P???";
        }
        aOut << std::endl;
    } else {
        aOut << "  home:not present" <<std::endl;
    }
    aOut << "  step:" <<getCurrentStep() <<"/"<< mStep_max;
    if( mReverse ) {
        aOut << "(reverse)";
    } else {
        aOut << "(forward)";
    }
    aOut << "  pos:" <<getCurrentPos() <<"/"<< mPos_length;
    aOut <<std::endl;
    
}
