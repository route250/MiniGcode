/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "MGmath.h"




double_vct vct::from_pos( pos_vct& aVct ){
    double_vct zRet(aVct.size());
    for( int i=0; i<aVct.size(); i++ ) {
        zRet[i] = aVct[i];
    }
    return zRet;
}
double_vct vct::from_step( step_vct& aVct ){
    double_vct zRet(aVct.size());
    for( int i=0; i<aVct.size(); i++ ) {
        zRet[i] = aVct[i];
    }
    return zRet;
}
pos_vct vct::to_pos( double_vct& aVct ){
    pos_vct zRet(aVct.size());
    for( int i=0; i<aVct.size(); i++ ) {
        zRet[i] = aVct[i];
    }
    return zRet;
}
step_vct vct::to_step( double_vct& aVct ){
    step_vct zRet(aVct.size());
    for( int i=0; i<aVct.size(); i++ ) {
        zRet[i] = aVct[i];
    }
    return zRet;
}

double_vct vct::diff( double_vct& a, double_vct& b ) {
    double_vct zRet(a.size());
    for( int i=0; i<a.size(); i++ ) {
        zRet[i] = a[i] - b[i];
    }
    return zRet;
}

double vct::length( double_vct& aVector ) {
    double zLen = 0;
    for( int i=0; i<aVector.size(); i++ ) {
        zLen = zLen + aVector[i]*aVector[i];
    }
    return sqrt(zLen);
}

double_vct vct::norm( double_vct& aVector, double aLength ) {
    double_vct zRet(aVector.size());
    for( int i=0; i<aVector.size(); i++ ) {
        zRet[i] = aVector[i]/aLength;
    }
    return zRet;
}

double_vct vct::norm( double_vct& aVector ) {
    double zLength = length(aVector);
    return norm( aVector, zLength );
}

bool vct::is_ok( pos_vct& aVector ) {
    if( aVector.size() == 0 ) {
        return false;
    }
    for( int i=0; i<aVector.size(); i++ ) {
        if( aVector[i] == INVALID_POS ) {
            return false;
        }
    }
    return true;
}
pos_vct vct::scale_pos( pos_vct& aPos, double aScale, pos_vct& aOffset ) {
    pos_vct zRet(aPos.size());
    for( int i=0; i<NUM_MOTORS; i++ ) {
        zRet[i] = aPos[i] * aScale + aOffset[i];
    }
    return zRet;
}

void out_pos( std::ostream &aOut, pos_vct& aPos ) {
    aOut << "(";
    for( int i=0; i<aPos.size(); i++ ) {
        if( i>0 ) {
            aOut << ",";
        }
        out_pos( aOut, aPos[i] );
    }
    aOut << ")";
}

void out_step( std::ostream &aOut, step_vct& aStep ) {
    aOut << "(";
    for( int i=0; i<aStep.size(); i++ ) {
        if( i>0 ) {
            aOut << ",";
        }
        out_step( aOut, aStep[i] );
    }
    aOut << ")";
}

void out_double( std::ostream &aOut, double_vct& aPos ) {
    aOut << "(";
    for( int i=0; i<aPos.size(); i++ ) {
        if( i>0 ) {
            aOut << ",";
        }
        out_double( aOut, aPos[i] );
    }
    aOut << ")";
}

usec_t pps_to_usec( step_t aSpeed ) {
    // pps = pulse/sec = pulse*1000000/usec
    // 1/pps = sec/pulse = usec/(pulse*1000000)
    if( aSpeed > 0 ) {
        return 1000000 / aSpeed;
    } else {
        return 0;
    }
}
step_t usec_to_pps( usec_t aUsec ) {
    // pps = pulse/sec = pulse*1000000/usec
    // 1/pps = sec/pulse = usec/(pulse*1000000)
    if( aUsec > 0 ) {
        return 1000000 / aUsec;
    } else {
        return 0;
    }
}