/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MGmath.h
 * Author: maeda
 *
 * Created on 2022年2月28日, 21:29
 */

#ifndef MGMATH_H
#define MGMATH_H

#include <cmath>
#include <vector>

#include "Gunit.h"

#define RAD360 (2*M_PI)
#define RAD180 M_PI
#define RAD90 (M_PI*0.5)
#define RAD270 (M_PI*1.5)
#define RAD30 (M_PI/6.0)

#define E3 1000
#define E6 1000000
#define E9 1000000000

using std::vector;

typedef double rad_t;
typedef double deg_t;
typedef double double_t;
typedef vector<step_t> step_vct;
typedef vector<pos_t> pos_vct;
typedef vector<double_t> double_vct;

namespace angle {

    /**
     * radをゼロ〜2*PIの間にする
     * @param rad
     * @return 
     */
    inline rad_t normlz_rad( rad_t rad ) {
        rad = fmod( rad, RAD360 );
        if( rad < 0 ) {
            return RAD360 + rad;
        } else {
            return rad;
        }
    }

    inline deg_t normlz_deg( deg_t deg ) {
        deg = fmod( deg, 360.0 );
        if( deg < 0 ) {
            return 360.0 + deg;
        } else {
            return deg;
        }
    }

    inline deg_t rad_to_deg( rad_t rad ) {
        double deg = rad / M_PI * 180.0;
        deg = normlz_deg( deg );
        return deg;
    }

    inline rad_t deg_to_rad( deg_t deg ) {
        rad_t rad = deg * M_PI/180.0;
        rad = normlz_rad( rad );
        return rad;
    }

    /**
     * (dx,dy)から時計回りとする角度へ変換
     * dx,dyはノーマライズされている前提
     * @param x
     * @param y
     * @return 
     */
    inline rad_t cw_to_rad( double dx, double dy ) {
        double rad = atan2(dy,dx);
        rad = normlz_rad( rad );
        return (rad_t)rad;
    }

    /**
     * 時計回りとする三角関数
     * @param rad
     * @return 
     */
    inline double cw_sin( rad_t v ) {
        return sin(v);
    }

    /**
     * 時計回りとする三角関数
     * @param rad
     * @return 
     */
    inline double cw_cos( rad_t v ) {
        return cos( v );
    }

    inline rad_t ccw_to_rad( double_t dx, double_t dy ) {
        double rad = normlz_rad( -(atan2(dy,dx)) );
        return (rad_t)rad;
    }

    inline double ccw_sin( rad_t rad ) {
        return sin( -(rad) );
    }

    inline double ccw_cos( rad_t rad ) {
        return cos( -(rad) );
    }
    /**
     * radの差分 a-bを計算
     * @param a
     * @param b
     * @return 
     */
    inline rad_t rad_diff( rad_t a, rad_t b ) {
        return angle::normlz_rad( angle::normlz_rad(a) + RAD360 - angle::normlz_rad(b) );
    }
}

namespace vct {
    double_vct from_pos( pos_vct& aVct );
    double_vct from_step( step_vct& aVct );
    pos_vct to_pos( double_vct& aVct );
    pos_vct scale_pos( pos_vct& aVct, double aScale, pos_vct& aOffset );
    step_vct to_step( double_vct& aVct );
    double_vct norm( double_vct& aVct, double aLen );
    double_vct norm( double_vct& aVct );
    double_vct diff( double_vct& a, double_vct& b );
    double_t length( double_vct& aVct );
    bool is_ok( pos_vct & aVct );
    double dot( double_vct& a, double_vct& b );
    double_vct corss( double_vct& a, double_vct& b );
}

void out_pos( std::ostream &aOut, pos_vct& aPos );
void out_step( std::ostream &aOut, step_vct& aStep );
void out_double( std::ostream &aOut, double_vct& aValue );

usec_t pps_to_usec( step_t aSpeed );
step_t usec_to_pps( usec_t aUsec );
#endif /* MGMATH_H */

