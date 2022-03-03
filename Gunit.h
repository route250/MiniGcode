/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Gunit.h
 * Author: maeda
 *
 * Created on 2022年2月24日, 6:28
 */

#ifndef GUNIT_H
#define GUNIT_H

#include <vector>
#include <climits>
#include <iostream>
#include <cmath>
#include <errno.h>

using std::string;
using std::vector;

#define NUM_MOTORS 3

typedef char address_t;
typedef int code_t;
typedef int step_t;
typedef int pos_t;

typedef int feed_t;
typedef long msec_t;
typedef long usec_t;
typedef long nanosec_t;
#define INVALID_POS INT_MIN

pos_t double_to_pos( double pos);
#define INPUT_BUFFER_SIZE 2048
#define INTR_INTERVAL_USEC 200
#define INVALID_STEP -99999999


void out_pos( std::ostream &aOut, pos_t aPos );
void out_step( std::ostream &aOut, step_t aStep );
void out_feed( std::ostream &aOut, feed_t aFeed );
void out_double( std::ostream &aOut, double aValue );
void out_code( std::ostream &aOut, address_t a, code_t c );
void print_HL( std::ostream &aOut, int state );

namespace str {
    string trim( string aValue );
    vector<string> split( string aValue );
    vector<string> split( string aValue, char aSep );
    string tolower( string aValue );
}

void swapi( int *a, int *b );
int gpio_to_bcm( int pin );
void sleep_msec( int ms );

double str_to_double( string aValue, double aDefault );
long str_to_long( string aValue, long aDefault );
long str_to_long( vector<string>& aValue, int idx, long aDefault );

int to_axis( string a );
int to_axis( vector<string>& aArgs, int idx );
int to_axis( char a );
    
#endif /* GUNIT_H */

