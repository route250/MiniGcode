/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <math.h>
#include <iomanip>
#include <cerrno>
#include "Gunit.h"


/**
 * 実数をpos_tに変換
 * 四捨五入する
 * @param value
 * @return 
 */
pos_t double_to_pos( double value ) {
    return (pos_t)round(value);
}

void out_pos( std::ostream &aOut, pos_t aPos ){
    if( aPos == 0 ) {
        aOut << '0';
    } else {
        if( aPos < 0 ) {
            aOut << '-';
            aPos = aPos * -1;
        }
        int mm = aPos / 1000;
        int um = aPos % 1000;
        if( mm != 0 ) {
            aOut << mm;;
        }
        aOut << '.';
        if( 0 < um ) {
            if( um < 10 ) {
                aOut << '0';
            }
            if( um < 100 ) {
                aOut << '0';
            }
            aOut << um;
        }
    }
}

void out_step( std::ostream &aOut, step_t aStep ){
    aOut << aStep;
}

void out_feed( std::ostream &aOut, feed_t aFeed ){
    out_pos( aOut, aFeed );
}

void out_double( std::ostream &aOut, double aValue ){
    aOut << std::setprecision(3) << aValue;
}


void out_code( std::ostream &aOut, address_t a, code_t c ){
    aOut << a;
    if( c < 10 ) {
        aOut << '0';
    }
    aOut << c;
}

string str::trim( string aValue ) {
    int s = 0;
    while( s < aValue.size() && std::isblank( aValue[s] ) ) {
        s++;
    }
    int e = aValue.size();
    while( s<e && std::isblank( aValue[e-1] ) ) {
        e--;
    }
    if( s == 0 && e == aValue.size() ) {
        return aValue;
    }
    if( s == e ) {
        return "";
    }
    return aValue.substr(s, e - s );
}

vector<string> str::split( string aValue ) {
    vector<string> zResult;
    std::stringstream zToken;
    int p = 0;
    while( p<aValue.size() ) {
        while( p<aValue.size() && std::isblank(aValue[p] ) ) {
            p++;
        }
        int zCount = 0;
        while( p<aValue.size() && !std::isblank(aValue[p] ) ) {
            zCount++;
            zToken.put( aValue[p] );
            p++;
        }
        if( zCount>0 ) {
            zResult.push_back( zToken.str() );
            zToken.str("");
        }
    }
    return zResult;
}

vector<string> str::split( string aValue, char aSep ) {
    vector<string> zResult;
    std::stringstream st{aValue};
    string zToken;
    while( getline(st,zToken, aSep ) ) {
        if( !zToken.empty() ) {
            zResult.push_back(zToken);
        }
    }
    return zResult;
}

string str::tolower( string aValue ) {
    std::stringstream zRet;
    bool update = false;
    for( int i=0; i<aValue.size(); i++ ) {
        char cc = aValue[i];
        if( 'A'<=cc && cc<='Z' ) {
            update = true;
            cc = ( cc + 33 );
        }
        zRet << cc;
    }
    if( update ) {
        return zRet.str();
    } else {
        return aValue;
    }
}

void swapi( int *a, int *b ) {
    int x = *a;
    *a = *b;
    *b = x;
}

int gpio_to_bcm( int pin ) {
    switch(pin) {
        case 0:
            return 17;
        case 1:
            return 18;
        case 2:
            return 21;
        case 3:
            return 22;
        case 4:
            return 23;
        case 5:
            return 24;
        case 6:
            return 25;
        case 21:
            return 5;
        case 22:
            return 6;
        case 23:
            return 13;
        case 24:
            return 19;
        case 25:
            return 26;
        case 26:
            return 12;
        case 27:
            return 16;
        case 28:
            return 20;
        case 29:
            return 21;
    }
    return 999;
}

void print_pin( std::ostream &aOut, int pin ) {
    int bcm = gpio_to_bcm( pin );
    aOut << "pin:" << pin << "(BCM" << bcm << ") ";
}

void print_HL( std::ostream &aOut, int state ) {
    if( state ) {
        aOut << "H";
    } else {
        aOut << "L";
    }
}

void sleep_msec( int ms ) {

    struct timespec req_time, remaining_time ;

    req_time.tv_sec  = (time_t)(ms / 1000) ;
    req_time.tv_nsec = (long)(ms % 1000) * 1000000 ;
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
    
}
double str_to_double( string aValue, double aDefault ) {
    try{
        double result = std::stod( aValue );
        return result;
    }catch(std::invalid_argument &ex){
        return aDefault;
    }
}
long str_to_long( string aValue, long aDefault ) {
    try{
        long result = std::stol( aValue );
        return result;
    }catch(std::invalid_argument &ex){
        return aDefault;
    }
}
long str_to_long( vector<string>& aValue, int idx, long aDefault ) {
    if( idx <aValue.size() ) {
        return str_to_long( aValue[idx], aDefault );
    }
    return aDefault;
}
int to_axis( string a ) {
    if( a == "x" || a == "X" ) {
        return 0;
    } else if( a == "y" || a == "Y" ) {
        return 1;
    } else if( a == "z" || a == "Z" ) {
        return 2;
    }
    return -1;
}

int to_axis( vector<string>& a, int idx ) {
    if( idx < a.size() ) {
        return to_axis( a[idx] );
    }
    return -1;
}

int to_axis( char a ) {
    if( a == 'x' || a == 'X' ) {
        return 0;
    } else if( a == 'y' || a == 'Y' ) {
        return 1;
    } else if( a == 'z' || a == 'Z' ) {
        return 2;
    }
    return -1;
}