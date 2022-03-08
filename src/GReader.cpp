/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GReader.cpp
 * Author: maeda
 * 
 * Created on 2022年2月24日, 6:26
 */
#include <readline/readline.h>
#include <readline/history.h>
#include "GReader.h"

GReader::GReader() {
    mFileName = "";
//    for( int i=0; i<3; i++ ) {
//        mCurrentPos[i] = 0;
//        mNextPos[i] = 0;
//    }
    mFeed = 0;
    is_open = true;
    mLineNo = 0;
}

GReader::GReader( std::string aFileName ) {
    mFileName = aFileName;
//    for( int i=0; i<3; i++ ) {
//        mCurrentPos[i] = 0;
//        mNextPos[i] = 0;
//    }
    mFeed = 0;
}

//GReader::GReader( const GReader& orig ) {
//}

GReader::~GReader( ) {
}

bool GReader::open( ) {
    printf( "open\n" );
    is_open = true;
    mLineNo = 0;
    if( !mFileName.empty() ) {
        mIn.open( mFileName );
        if( !mIn.is_open( ) ) {
            return false;
        }
        return true;
    } else {
        return true;
    }
}
void GReader::stripLine() {
    // コメント削除
    std::string::size_type p = mBuffer.find( '(' );
    if( p != std::string::npos ) {
        std::string::size_type e = mBuffer.find( ')', p );
        if( e != std::string::npos ) {
            // コメントの前後がブランクだけ？
            bool sp = true;
            for( int i=0; i<p; i++ ) {
                if( !std::isblank( mBuffer[i] ) ) {
                    sp = false;
                    break;
                }
            }
            for( int i=e+1; i<mBuffer.size(); i++ ) {
                if( !std::isblank( mBuffer[i] ) && mBuffer[i] != ';' ) {
                    sp = false;
                    break;
                }
            }
            if( sp ) {
                vector<string> list = str::split( mBuffer.substr( p+1,(e-p)-1) );
                int zCode = list.size()>0 ? to_cmd( list[0] ) : CMD_NOT_FOUND;
                if( zCode != CMD_NOT_FOUND ) {
                    mBuffer.erase( 0 );
                    mBuffer = "(" + std::to_string(zCode);
                    for( int i=1; i<list.size(); i++ ) {
                        mBuffer = mBuffer + " " + list[i];
                    }
                } else {
                    mBuffer.erase( p, ( e - p ) + 1 );
                }
            } else {
                mBuffer.erase( p, ( e - p ) + 1 );
            }
        } else {
            mBuffer.erase( p );
        }
    }
    p = mBuffer.find( ';' );
    if( p != std::string::npos ) {
        mBuffer.erase( p );
    }
}

void GReader::readLine() {
    mBuffer.erase( 0 );
    if( mFileName.empty() ) {
        while( mBuffer.empty() ) {
            char *line = readline( "MiniCNC>> " );
            if( line != NULL ) {
                if( line[0] != '\0' ) {
                    add_history(line);
                }
                string s = line;
                mBuffer = s;
                mLineNo++;
                stripLine();
                if( mBuffer[0] == '%' ) {
                    mBuffer.erase( 0, 1 );
                }
            } else {
                printf("EOF\n");
                break;
            }
        }
    } else {
        if( !is_open ) {
            open( );
        }
        while( mIn.is_open() && mBuffer.empty( ) ) {
            if( std::getline( mIn, mBuffer ) ) {
                mLineNo++;
                stripLine();
                if( mBuffer[0] == '%' ) {
                    mBuffer.erase( 0, 1 );
                }
            } else {
                close( );
            }
        }
    }
}

int GReader::read( ) {
    if( mPos >= mBuffer.size( ) ) {
        mPos = 0;
        readLine();
        if( mBuffer.empty() ) {
            return -1;
        }
        mBuffer += '\n';
    }
    unsigned char cc = mBuffer[mPos++];
    return(int) cc;
}
int GReader::readInt( int p ) {
    int value = 0;
    for( ; p<mBuffer.size(); p++ ) {
        char cc = mBuffer[p];
        int digit = cc - '0';
        if( 0<= digit && digit <=9 ) {
            value = value * 10 + digit;
        } else {
            break;
        }
    }
    return value;
}

pos_t GReader::readPosRef( pos_t aCurrentPos, int p ) {
    pos_t zPos = 0;
    int count = 0;
    int sigin = 0;
    // 符号
    if( p<mBuffer.size() ) {
        switch( mBuffer[p] ) {
            case '+':
                sigin = 1;
                p++;
                break;
            case '-':
                sigin = -1;
                p++;
                break;
        }
    }
    // ミリ単位
    while( p<mBuffer.size() ) {
        char cc = mBuffer[p];
        if( '0' <= cc && cc <= '9' ) {
            count++;
            zPos = zPos * 10 + (cc-'0');
            p++;
        } else {
            break;
        }
    }
    // 小数点
    if( p<mBuffer.size() ) {
        if( mBuffer[p] == '.' ) {
            zPos = zPos*1000;
            p++;
        }
    }
    // マイクロ単位
    int rate = 100;
    while( p<mBuffer.size() ) {
        char cc = mBuffer[p];
        if( '0' <= cc && cc <= '9' ) {
            count++;
            if( rate > 0 ) {
                zPos = zPos + ( rate * (cc-'0') );
                rate = rate/10;
            }
            p++;
        } else {
            break;
        }
    }
    // 結果
    if( count <=0 ) {
        // 数値なし
        return INVALID_POS;
    } else if( sigin >= 0 ) {
        // 絶対値
        return aCurrentPos + zPos;
    } else {
        // 相対値
        return aCurrentPos - zPos;
    }
}

pos_t GReader::readPos( int p ) {
    pos_t zPos = 0;
    int count = 0;
    int sigin = 0;
    // 符号
    if( p<mBuffer.size() ) {
        switch( mBuffer[p] ) {
            case '+':
                sigin = 1;
                p++;
                break;
            case '-':
                sigin = -1;
                p++;
                break;
        }
    }
    // ミリ単位
    while( p<mBuffer.size() ) {
        char cc = mBuffer[p];
        if( '0' <= cc && cc <= '9' ) {
            count++;
            zPos = zPos * 10 + (cc-'0');
            p++;
        } else {
            break;
        }
    }
    // 小数点
    if( p<mBuffer.size() ) {
        if( mBuffer[p] == '.' ) {
            zPos = zPos*1000;
            p++;
        }
    }
    // マイクロ単位
    int rate = 100;
    while( p<mBuffer.size() ) {
        char cc = mBuffer[p];
        if( '0' <= cc && cc <= '9' ) {
            count++;
            if( rate > 0 ) {
                zPos = zPos + ( rate * (cc-'0') );
                rate = rate/10;
            }
            p++;
        } else {
            break;
        }
    }
    // 結果
    if( count <=0 ) {
        // 数値なし
        return INVALID_POS;
    } else if( sigin >= 0 ) {
        // 絶対値
        return zPos;
    } else {
        // 相対値
        return  -(zPos);
    }
}

char ADDR_LIST[] = "GM(";

GBlock GReader::nextBlock( pos_vct& aCurrentPos ) {
    GBlock zGBlock;
    readLine();
    if( mBuffer.empty() ) {
        return zGBlock;
    }
    int zLineNo = mLineNo;
    std::string zRawLine = mBuffer;
    pos_t mCurrentPos[NUM_MOTORS];
    pos_t mNextPos[NUM_MOTORS];
    pos_t mNextCenter[3];
    feed_t zFeed;
    for( int i=0; i<3; i++ ) {
        mCurrentPos [i] = aCurrentPos[i];
        mNextPos[i] = aCurrentPos[i];
        mNextCenter [i] = mCurrentPos[i];
    }
    address_t zAddress=0;
    code_t zCode=0;
    std::string::size_type p;

    for( char *x = ADDR_LIST; *x!='\0'; x++ ) {
        p = mBuffer.find( *x );
        if( p != std::string::npos ) {
            zAddress=*x;
            zCode = readInt(p+1);
        }
    }
    
    bool haveXYZ = false;
    for( int i=0; i<3; i++ ) {
        char cc = 'X' + i;
        p = mBuffer.find( cc );
        if( p != std::string::npos ) {
            haveXYZ = true;
            mNextPos[i] = readPosRef( 0, p+1 );
        }
    }
    bool haveIJK = false;
    for( int i=0; i<3; i++ ) {
        char cc = 'I' + i;
        p = mBuffer.find( cc );
        if( p != std::string::npos ) {
            haveIJK = true;
            mNextCenter[i] = readPosRef( mCurrentPos[i], p+1 );
        }
    }
    bool haveFeed = false;
    p = mBuffer.find( 'F' );
    if( p != std::string::npos ) {
        haveFeed = true;
        mFeed = readPosRef( 0, p+1 );
    }
    if( haveXYZ ) {
        if( haveIJK ) {
            zGBlock = GBlock( zLineNo, zRawLine, zAddress,zCode, mCurrentPos, mNextPos, mNextCenter, mFeed );
        } else {
            zGBlock = GBlock( zLineNo, zRawLine, zAddress,zCode, mCurrentPos, mNextPos, mFeed );
        }
    } else {
        zGBlock = GBlock( zLineNo, zRawLine, zAddress,zCode, mCurrentPos, mFeed );
    }
    return zGBlock;
}

void GReader::close( ) {
    if( !mFileName.empty() && mIn.is_open( ) ) {
        mIn.close( );
    }
}

