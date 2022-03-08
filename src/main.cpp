/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: maeda
 *
 * Created on 2022年2月24日, 6:26
 */

#include <cstdlib>

#include <math.h>
#include "GReader.h"
#include "GThread.h"
#include "config.h"

#include <signal.h>
using std::cout;
using std::endl;

/* シグナルハンドラ */
void sig_handler( int signo ) {
    if( signo == SIGUSR1 ) {
        printf( "received SIGUSR1\n" );
    } else if( signo == SIGHUP ) {
        printf( "received SIGHUP\n" );
    } else if( signo == SIGQUIT ) {
        printf( "received SIGQUIT\n" );
    } else if( signo == SIGILL ) {
        printf( "received SIGILL\n" );
    } else if( signo == SIGKILL ) {
        printf( "received SIGKILL\n" );
    } else if( signo == SIGSTOP ) {
        printf( "received SIGSTOP\n" );
    } else if( signo == SIGTERM ) {
        printf( "received SIGTERM\n" );
    }
}

bool check( std::ostream &zOut, GThread& zThread, int *zStatus ) {
    if( !zThread.is_power( ) ) {
        if( *zStatus != 4 ) {
            zOut << "ERROR: power is off" << endl;
            *zStatus = 4;
        }
        return false;
    } else if( !zThread.isDoneHoming( ) ) {
        if( *zStatus != 2 ) {
            zOut << "ERROR: Homing is incomplite" << endl;
            *zStatus = 2;
        }
        return false;
    } else if( zThread.isAbort( ) ) {
        if( *zStatus != 1 ) {
            zOut << "ERROR: detect abort signal" << endl;
        }
        *zStatus = 1;
        return false;
    }
    return *zStatus == 0;
}

bool zG02toG01 = false;
bool zCCW = false;

feed_t zMaxFeed = 800000; // 800mm/min
pos_vct xCurrentPos( NUM_MOTORS );

double getPosPerUsec( std::ostream &zOut, GThread& zThread, GBlock& zBlk ) {
    // 送り速度
    double zFeed = zMaxFeed;
    if( zBlk.getCode() != 0 ) {
        zFeed = (double) zBlk.getFeed( );
        if( zFeed <= 0 ) {
            zOut << "ERROR:feed=0.0 use 400.0" << endl;
            zFeed = 400.;
        }
    }
    // 送り速度(pos/min)から pos/usec つまり um/usecに変換
    //double x = (zFeed1)/60.0/1000.0/1000.0;
    double zPosPerUsec = zFeed / 60.0 / 1000.0 / 1000.0; // speed (pos/usec)
    return zPosPerUsec;

}
/**
 直線移動の計算用
 */
struct line_move_data {
    // 移動開始位置
    pos_t start_pos;
    // 移動終了位置
    pos_t end_pos;
    // 移動距離
    pos_t length_pos;
    
    // モータの１ステップで移動する距離
    double pos_per_step;
    // マイクロ秒あたりに移動すべき距離
    double pos_per_usec;
    // １ステップの時間
    double usec_per_step;
    
    // 移動距離をステップに変換した値
    step_t total_steps;
    // 移動にかける時間
    double total_time;
    
    // 現在のステップ(開始位置をゼロとして)
    step_t step;
    // 現在の時間
    double current_time_usec;
    // 現在の位置
    pos_t current_pos;
    // 次の時間
    double next_time_usec;
    /**
     * 初期化
     * @param aStart
     * @param aEnd
     * @param aPosPerStep
     * @param aTimeUsec
     */
    void init( pos_t aStart, pos_t aEnd, double aPosPerStep, double aTimeUsec ) {
        step=0;
        current_time_usec=0;
        start_pos = aStart;
        current_pos = aStart;
        end_pos = aEnd;
        // 移動する距離
        length_pos = aEnd - aStart;
        // 各軸のステップ距離
        pos_per_step = aPosPerStep;
        // 各軸の速度
        pos_per_usec = length_pos / aTimeUsec;
        // 各軸の1ステップあたりの時間
        usec_per_step = std::abs( pos_per_step / pos_per_usec );
        //
        total_steps = aTimeUsec/usec_per_step;
        total_time = aTimeUsec;
        next_time_usec = usec_per_step;
    }
    /**
     * 次の位置を計算
     */
    void next() {
        step++;
        if( step<total_steps ) {
            current_time_usec = next_time_usec;
            next_time_usec = step * usec_per_step + usec_per_step;
            current_pos = start_pos + std::round( pos_per_usec * current_time_usec );
        } else {
            step = total_steps;
            current_time_usec = total_time;
            next_time_usec = total_time;
            current_pos = end_pos;
        }
    }
    /**
     * 次の位置が時間範囲に合致するかい？
     * @param t1
     * @param t2
     * @return 
     */
    bool hit( double t1, double t2 ) {
        return t1<=next_time_usec && next_time_usec <=t2;
    }
    /**
     * 次の時間へ進める
     * @param t1
     * @param t2
     * @return 
     */
    bool next( double t1, double t2 ) {
        if( hit( t1, t2 ) ) {
            next();
            return true;
        }
        return false;
    }
};
/**
 * 直線移動計算
 * 
 * @param zOut
 * @param zThread
 * @param zBlk
 */
void G01( std::ostream &zOut, GThread& zThread, GBlock& zBlk ) {

    int zCode = zBlk.getCode( );
    string zRawLine = zBlk.getLine( );
    // 直線移動
    // G00 最高速度で移動
    // G01 指定速度で移動

    // 開始位置
    pos_vct zPos_Start = zBlk.getCurrent( );
    // 終了位置
    pos_vct zPos_End = zBlk.getTarget( );
    // 送り速度
    double zPosPerUsec = getPosPerUsec( zOut, zThread, zBlk );
    if( !vct::is_ok( zPos_Start ) ) {
        zOut << "ERROR:invalid start pos " << endl;
        return;
    }

    if( !vct::is_ok( zPos_End ) ) {
        zOut << "ERROR:invalid end pos " << endl;
        return;
    }
    // 開始位置(実数化)
    double_vct zXYZ_Start = vct::from_pos( zPos_Start );
    double_vct zXYZ_End = vct::from_pos( zPos_End );
    // 移動ベクトル
    double_vct zXYZ_Vector = vct::diff( zXYZ_End, zXYZ_Start );
    zOut << " diff ";
    out_pos( zOut, zPos_Start );
    zOut << "->";
    out_pos( zOut, zPos_End );
    zOut << endl;
    
    // 移動距離を計算
    double zLength = vct::length( zXYZ_Vector );
    // 移動にかかる時間を計算
    long zTotalUsec = zLength / zPosPerUsec;

    // usecあたりの移動距離
    double_vct zVector = vct::norm( zXYZ_Vector, zTotalUsec );

    // 現時点の位置を初期化
    step_vct zCurrentStep = zThread.pos_to_step( zPos_Start );

    zOut << "move ";
    out_pos( zOut, zPos_Start );
    out_step( zOut, zCurrentStep );
    zOut << " to ";
    out_pos( zOut, zPos_End );
    step_vct end = zThread.pos_to_step( zPos_End );
    out_step( zOut, end );
    zOut << endl;

//    int zI[NUM_MOTORS];
//    double zDT[NUM_MOTORS];
//    for( int i=0; i<NUM_MOTORS; i++ ) {
//        zI[i] = 0;
//        long ds = end[i] - zCurrentStep[i];
//        if( ds > 0 ) {
//            zDT[i] = zTotalUsec/ds;
//        } else {
//            zDT[i] = zTotalUsec;
//        }
//    }
//    
    zThread.send_start_block( zRawLine, zCurrentStep );
    // 計算ステップ
    double zStepUsec = 10;
    // 移動ループ
    for( double zUsec = zStepUsec; !zThread.isAbort( ); zUsec += zStepUsec ) {
        bool update = false;
        if( zUsec < zTotalUsec ) {
            for( int i = 0; i < NUM_MOTORS; i++ ) {
                double zDoublePos = zXYZ_Start[i] + zVector[i] * zUsec;
                pos_t zPos = double_to_pos( zDoublePos );
                step_t zStep = zThread.pos_to_step( i, zPos );
                if( zCurrentStep[i] != zStep ) {
                    update = true;
                    zCurrentStep[i] = zStep;
                }
            }
        } else {
            zUsec = zTotalUsec;
            for( int i = 0; i < NUM_MOTORS; i++ ) {
                step_t zStep = zThread.pos_to_step( i, zPos_End[i] );
                if( zCurrentStep[i] != zStep ) {
                    update = true;
                    zCurrentStep[i] = zStep;
                }
            }
        }
        if( update || zUsec >= zTotalUsec ) {
            // stepが変わったら信号を送る
            zThread.send_step( zUsec, zCurrentStep );
        }
        if( zUsec >= zTotalUsec ) {
            break;
        }
    }
    
    
    cout << "length:"<<zLength <<"(pos)"<< endl;
    cout << "time:"<<zTotalUsec<<"(usec)"<<endl;
    
    double zSS[] = {300,200,100};
    
    line_move_data zMV[NUM_MOTORS];
    for( int i=0; i<NUM_MOTORS; i++ ) {
        zMV[i].init( zXYZ_Start[i], zXYZ_End[i], zSS[i], zTotalUsec );
    }
    
    cout << "-----------------" <<endl;
    double zMinimumTime_usec = 50.0;

    double zNextTime1 = 0.0;
    while( zNextTime1 < zTotalUsec ) {

        zNextTime1 = zTotalUsec;
        for( int i=0;i<NUM_MOTORS; i++ ) {
            if( zMV[i].next_time_usec < zNextTime1 ) {
                zNextTime1 = zMV[i].next_time_usec;
            }
        }
        double zNextTime2 = zNextTime1 + zMinimumTime_usec;

        for( int i=0;i<NUM_MOTORS; i++ ) {
            if( zMV[i].next( zNextTime1,zNextTime2) ) {
            }
        }
//        cout<< fixed << setprecision(4);
//        cout<<" t:"<<zNextTime1;
//        for( int i=0;i<NUM_MOTORS; i++ ) {
//            cout<< " "<<zMV[i].step<<"/"<<zMV[i].total_steps;
//        }
//        for( int i=0;i<NUM_MOTORS; i++ ) {
//            if( i==0 ) {
//                cout << " (";
//            } else {
//                cout << ",";
//            }
//            cout<<zMV[i].current_pos;
//        }
//        cout<<")";
//        cout<<endl;
    }
    for( int i = 0; i < NUM_MOTORS; i++ ) {
        xCurrentPos[i] = zPos_End[i];
    }
    zThread.send_end_block( zCurrentStep );

}

/**
 * 円弧移動計算
 * 
 * @param zOut
 * @param zThread
 * @param zBlk
 */
void G02( std::ostream &zOut, GThread& zThread, GBlock& zBlk ) {

    char zAddress = zBlk.getAddress( );
    int zCode = zBlk.getCode( );
    string zRawLine = zBlk.getLine( );

    // 円弧移動
    // G02 時計回りで移動
    // G03 反時計回りで移動
    bool is_cw = zCode == 2 ? zCCW : !zCCW;

    // 開始位置
    pos_vct zPos_Start = zBlk.getCurrent( );
    // 中心位置
    pos_vct zPos_Center = zBlk.getCenter( );
    // 終了位置
    pos_vct zPos_End = zBlk.getTarget( );

    if( !vct::is_ok( zPos_Start ) ) {
        zOut << "ERROR:invalid start pos " << endl;
        return;
    }

    if( !vct::is_ok( zPos_End ) ) {
        zOut << "ERROR:invalid end pos " << endl;
        return;
    }
    if( !vct::is_ok( zPos_Center ) ) {
        zOut << "ERROR:invalid center pos " << endl;
        return;
    }
    // 送り速度(mm/min)から pos/usec つまり um/usecに変換
    double zPosPerUsec = getPosPerUsec( zOut, zThread, zBlk );

    // 開始位置(実数化)
    double_vct zXYZ_Start = vct::from_pos( zPos_Start );
    // 中心位置(実数化)
    double_vct zXYZ_Center = vct::from_pos( zPos_Center );
    // 開始位置(実数化)
    double_vct zXYZ_End = vct::from_pos( zPos_End );

    double dx0 = zXYZ_Start[0] - zXYZ_Center[0];
    double dy0 = zXYZ_Start[1] - zXYZ_Center[1];
    double zR = sqrt( dx0 * dx0 + dy0 * dy0 );

    dx0 = dx0 / zR;
    dy0 = dy0 / zR;

    double dx1 = ( zXYZ_End[0] - zXYZ_Center[0] ) / zR;
    double dy1 = ( zXYZ_End[1] - zXYZ_Center[1] ) / zR;

    rad_t zRad_Start, zRad_End;
    if( is_cw ) {
        zRad_Start = angle::cw_to_rad( dx0, dy0 );
        zRad_End = angle::cw_to_rad( dx1, dy1 );
    } else {
        zRad_Start = angle::ccw_to_rad( dx0, dy0 );
        zRad_End = angle::ccw_to_rad( dx1, dy1 );
    }
    rad_t zRad_Diff = angle::rad_diff( zRad_End, zRad_Start );

    // 移動距離を計算
    double zLength = zR * zRad_Diff;
    // 移動にかかる時間を計算
    long zTotalUsec = zLength / zPosPerUsec;

    // usecあたりの移動角度
    rad_t zRadPerUsec = zRad_Diff / zTotalUsec;

    // 現時点の位置を初期化
    step_vct zCurrentStep = zThread.pos_to_step( zPos_Start );

    zThread.send_start_block( zRawLine, zCurrentStep );
    // 計算ステップ
    double zStepUsec = 10;
    // 移動ループ
    for( double zUsec = 0; !zThread.isAbort( ) && zUsec < zTotalUsec; zUsec += zStepUsec ) {
        bool update = false;
        if( zUsec < zTotalUsec ) {
            rad_t th = angle::normlz_rad( zRad_Start + zRadPerUsec * zUsec );
            for( int i = 0; i < 2; i++ ) {
                double f;
                if( is_cw ) {
                    if( i == 0 ) {
                        f = angle::cw_cos( th );
                    } else {
                        f = angle::cw_sin( th );
                    }
                } else {
                    if( i == 0 ) {
                        f = angle::ccw_cos( th );
                    } else {
                        f = angle::ccw_sin( th );
                    }
                }
                double xx = zXYZ_Center[i] + zR * f;
                pos_t zPos = double_to_pos( xx );
                step_t zStep = zThread.pos_to_step( i, zPos );
                if( zCurrentStep[i] != zStep ) {
                    update = true;
                    zCurrentStep[i] = zStep;
                }
            }
        } else {
            for( int i = 0; i < 2; i++ ) {
                step_t zStep = zThread.pos_to_step( i, zPos_End[i] );
                if( zCurrentStep[i] != zStep ) {
                    update = true;
                    zCurrentStep[i] = zStep;
                }
            }
        }
        if( update ) {
            // stepが変わったら信号を送る
            zThread.send_step( zUsec, zCurrentStep );
        }
        if( zUsec >= zTotalUsec ) {
            break;
        }
    }
    for( int i = 0; i < NUM_MOTORS; i++ ) {
        xCurrentPos[i] = zPos_End[i];
    }
    zThread.send_end_block( zCurrentStep );


}

/*
 * 
 */
int main( int argc, char** argv ) {

    std::ostream &zOut = std::cout;

    wiringPiSetup( );

    /* シグナルハンドラの設定 */
    if( signal( SIGUSR1, sig_handler ) == SIG_ERR ) {
        printf( "\ncan't catch SIGUSR1\n" );
    }
    if( signal( SIGHUP, sig_handler ) == SIG_ERR ) {
        printf( "\ncan't catch SIGHUP\n" );
    }
    if( signal( SIGQUIT, sig_handler ) == SIG_ERR ) {
        printf( "\ncan't catch SIGQUIT\n" );
    }
    if( signal( SIGILL, sig_handler ) == SIG_ERR ) {
        printf( "\ncan't catch SIGILL\n" );
    }
    if( signal( SIGKILL, sig_handler ) == SIG_ERR ) {
        printf( "\ncan't catch SIGKILL\n" );
    }
    if( signal( SIGSTOP, sig_handler ) == SIG_ERR ) {
        printf( "\ncan't catch SIGSTOP\n" );
    }
    if( signal( SIGTERM, sig_handler ) == SIG_ERR ) {
        printf( "\ncan't catch SIGTERM\n" );
    }


    config zConfig;
    zConfig.read( "testData/config" );
    zOut << "----dump----" << endl;
    zConfig.dump( "  " );

    step_vct zCurrentStep( NUM_MOTORS );
    pos_vct zCurrentPos( NUM_MOTORS );

    zOut << "--" << endl;
    GThread zThread;
    zThread.configure( zConfig );

    zOut << "--" << endl;

    GReader zTopGr;
    GReader *zReader = &zTopGr;
    vector<GReader*> zReaderStack;
    for( int i = 0; i < NUM_MOTORS; i++ ) {
        xCurrentPos[i] = zThread.getCurrentPos( i );
    }
    int zAbort = 0;
    for( int i = 0;; i++ ) {

        if( !zReaderStack.empty( ) && !check( zOut, zThread, &zAbort ) ) {
            zReader->close( );
            zReader = zReaderStack.back( );
            zReaderStack.pop_back( );
            continue;
        }

        GBlock zBlk = zReader->nextBlock( xCurrentPos );
        zBlk.out( zOut );
        if( zBlk.getAddress( ) == '\0' ) {
            zReader->close( );
            if( !zReaderStack.empty( ) ) {
                zReader = zReaderStack.back( );
                zReaderStack.pop_back( );
            } else {
                break;
            }
        }

        char zAddress = zBlk.getAddress( );
        int zCode = zBlk.getCode( );

        if( zAddress == 'G' || zAddress == 'M' ) {
            if( !check( zOut, zThread, &zAbort ) ) {
                zOut << "ERROR: ignore command" << endl;
                continue;
            }
        }
        if( zG02toG01 ) {
            if( zAddress == 'G' && ( zCode == 2 || zCode == 3 ) ) {
                zCode = 1;
            }
        }

        if( zBlk.getAddress( ) == 'G' && ( zCode == 0 || zCode == 1 ) ) {
            G01( zOut, zThread, zBlk );

        } else if( zBlk.getAddress( ) == 'G' && ( zCode == 2 || zCode == 3 ) ) {
            G02( zOut, zThread, zBlk );

        } else if( zBlk.getAddress( ) == '(' ) {

            if( zCode == CMD_HELP ) {
                zOut << "((help未実装))" << endl;

            } else if( zCode == CMD_CLEAR_ABORT ) {
                zOut << "((clear))" << endl;
                zAbort = 0;

            } else if( zCode == CMD_ADJUST ) {
                zOut << "((adjust))" << endl;
                zThread.adjust( zOut );

            } else if( zCode == CMD_SHOW ) {
                zOut << "((show))" << endl;
                zThread.show( zOut );

            } else if( zCode == CMD_CLEAR_ABORT ) {
                zOut << "((clear abort))" << endl;
                //zThread.pon();

            } else if( zCode == CMD_PON ) {
                zOut << "((pon))" << endl;
                zThread.pon( );

            } else if( zCode == CMD_POFF ) {
                zOut << "((poff))" << endl;
                zThread.poff( );

            } else if( zCode == CMD_SCAN ) {
                zOut << "((scan))" << endl;
                if( !zThread.is_power( ) ) {
                    zOut << "ERROR:power is off" << endl;
                } else if( zThread.isAbort( ) ) {
                    zOut << "ERROR:aborted" << endl;
                } else {
                    vector<string>zArgs = zBlk.getArgs( );
                    for( int i = 0; i < zArgs.size( ); i++ ) {
                        string a = zArgs[i];
                        for( int j = 0; j < a.size( ); j++ ) {
                            if( a[j] == 'x' ) {
                                zThread.scan_origin( 0, zOut );
                            } else if( a[j] == 'y' ) {
                                zThread.scan_origin( 1, zOut );
                            } else if( a[j] == 'z' ) {
                                zThread.scan_origin( 2, zOut );
                            }
                        }
                    }
                }

            } else if( zCode == CMD_SET_HOME ) {
                zOut << "((set home))" << endl;
                vector<string>zArgs = zBlk.getArgs( );
                int zNo = to_axis( zArgs, 1 );
                if( zNo >= 0 ) {
                    if( zThread.set_home( zNo, zOut ) ) {
                        zOut << "((set_home))ERROR:" << endl;
                    } else {
                        zOut << "((set_home))ERROR:" << endl;
                    }
                } else {
                    zOut << "((set_home))ERROR:axis not present" << endl;
                }

            } else if( zCode == CMD_SET_LIMIT ) {
                zOut << "((set limit))" << endl;
                vector<string>zArgs = zBlk.getArgs( );
                int zNo = to_axis( zArgs, 1 );
                if( zNo >= 0 ) {
                    if( zThread.set_home( zNo, zOut ) ) {
                        zOut << "((set limit))ERROR:" << endl;
                    } else {
                        zOut << "((set limit))ERROR:" << endl;
                    }
                } else {
                    zOut << "((set limit))ERROR:axis not present" << endl;
                }

            } else if( zCode == CMD_SET_ZERO ) {
                zOut << "((set zero))" << endl;
                vector<string>zArgs = zBlk.getArgs( );
                int zNo = to_axis( zArgs, 1 );
                if( zNo >= 0 ) {
                    if( zThread.set_zero( zNo, zOut ) ) {
                        zOut << "((set zero))OK:" << endl;
                    } else {
                        zOut << "((set zero))ERROR:" << endl;
                    }
                } else {
                    zOut << "((set zero))ERROR:axis not present" << endl;
                }

            } else if( zCode == CMD_MOVE ) {
                vector<string>zArgs = zBlk.getArgs( );
                int zNo = to_axis( zArgs, 1 );
                step_t zStep = str_to_long( zArgs, 2, INVALID_STEP );
                if( zNo >= 0 && zStep != INVALID_STEP ) {
                    if( zThread.move( zNo, zStep, zOut ) ) {
                        zOut << "((move))OK:" << endl;
                    } else {
                        zOut << "((move))ERROR:" << endl;
                    }
                } else {
                    zOut << "((move))ERROR:axis not present" << endl;
                }

            } else if( zCode == CMD_EXEC ) {
                vector<string>zArgs = zBlk.getArgs( );
                if( zArgs.size( ) > 1 ) {
                    string zFileName = zArgs[1];
                    GReader *zNew = new GReader( zFileName );
                    if( zNew->open( ) ) {
                        zOut << "((exec)) " << zFileName << endl;
                        zReaderStack.push_back( zReader );
                        zReader = zNew;
                    } else {
                        zOut << "((exec))ERROR:can not open " << zFileName << endl;
                    }
                } else {
                    zOut << "((exec))ERROR:file not present" << endl;
                }

            } else if( zCode == CMD_SCALE ) {
                zOut << "((scale))" << endl;
                vector<string>zArgs = zBlk.getArgs( );
                if( zArgs.size( ) > 1 ) {
                    //zScale = str_to_double( zArgs[1], zScale );
                    //zOut << " scale x " << zScale << endl;
                }

            } else if( zCode == CMD_OFFSET ) {
                zOut << "((scale))" << endl;
                vector<string>zArgs = zBlk.getArgs( );
                if( zArgs.size( ) > 1 ) {
                    long zOffsetx = str_to_long( zArgs[1], 0 );
                    zOut << " offset x " << zOffsetx << endl;
                    //zOffset[0] = zOffsetx;
                    //zOffset[1] = zOffsetx;
                }

            } else if( zCode == CMD_G02TOG01 ) {
                zG02toG01 = !zG02toG01;
                zOut << "((G02toG01))" << zG02toG01 << endl;

            } else if( zCode == CMD_CCW ) {
                zCCW = !zCCW;
                zOut << "((G02toG01))" << zCCW << endl;

            } else if( zCode == CMD_DBGWAIT ) {
                vector<string>zArgs = zBlk.getArgs( );
                if( zArgs.size( ) > 1 ) {
                    long zOffsetx = str_to_long( zArgs[1], zThread.getDbgWait( ) );
                    zThread.setDbgWait( zOffsetx );
                }
                zOut << "((DbgWait))" << zThread.getDbgWait( ) << endl;
            }
            for( int i = 0; i < NUM_MOTORS; i++ ) {
                xCurrentPos[i] = zThread.getCurrentPos( i );
            }
        }

    }

    return 0;
}

