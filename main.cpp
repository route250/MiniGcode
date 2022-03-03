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
void sig_handler(int signo) {
    if (signo == SIGUSR1) {
        printf("received SIGUSR1\n");
    } else if (signo == SIGHUP) {
        printf("received SIGHUP\n");
    } else if (signo == SIGQUIT) {
        printf("received SIGQUIT\n");
    } else if (signo == SIGILL) {
        printf("received SIGILL\n");
    } else if (signo == SIGKILL) {
        printf("received SIGKILL\n");
    } else if (signo == SIGSTOP) {
        printf("received SIGSTOP\n");
    } else if (signo == SIGTERM) {
        printf("received SIGTERM\n");
    }
}

bool check( std::ostream &zOut, GThread& zThread, int *zStatus ) {
    if( !zThread.is_power() ) {
        if( *zStatus != 4 ) {
            zOut<<"ERROR: power is off"<<endl;
            *zStatus = 4;
        }
        return false;
    } else if( !zThread.isDoneHoming() ) {
        if( *zStatus != 2 ) {
            zOut<<"ERROR: Homing is incomplite"<<endl;
            *zStatus = 2;
        }
        return false;
    } else if( zThread.isAbort() ) {
        if( *zStatus != 1 ) {
            zOut<<"ERROR: detect abort signal"<<endl;
        }
        *zStatus = 1;
        return false;
    }
    return *zStatus == 0;
}

/*
 * 
 */
int main( int argc, char** argv ) {
    
    wiringPiSetup( );

    /* シグナルハンドラの設定 */
    if (signal(SIGUSR1, sig_handler) == SIG_ERR) {
        printf("\ncan't catch SIGUSR1\n");
    }
    if (signal(SIGHUP, sig_handler) == SIG_ERR) {
        printf("\ncan't catch SIGHUP\n");
    }
    if (signal(SIGQUIT, sig_handler) == SIG_ERR) {
        printf("\ncan't catch SIGQUIT\n");
    }
    if (signal(SIGILL, sig_handler) == SIG_ERR) {
        printf("\ncan't catch SIGILL\n");
    }
    if (signal(SIGKILL, sig_handler) == SIG_ERR) {
        printf("\ncan't catch SIGKILL\n");
    }
    if (signal(SIGSTOP, sig_handler) == SIG_ERR) {
        printf("\ncan't catch SIGSTOP\n");
    }
    if (signal(SIGTERM, sig_handler) == SIG_ERR) {
        printf("\ncan't catch SIGTERM\n");
    }
    
    config zConfig;
    zConfig.read( "testData/config" );
    std::cout <<"----dump----" << std::endl;
    zConfig.dump("  ");

    step_vct zCurrentStep(NUM_MOTORS);
    pos_vct zCurrentPos(NUM_MOTORS);

    std::cout <<"--"<<std::endl;
    GThread zThread;
    zThread.configure( zConfig );

    std::ostream &zOut = std::cout;
    zOut << "--" << std::endl;
    
    GReader zTopGr;
    GReader *zReader = &zTopGr;
    bool zG02toG01 = false;
    bool zCCW = false;
    vector<GReader*> zReaderStack;

    feed_t zMaxFeed = 800000; // 800mm/min
    pos_vct xCurrentPos(NUM_MOTORS);
    double zScale = 1.0;
    pos_vct zOffset(NUM_MOTORS);
    for( int i=0; i<NUM_MOTORS; i++ ) {
        xCurrentPos[i] = zThread.getCurrentPos(i, zScale, zOffset[i] );
    }
    int zAbort = 0;
    for( int i=0; ; i++ ) {
        
        if( !zReaderStack.empty() && !check( zOut, zThread, &zAbort)) {
            zReader->close();
            zReader = zReaderStack.back();
            zReaderStack.pop_back();
            continue;
        }

        GBlock zBlk = zReader->nextBlock(xCurrentPos);
        zBlk.out( zOut );
        if( zBlk.getAddress() == '\0' ) {
            zReader->close();
            if( !zReaderStack.empty() ) {
                zReader = zReaderStack.back();
                zReaderStack.pop_back();
            } else {
                break;
            }
        }
        
        char zAddress = zBlk.getAddress();
        int zCode = zBlk.getCode();
        string zRawLine = zBlk.getLine();
        if( zAddress=='G' || zAddress=='M' ) {
            if( !check( zOut, zThread, &zAbort) ) {
                zOut<<"ERROR: ignore command"<<endl;
                continue;
            }
        }
        if( zG02toG01 ) {
            if( zAddress=='G' && ( zCode == 2 || zCode == 3 ) ) {
                zCode = 1;
            }
        }
        
        if( zBlk.getAddress()=='G' && ( zCode == 0 || zCode == 1 ) ) {
            // 直線移動
            // G00 最高速度で移動
            // G01 指定速度で移動
            
            // 開始位置
            pos_vct zPos_Start = zBlk.getCurrent();
            // 終了位置
            pos_vct zPos_End = zBlk.getTarget();
            // 送り速度
            double zFeed = zMaxFeed;
            if( zCode!=0 ) {
                zFeed = (double)zBlk.getFeed();
                if( zFeed <= 0 ) {
                    std::cout <<"ERROR:feed=0.0 use 400.0"<<std::endl;
                    zFeed = 400.;
                }
            }
            if( vct::is_ok(zPos_Start) && vct::is_ok(zPos_End ) ) {
                // 送り速度(pos/min)から pos/usec つまり um/usecに変換
                //double x = (zFeed1)/60.0/1000.0/1000.0;
                double zPosPerUsec = zFeed/60.0/1000.0/1000.0; // speed (pos/usec)

                // 開始位置(実数化)
                double_vct zXYZ_Start = vct::from_pos( zPos_Start );
                double_vct zXYZ_End = vct::from_pos( zPos_End );
                // 移動ベクトル
                double_vct zXYZ_Vector = vct::diff( zXYZ_End, zXYZ_Start );
                std::cout <<" diff ";
                out_pos( std::cout, zPos_Start ); std::cout <<"->"; out_pos( std::cout, zPos_End );
                std::cout <<std::endl;
                // 移動距離を計算
                double zLength = vct::length( zXYZ_Vector );
                // 移動にかかる時間を計算
                long zTotalUsec = zLength / zPosPerUsec;

                // usecあたりの移動距離
                double_vct zVector = vct::norm( zXYZ_Vector, zTotalUsec );

                // 現時点の位置を初期化
                step_vct zCurrentStep = zThread.pos_to_step( zPos_Start, zScale, zOffset );

                std::cout << "move ";
                out_pos(std::cout,zPos_Start);out_step(std::cout,zCurrentStep);
                std::cout << " to ";
                out_pos(std::cout,zPos_End);
                step_vct end = zThread.pos_to_step( zPos_End, zScale, zOffset );
                out_step(std::cout,end);
                std::cout << std::endl;

                zThread.send_start_block( zRawLine, zCurrentStep );
                // 計算ステップ
                double zStepUsec = 10;
                // 移動ループ
                for( double zUsec = zStepUsec; !zThread.isAbort(); zUsec+=zStepUsec ) {
                    bool update = false;
                    if( zUsec < zTotalUsec ) {
                        for( int i=0; i<NUM_MOTORS; i++ ) {
                            double zDoublePos = zXYZ_Start[i] + zVector[i] * zUsec;
                            pos_t zPos = double_to_pos( zDoublePos );
                            step_t zStep = zThread.pos_to_step( i, zPos, zScale, zOffset[i] );
                            if( zCurrentStep[i] != zStep ) {
                                update = true;
                                zCurrentStep[i] = zStep;
                            }
                        }
                    } else {
                        zUsec = zTotalUsec;
                        for( int i=0; i<NUM_MOTORS; i++ ) {
                            step_t zStep = zThread.pos_to_step( i, zPos_End[i], zScale, zOffset[i] );
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
                for( int i=0; i<NUM_MOTORS; i++ ) {
                    xCurrentPos[i] = zPos_End[i];
                }
                zThread.send_end_block( zCurrentStep );
            } else {
                std::cout << "ERROR" << std::endl;
            }
            
        } else if( zBlk.getAddress()=='G' && ( zCode == 2 || zCode == 3 ) ) {
            
            // 円弧移動
            // G02 時計回りで移動
            // G03 反時計回りで移動
            bool is_cw = zCode==2 ? zCCW : !zCCW;
            
            // 開始位置
            pos_vct zPos_Start = zBlk.getCurrent();
            // 中心位置
            pos_vct zPos_Center = zBlk.getCenter();
            // 終了位置
            pos_vct zPos_End = zBlk.getTarget();

            // 送り速度
            double zFeed = zMaxFeed;
            if( zCode!=0 ) {
                zFeed = (double)zBlk.getFeed();
                if( zFeed <= 0 ) {
                    std::cout <<"ERROR:feed=0.0 use 400.0"<<std::endl;
                    zFeed = 400.;
                }
            }
            if( vct::is_ok( zPos_Start) && vct::is_ok( zPos_End) && vct::is_ok( zPos_Center ) ) {
                // 送り速度(mm/min)から pos/usec つまり um/usecに変換
                //double x = (zFeed*1000.0)/60.0/1000.0/1000.0;
                double zPosPerUsec = zFeed/60.0/1000.0; // speed (pos/usec)

                // 開始位置(実数化)
                double_vct zXYZ_Start = vct::from_pos( zPos_Start );
                // 中心位置(実数化)
                double_vct zXYZ_Center = vct::from_pos( zPos_Center );
                // 開始位置(実数化)
                double_vct zXYZ_End = vct::from_pos( zPos_End );

                double dx0 = zXYZ_Start[0] - zXYZ_Center[0];
                double dy0 = zXYZ_Start[1] - zXYZ_Center[1];
                double zR = sqrt( dx0*dx0 + dy0*dy0 );

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
                step_vct zCurrentStep2 = zThread.pos_to_step( zPos_Start, zScale, zOffset );

                zThread.send_start_block( zRawLine, zCurrentStep2 );
                // 計算ステップ
                double zStepUsec = 10;
                // 移動ループ
                for( double zUsec = 0; !zThread.isAbort() && zUsec<zTotalUsec; zUsec+=zStepUsec ) {
                    bool update = false;
                    if( zUsec < zTotalUsec ) {
                        rad_t th = angle::normlz_rad( zRad_Start + zRadPerUsec * zUsec );
                        for( int i=0; i<2; i++ ) {
                            double f;
                            if( is_cw ) {
                                if( i==0 ) {
                                    f = angle::cw_cos(th);
                                } else {
                                    f = angle::cw_sin(th);
                                }
                            } else {
                                if( i==0 ) {
                                    f = angle::ccw_cos(th);
                                } else {
                                    f = angle::ccw_sin(th);
                                }
                            }
                            double xx = zXYZ_Center[i] + zR * f;
                            pos_t zPos = double_to_pos( xx );
                            step_t zStep = zThread.pos_to_step( i, zPos, zScale, zOffset[i] );
                            if( zCurrentStep2[i] != zStep ) {
                                update = true;
                                zCurrentStep2[i] = zStep;
                            }
                        }
                    } else {
                        for( int i=0; i<2; i++ ) {
                            step_t zStep = zThread.pos_to_step( i, zPos_End[i], zScale, zOffset[i] );
                            if( zCurrentStep2[i] != zStep ) {
                                update = true;
                                zCurrentStep2[i] = zStep;
                            }
                        }
                    }
                    if( update ) {
                        // stepが変わったら信号を送る
                        zThread.send_step( zUsec, zCurrentStep2 );
                    }
                    if( zUsec >= zTotalUsec ) {
                        break;
                    }
                }
                for( int i=0; i<NUM_MOTORS; i++ ) {
                    xCurrentPos[i] = zPos_End[i];
                }
                zThread.send_end_block( zCurrentStep );

            } else {
                std::cout << "ERROR:" << std::endl;
            }
        } else if( zBlk.getAddress()=='(' ) {

            if ( zCode == CMD_HELP )  {
                cout << "((help未実装))" << std::endl;
                
            } else if(  zCode == CMD_CLEAR_ABORT ) {
                cout << "((clear))" << std::endl;
                zAbort = 0;

            } else if(  zCode == CMD_ADJUST ) {
                cout << "((adjust))" << std::endl;
                zThread.adjust(zOut);

            } else if(  zCode == CMD_SHOW ) {
                cout << "((show))" << std::endl;
                zThread.show( cout, zScale, zOffset );
                
            } else if(  zCode == CMD_CLEAR_ABORT ) {
                cout << "((clear abort))" << std::endl;
                //zThread.pon();

            } else if(  zCode == CMD_PON ) {
                cout << "((pon))" << std::endl;
                zThread.pon();
            } else if(  zCode == CMD_POFF ) {
                cout << "((poff))" << std::endl;
                zThread.poff();

            } else if(  zCode == CMD_SCAN ) {
                cout << "((scan))" << std::endl;
                if( !zThread.is_power() ) {
                    zOut<<"ERROR:power is off"<<endl;
                } else if( zThread.isAbort() ) {
                    zOut<<"ERROR:aborted"<<endl;
                } else {
                    vector<string>zArgs = zBlk.getArgs();
                    for( int i=0; i<zArgs.size(); i++ ) {
                        string a = zArgs[i];
                        for( int j=0; j<a.size(); j++ ) {
                            if( a[j] == 'x' ) {
                                zThread.scan_origin( 0,cout);
                            } else if( a[j] == 'y' ) {
                                zThread.scan_origin( 1,cout);
                            } else if( a[j] == 'z' ) {
                                zThread.scan_origin( 2,cout);
                            }
                        }
                    }
                }
                
            } else if(  zCode == CMD_SET_HOME ) {
                cout << "((set home))" << std::endl;
                vector<string>zArgs = zBlk.getArgs();
                int zNo = to_axis( zArgs, 1 );
                if( zNo>=0 ) {
                    if( zThread.set_home(zNo,zOut) ) {
                        zOut<<"((set_home))ERROR:"<<endl;
                    } else {
                        zOut<<"((set_home))ERROR:"<<endl;
                    }
                } else {
                    zOut<<"((set_home))ERROR:axis not present"<<endl;
                }
            } else if(  zCode == CMD_SET_LIMIT ) {
                cout << "((set limit))" << std::endl;
                vector<string>zArgs = zBlk.getArgs();
                int zNo = to_axis( zArgs, 1 );
                if( zNo>=0 ) {
                    if( zThread.set_home(zNo,zOut) ) {
                        zOut<<"((set limit))ERROR:"<<endl;
                    } else {
                        zOut<<"((set limit))ERROR:"<<endl;
                    }
                } else {
                    zOut<<"((set limit))ERROR:axis not present"<<endl;
                }
            } else if(  zCode == CMD_SET_ZERO ) {
                cout << "((set zero))" << std::endl;
                vector<string>zArgs = zBlk.getArgs();
                int zNo = to_axis( zArgs, 1 );
                if( zNo>=0 ) {
                    if( zThread.set_zero(zNo,zOut) ) {
                        zOut<<"((set zero))OK:"<<endl;
                    } else {
                        zOut<<"((set zero))ERROR:"<<endl;
                    }
                } else {
                    zOut<<"((set zero))ERROR:axis not present"<<endl;
                }

            } else if(  zCode == CMD_MOVE ) {
                vector<string>zArgs = zBlk.getArgs();
                int zNo = to_axis( zArgs, 1 );
                step_t zStep = str_to_long( zArgs, 2, INVALID_STEP );
                if( zNo>=0 && zStep != INVALID_STEP ) {
                    if( zThread.move(zNo,zStep,zOut) ) {
                        zOut<<"((move))OK:"<<endl;
                    } else {
                        zOut<<"((move))ERROR:"<<endl;
                    }
                } else {
                    zOut<<"((move))ERROR:axis not present"<<endl;
                }

            } else if(  zCode == CMD_EXEC ) {
                vector<string>zArgs = zBlk.getArgs();
                if( zArgs.size()>1 ) {
                    string zFileName = zArgs[1];
                    GReader *zNew = new GReader(zFileName);
                    if( zNew->open() ) {
                        zOut << "((exec)) "<<zFileName<<endl;
                        zReaderStack.push_back( zReader );
                        zReader = zNew;
                    } else {
                        zOut << "((exec))ERROR:can not open "<<zFileName<<endl;
                    }
                } else {
                    zOut<<"((exec))ERROR:file not present"<<endl;
                }
                
            } else if(  zCode == CMD_SCALE ) {
                cout << "((scale))" << std::endl;
                vector<string>zArgs = zBlk.getArgs();
                if( zArgs.size()>1 ) {
                    zScale = str_to_double( zArgs[1], zScale );
                    cout << " scale x " << zScale << std::endl;
                }
            } else if(  zCode == CMD_OFFSET ) {
                cout << "((scale))" << std::endl;
                vector<string>zArgs = zBlk.getArgs();
                if( zArgs.size()>1 ) {
                    long zOffsetx = str_to_long( zArgs[1], 0 );
                    cout << " offset x " << zOffsetx << std::endl;
                    zOffset[0] = zOffsetx;
                    zOffset[1] = zOffsetx;
                }
            } else if( zCode == CMD_G02TOG01 ) {
                zG02toG01 = !zG02toG01;
                cout << "((G02toG01))" <<zG02toG01 << std::endl;
            } else if( zCode == CMD_CCW ) {
                zCCW = !zCCW;
                cout << "((G02toG01))" <<zCCW << std::endl;
            } else if( zCode == CMD_DBGWAIT ) {
                vector<string>zArgs = zBlk.getArgs();
                if( zArgs.size()>1 ) {
                    long zOffsetx = str_to_long( zArgs[1], zThread.getDbgWait() );
                    zThread.setDbgWait(zOffsetx);
                }
                cout << "((DbgWait))" << zThread.getDbgWait() << std::endl;
            }
            for( int i=0; i<NUM_MOTORS; i++ ) {
                xCurrentPos[i] = zThread.getCurrentPos(i, zScale, zOffset[i] );
            }
        }

    }

    return 0;
}

