/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GBlock.cpp
 * Author: maeda
 * 
 * Created on 2022年2月24日, 7:34
 */

/*
M00	プログラムストップ
M01	オプショナルストップ
M02	プログラム終了
M03	主軸正転
M04	主軸逆転
M05	主軸停止
M06	工具交換
M08	クーラントの吐出
M09	クーラントの吐出停止
M19	主軸定位置停止
M20	自動電源しゃ断
M30	プログラム終了と頭出し
M33	工具収納
M98	サブプログラム呼び出し
M99	サブプログラム終了
 */

/*
G00	01	位置決め
G01	01	直線補間
G02	01	円弧補間/ヘリカル補間　CW
G03	01	円弧補間/ヘリカル補間　CCW
G04	00	ドゥエル、イグザクトストップ
G05	00	高速サイクル加工
G08	00	先行制御
G09	00	イグザクトストップ
G10	00	データ設定
G11	00	データ設定モードキャンセル
G15	17	極座標指令キャンセル
G16	17	極座標指令
G17	02	XY平面
G18	02	ZX平面
G19	02	YZ平面
G20	06	インチ入力
G21	06	メトリック入力
G22	04	ストアードストロークチェックオン
G23	04	ストアードストロークチェックオフ
G27	00	リファレンス点復帰チェック
G28	00	リファレンス点への自動復帰
G29	00	リファレンス点からの自動復帰
G30	00	第2、第3、第4リファレンス点復帰
G31	00	スキップ機能
G33	01	ねじ切り
G37	00	工具長自動測定
G39	00	コーナオフセット円弧補間
G40	07	工具径補正キャンセル
G41	07	工具径補正左
G42	07	工具径補正右
G43	08	工具長補正＋
G44	08	工具長補正−
G45	00	工具位置オフセット　伸張
G46	00	工具位置オフセット　縮小
G47	00	工具位置オフセット　2倍伸張
G48	00	工具位置オフセット　2倍縮小
G49	08	工具長補正キャンセル
G50	11	スケーリングキャンセル
G51	11	スケーリング
G52	00	ローカル座標系設定
G53	00	機械座標系選択
G54	14	ワーク座標系1選択
G55	14	ワーク座標系2選択
G56	14	ワーク座標系3選択
G57	14	ワーク座標系4選択
G58	14	ワーク座標系5選択
G59	14	ワーク座標系6選択
G60	00	一方向位置決め
G61	15	イグザクトストップモード
G62	15	自動コーナオーバライドモード
G63	15	タッピングモード
G64	15	切削モード
G65	00	マクロ呼び出し
G66	12	マクロモーダル呼び出し
G67	12	マクロモーダル呼び出しキャンセル
G68	16	座標回転
G69	16	座標回転キャンセル
G73	09	ペックドリリングサイクル
G74	09	逆タッピングサイクル
G75	01	プランジ研削サイクル（0-GSC）
G76	09	ファインボーリングサイクル
G77	01	プランジ直接定寸研削サイクル（0-GSC)
G78	01	連続送り平研削サイクル（0-GSC)
G79	01	間欠送り平研削サイクル（0-GSC)
G80	09	固定サイクルキャンセル/外部動作機能キャンセル
G81	09	ドリルサイクル、スポットボーリング/外部動作機能
G82	09	ドリルサイクル、カウンターボーリングサイクル
G83	09	ペックドリリングサイクル
G84	09	タッピングサイクル
G85	09	ボーリングサイクル
G86	09	ボーリングサイクル
G87	09	バックボーリングサイクル
G88	09	ボーリングサイクル
G89	09	ボーリングサイクル
G90	03	アブソリュート指令
G91	03	インクレメンタル指令
G92	00	ワーク座標系の変更／主軸最高回転数クランプ
G94	05	毎分送り
G95	05	毎回転送り
G96	13	周速一定制御
G97	13	周速一定制御キャンセル
G98	10	固定サイクルイニシャルレベル復帰
G99	10	固定サイクルR点レベル復帰
G107	00	円筒補間
G150	19	法線方向制御キャンセルモード
G151	19	法線方向制御左側オン
G152	19	法線方向制御右側オン
G160	20	インフィード制御機能キャンセル（0-GSC)
G161	20	インフィード制御機能（0-GSC) 
 */

#include "GBlock.h"

GBlock::GBlock() {
    mLineNo = 0;
    mRawLine = "";
    mAddress = 0;
    mCode = 0;
    for( int i=0; i<3; i++ ) {
        mCurrentPos[i] = INVALID_POS;
        mXYZ[i] = INVALID_POS;
        mIJK[i] = INVALID_POS;
    }
    mFeed = 0;
}
GBlock::GBlock( int aLineNo, std::string aRawLine, address_t aAddress, code_t aCode, pos_t aCurrentPos[] ) {
    mLineNo = aLineNo;
    mRawLine = aRawLine;
    mAddress = aAddress;
    mCode = aCode;
    for( int i=0; i<3; i++ ) {
        mCurrentPos[i] = aCurrentPos[i];
        mXYZ[i] = INVALID_POS;
        mIJK[i] = INVALID_POS;
    }
    mFeed = 0;
}
GBlock::GBlock( int aLineNo, std::string aRawLine, address_t aAddress, code_t aCode, pos_t aCurrentPos[], feed_t aFeed ) {
    mLineNo = aLineNo;
    mRawLine = aRawLine;
    mAddress = aAddress;
    mCode = aCode;
    for( int i=0; i<3; i++ ) {
        mCurrentPos[i] = aCurrentPos[i];
        mXYZ[i] = INVALID_POS;
        mIJK[i] = INVALID_POS;
    }
    mFeed = aFeed;
}
GBlock::GBlock( int aLineNo, std::string aRawLine, address_t aAddress, code_t aCode, pos_t aCurrentPos[], pos_t aXYZ[], feed_t aFeed ) {
    mLineNo = aLineNo;
    mRawLine = aRawLine;
    mAddress = aAddress;
    mCode = aCode;
    for( int i=0; i<3; i++ ) {
        mCurrentPos[i] = aCurrentPos[i];
        mXYZ[i] = aXYZ[i];
        mIJK[i] = INVALID_POS;
    }
    mFeed = aFeed;
}
GBlock::GBlock( int aLineNo, std::string aRawLine, address_t aAddress, code_t aCode, pos_t aCurrentPos[], pos_t aXYZ[], pos_t aIJK[], feed_t aFeed ) {
    mLineNo = aLineNo;
    mRawLine = aRawLine;
    mAddress = aAddress;
    mCode = aCode;
    for( int i=0; i<3; i++ ) {
        mCurrentPos[i] = aCurrentPos[i];
        mXYZ[i] = aXYZ[i];
        mIJK[i] = aIJK[i];
    }
    mFeed = aFeed;
}

GBlock::GBlock( const GBlock& orig ) {
}

GBlock::~GBlock( ) {
}

void GBlock::out( std::ostream &aOut ) {
    if( mLineNo > 0 ) {
        aOut << '[' << mLineNo << ']' <<mRawLine <<std::endl;
    }
    if( mAddress == '\0' ) {
        aOut << "[EOF]";
    } else {
        out_code( aOut, mAddress, mCode );

        for( int i=0; i<NUM_MOTORS; i++ ) {
            if( mXYZ[i] != INVALID_POS ) {
                aOut << ' ' << (char)('X'+i);
                out_pos( aOut, mXYZ[i] );
            }
        }
        for( int i=0; i<3; i++ ) {
            if( mIJK[i] != INVALID_POS ) {
                aOut << ' ' << (char)('I'+i);
                out_pos( aOut, mIJK[i] );
            }
        }
        if( mFeed >0 ) {
            aOut << ' ' << 'F';
            out_feed( aOut, mFeed );
        }
    }
    aOut << std::endl;
}

vector<string> GBlock::getArgs() {
    vector<string> aa = str::split( mRawLine );
    return aa;
}





int to_cmd( string aCmd ) {
    for( int i=0; MG_CMD_MAP[i].no>=0 ; i++ ) {
        if( MG_CMD_MAP[i].cmd == aCmd ) {
            return MG_CMD_MAP[i].no;
        }
    }
    return CMD_NOT_FOUND;
}

string from_cmd( int aCmd ) {
    for( int i=0; MG_CMD_MAP[i].no>=0 ; i++ ) {
        if( MG_CMD_MAP[i].no == aCmd ) {
            return MG_CMD_MAP[i].cmd;
        }
    }
    return "";
}
