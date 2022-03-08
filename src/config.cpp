/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   config.cpp
 * Author: maeda
 * 
 * Created on 2022年2月25日, 18:10
 */

#include "config.h"

config::config( ) {
}
config::config( string aKey ) {
    mKey = aKey;
}

config::config( string aKey, string aValue ) {
    mKey = aKey;
    mValue = aValue;
}

config::config( const config& orig ) {
}

config::~config( ) {
    while( !mChilds.empty() ) {
        config *p = mChilds.back();
        mChilds.pop_back();
        delete p;
    }
}

void config::set( string aValue ) {
    mValue = aValue;
}

void config::append( config *a ) {
    mChilds.push_back( a );
}

string config::getText() {
    return mValue;
}

double config::getNumber( double aDefault ) {
    try{
        string zText = getText();
        double result = std::stod( zText );
        return result;
    }catch(std::invalid_argument &ex){
        return aDefault;
    }
}

string config::getText( string aPath ) {
    int p = aPath.find("/");
    string zKey;
    string zNextPath;
    if( p<0 ) {
        zKey = aPath;
        zNextPath = "";
    } else {
        zKey = aPath.substr(0,p);
        zNextPath = aPath.substr(p+1);
    }
    //std::cout<<"getText "<<aPath<<" -> "<<zKey << "," <<zNextPath <<std::endl;
    for( int i=0; i<mChilds.size(); i++ ) {
        if( str::tolower(mChilds[i]->mKey) == str::tolower(zKey) ) {
            if( zNextPath.empty() ) {
                return mChilds[i]->getText();
            } else {
                return mChilds[i]->getText(zNextPath);
            }
        }
    }
    return "";
}

double config::getNumber( string aPath, double aDefault ) {
    try{
        string zText = getText( aPath );
        //std::cout<<"getNumber "<<aPath<<" text "<<zText <<std::endl;
        double result = std::stod( zText );
        //std::cout<<"getNumber "<<aPath<<" number "<<result <<std::endl;
        return result;
    }catch(std::invalid_argument &ex){
        //std::cout<<"getNumber "<<aPath<<" ERROR "<<std::endl;
        return aDefault;
    }
}

bool config::getBool( string aPath, bool aDefault ) {
    try{
        string zText = str::tolower( getText( aPath ) );
        if( zText == "true" || zText == "on" ) {
            return true;
        }
        if( zText == "false" || zText == "off" ) {
            return false;
        }
    }catch(std::invalid_argument &ex){
    }
    return aDefault;
}

config *config::getChild( string aPath ) {
    int p = aPath.find("/");
    string zKey;
    string zNextPath;
    if( p<0 ) {
        zKey = aPath;
        zNextPath = "";
    } else {
        zKey = aPath.substr(0,p);
        zNextPath = aPath.substr(p+1);
    }
    for( int i=0; i<mChilds.size(); i++ ) {
        if( str::tolower(mChilds[i]->mKey) == str::tolower(zKey) ) {
            if( zNextPath.empty() ) {
                return mChilds[i];
            } else {
                return mChilds[i]->getChild(zNextPath);
            }
        }
    }
    return nullptr;
}

bool config::read( std::string aFileName ) {
    int zLineNo = 0;
    std::ifstream zIn;
    zIn.open( aFileName );
    if( !zIn.is_open( ) ) {
        return false;
    }
    bool zRet = true;
    std::string zBuffer, zLine;

    config *zCurrent = this;
    vector<config*> zStack;
    while( std::getline( zIn, zBuffer ) ) {
        zLineNo++;
        zLine = zBuffer;
        //std::cout << "[#1]" << zBuffer <<std::endl;
        //std::cout << "[#2]" << zLine <<std::endl;
        // コメント除去
        std::string::size_type sp = zLine.find( '#' );
        if( sp != std::string::npos ) {
            zLine.erase( sp );
        }
        // 先頭空白飛ばし
        int s = 0;
        while( s<zLine.size() && std::isblank ( zLine[s] ) ) {
            s++;
        }
        // 項目名切り出し
        int e = s;
        while( e<zLine.size() && ( !std::isblank ( zLine[e] ) || zLine[e] == '{' ) ) {
            e++;
        }
        if( s==e ) {
            continue;
        }
        //std::cout << "[#1]" << zLine <<std::endl;
        //std::cout << "    " << s <<" " << e <<std::endl;
        // 分解
        string zKey = (e>s) ? zLine.substr(s,(e-s)) : "";
        string zValue = str::trim( zLine.substr(e) );
        //std::cout << "    key:\"" <<zKey<<"\"";
        //std::cout << "  value:\"" <<zValue<<"\""<<std::endl;

        if( !zKey.empty() ) {
            if( zValue.empty() && zKey == "}" ) {
                // 階層上がる
                if( zStack.size()>0 ) {
                    zCurrent = zStack.back();
                    zStack.pop_back();
                } else {
                    zRet = false;
                    std::cerr << "ERROR:{}の組み合わせが合わない" << std::endl;
                }
            } else if( zValue == "{" ) {
                // 階層下がる
                config *zChild = new config( zKey );
                zCurrent->append( zChild );
                zStack.push_back( zCurrent );
                zCurrent = zChild;
            } else  {
                // 値を記録
                config *zChild = new config( zKey, zValue );
                zCurrent->append( zChild );
            }
        }
    }
    zIn.close( );
    return zRet;
}

void config::dump( string aIndent ) {
    std::cout << aIndent << mKey ;
    if( !mValue.empty() ) {
        std::cout << " = " << mValue;
    }
    if( mChilds.size()>0 ) {
        std::cout << " {" << std::endl;
        for( config *zChild : mChilds ) {
            zChild->dump( aIndent + "    " );
        }
        std::cout << aIndent << "}";
    }
    std::cout << std::endl;
}