/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   config.h
 * Author: maeda
 *
 * Created on 2022年2月25日, 18:10
 */

#ifndef CONFIG_H
#define CONFIG_H
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

#include "Gunit.h"

using std::string;
using std::vector;

class config {
private:
    string mKey;
    string mValue;
    vector<config*> mChilds;
public:
    config();
    config( string aKey );
    config( string aKey, string aValue );
    config(const config& orig);
    virtual ~config();
public:
    bool read( std::string aFileName  );
    void dump( string aIndent );
    void set( string aValue );
    void append( config *aValue );
    string getText();
    double getNumber( double aDefault );
    string getText( string aKey );
    double getNumber( string aKey, double aDefault );
    bool getBool( string aKey, bool aDefault );
    config *getChild( string aKey );
private:

};

#endif /* CONFIG_H */

