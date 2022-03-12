/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MGmotor.h
 * Author: maeda
 *
 * Created on 2022年3月10日, 10:37
 */

#ifndef MG_STEPPER_H
#define MG_STEPPER_H

#include "Gunit.h"
#include "MGmath.h"

class MGstepper {
public:
    MGstepper();
    MGstepper(const MGstepper& orig);
    virtual ~MGstepper();
private:

};

#endif /* MG_STEPPER_H */

