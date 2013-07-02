/*****************************************************************************
 * Copyright (c) 2013, Worcester Polytechnic Institute                       *
 * All rights reserved.                                                      *
 *                                                                           *
 * Author: Nicholas Alunni <nick.alunni@gmail.com>                           *
 * Date: June 20, 2013                                                       *
 *                                                                           *
 * Human Interaction in Virtual Enviroments (HIVE) Lab                       *
 * Worcester Polytechnic Institute, Worcester Massachusetts                  *
 * Director: Robert W. Lindeman     <gogo@wpi.edu>                           *
 * Website: http://web.cs.wpi.edu/~hive/                                     *
 *                                                                           *
 *                                                                           *
 * Contributors:                                                             *
 * Michael X. Grey <mxgrey@gatech.edu>                                       *
 *                                                                           *
 * Thanks:                                                                   *
 *                                                                           *
 *                                                                           *
 *                                                                           *
 * This file is provided under the following "BSD-style" License:            *
 *   Redistribution and use in source and binary forms, with or without      *
 *   modification, are permitted provided that the following conditions are  *   
 *   met:                                                                    *
 *   * Redistributions of source code must retain the above copyright        *
 *     notice, this list of conditions and the following disclaimer.         *
 *   * Redistributions in binary form must reproduce the above               *
 *     copyright notice, this list of conditions and the following           *
 *     disclaimer in the documentation and/or other materials provided       *
 *     with the distribution.                                                *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND                  *
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,             *
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF                *
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                *
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR                   *
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,            *
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        *
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF        *
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED         *
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT             *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN       *
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE         *
 *   POSSIBILITY OF SUCH DAMAGE.                                             *
 ****************************************************************************/

#include "hubo_sensor_control.h"

#include <stdlib.h>

//Namespace for the project/plugin
namespace DRC_Hubo_Interface {

void HuboSensorControlWidget::initializeConfigTab(void){
    //======================================================
    //=====           Build the Overall Tab            =====
    //======================================================


//    QVBoxLayout* masterCTLayout = new QVBoxLayout;
//    masterCTLayout->addWidget(myLabel);

//    myLabel->setGeometry(QRect(0, 0, 30, 50));


    configTab = new QWidget;

    myLabel = new QLabel(configTab);

//    QPixmap* mypix = new QPixmap("../resources/Meter_1.png");

    meter.push_back(new QPixmap("../resources/Meter_1.png"));
    meter.push_back(new QPixmap("../resources/Meter_2.png"));
    meter.push_back(new QPixmap("../resources/Meter_3.png"));
    meter.push_back(new QPixmap("../resources/Meter_4.png"));
    meter.push_back(new QPixmap("../resources/Meter_5.png"));
    meter.push_back(new QPixmap("../resources/Meter_6.png"));
    meter.push_back(new QPixmap("../resources/Meter_7.png"));
    meter.push_back(new QPixmap("../resources/Meter_8.png"));
    meter.push_back(new QPixmap("../resources/Meter_9.png"));
    meter.push_back(new QPixmap("../resources/Meter_10.png"));
    meter.push_back(new QPixmap("../resources/Meter_11.png"));
    meter.push_back(new QPixmap("../resources/Meter_12.png"));



    myLabel->setPixmap(*meter[0]);
    myLabel->setGeometry(QRect(0, 0, 200, 200));
    myLabel->show();

}

void HuboSensorControlWidget::DrawConfigTab(int loc) {

    myLabel->setPixmap(*meter[loc]);

}

void HuboSensorControlWidget::RefreshComms(void){

    static int count = 0;

    int num = std::rand() % 2;

    if (num == 0) count++;
    if (num == 1) count--;

    if (count < 0) count = 0;
    if (count > 11) count = 11;

    DrawConfigTab(count);

}

}
