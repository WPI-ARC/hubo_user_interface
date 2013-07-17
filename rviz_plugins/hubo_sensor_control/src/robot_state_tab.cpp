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

//Namespace for the project/plugin
namespace DRC_Hubo_Interface {

void HuboSensorControlWidget::initializeRobotStateFeedTab()
{

    jointAnglesHz = 0;
    forceSensorHz = 0;
    accelGryoHz = 0;
    touchSensorHz = 0;

    //======================================================
    //===== Joint Angles Box                           =====
    //======================================================  

    //Create the layout for the specific Camera Slider Item
    QVBoxLayout* jointAnglesLayout = new QVBoxLayout;

    //Create a label for the tickmarks on the slider
    QLabel* jointAnglesSliderTicks = new QLabel;
    jointAnglesSliderTicks->setSizePolicy
        (QSizePolicy::Maximum, QSizePolicy::Maximum);
    jointAnglesSliderTicks->setText
        ("XX      1        5      10      25     50    100    oo (Hz)");

    //Add the Label to the jointAnglesLayout
    jointAnglesLayout->addWidget(jointAnglesSliderTicks, 0, Qt::AlignLeft);
    
    //Create the slider for the camera feed
    QSlider* jointAnglesSlider = new QSlider;
    jointAnglesSlider->setObjectName(QString::fromUtf8("jointAnglesSlider"));
    jointAnglesSlider->setOrientation(Qt::Horizontal);
    jointAnglesSlider->setGeometry(QRect(0, 0, 257, 29));
    jointAnglesSlider->setMaximum(7);
    jointAnglesSlider->setSliderPosition(0);
    jointAnglesSlider->setTickPosition(QSlider::TicksAbove);
    jointAnglesSlider->setMinimumSize(QSize(257, 29));
    jointAnglesSlider->setMaximumSize(QSize(257, 29));

    //Add the Slider to the jointAnglesLayout
    jointAnglesLayout->addWidget(jointAnglesSlider);

    //Create the box that will hold all of the camera controls
    QGroupBox* jointAnglesBox = new QGroupBox;
    jointAnglesBox->setStyleSheet(groupStyleSheet);
    jointAnglesBox->setTitle("Joint Angles");
    jointAnglesBox->setLayout(jointAnglesLayout);

    //======================================================
    //===== Forse Sensor Box                           =====
    //======================================================  

    //Create the layout for the specific Camera Slider Item
    QVBoxLayout* forseSensorLayout = new QVBoxLayout;

    //Create a label for the tickmarks on the slider
    QLabel* forseSensorSliderTicks = new QLabel;
    forseSensorSliderTicks->setSizePolicy
        (QSizePolicy::Maximum, QSizePolicy::Maximum);
    forseSensorSliderTicks->setText
        ("XX      1        5      10      25     50    100    oo (Hz)");

    //Add the Label to the forseSensorLayout
    forseSensorLayout->addWidget(forseSensorSliderTicks, 0, Qt::AlignLeft);
    
    //Create the slider for the camera feed
    QSlider* forceSensorSlider = new QSlider;
    forceSensorSlider->setObjectName(QString::fromUtf8("forceSensorSlider"));
    forceSensorSlider->setOrientation(Qt::Horizontal);
    forceSensorSlider->setGeometry(QRect(0, 0, 257, 29));
    forceSensorSlider->setMaximum(7);
    forceSensorSlider->setSliderPosition(0);
    forceSensorSlider->setTickPosition(QSlider::TicksAbove);
    forceSensorSlider->setMinimumSize(QSize(257, 29));
    forceSensorSlider->setMaximumSize(QSize(257, 29));

    //Add the Slider to the forseSensorLayout
    forseSensorLayout->addWidget(forceSensorSlider);

    //Create the box that will hold all of the camera controls
    QGroupBox* forceSensorBox = new QGroupBox;
    forceSensorBox->setStyleSheet(groupStyleSheet);
    forceSensorBox->setTitle("Force Sensors");
    forceSensorBox->setLayout(forseSensorLayout);

    //======================================================
    //===== Acceleromter / Gyro Box                    =====
    //======================================================  
    //Create the layout for the specific Camera Slider Item
    QVBoxLayout* accelGryoLayout = new QVBoxLayout;

    //Create a label for the tickmarks on the slider
    QLabel* accelGryoSliderTicks = new QLabel;
    accelGryoSliderTicks->setSizePolicy
        (QSizePolicy::Maximum, QSizePolicy::Maximum);
    accelGryoSliderTicks->setText
        ("XX      1        5      10      25     50    100    oo (Hz)");

    //Add the Label to the accelGryoLayout
    accelGryoLayout->addWidget(accelGryoSliderTicks, 0, Qt::AlignLeft);
    
    //Create the slider for the camera feed
    QSlider* accelGryoSlider = new QSlider;
    accelGryoSlider->setObjectName(QString::fromUtf8("accelGryoSlider"));
    accelGryoSlider->setOrientation(Qt::Horizontal);
    accelGryoSlider->setGeometry(QRect(0, 0, 257, 29));
    accelGryoSlider->setMaximum(7);
    accelGryoSlider->setSliderPosition(0);
    accelGryoSlider->setTickPosition(QSlider::TicksAbove);
    accelGryoSlider->setMinimumSize(QSize(257, 29));
    accelGryoSlider->setMaximumSize(QSize(257, 29));

    //Add the Slider to the accelGryoLayout
    accelGryoLayout->addWidget(accelGryoSlider);

    //Create the box that will hold all of the camera controls
    QGroupBox* accelGryoBox = new QGroupBox;
    accelGryoBox->setStyleSheet(groupStyleSheet);
    accelGryoBox->setTitle("Accelerometers and Gryos");
    accelGryoBox->setLayout(accelGryoLayout);

    //======================================================
    //===== Touch Sensor Box                           =====
    //======================================================  

    //Create the layout for the specific Camera Slider Item
    QVBoxLayout* touchSensorLayout = new QVBoxLayout;

    //Create a label for the tickmarks on the slider
    QLabel* touchSensorSliderTicks = new QLabel;
    touchSensorSliderTicks->setSizePolicy
        (QSizePolicy::Maximum, QSizePolicy::Maximum);
    touchSensorSliderTicks->setText
        ("XX      1        5      10      25     50    100    oo (Hz)");

    //Add the Label to the touchSensorLayout
    touchSensorLayout->addWidget(touchSensorSliderTicks, 0, Qt::AlignLeft);
    
    //Create the slider for the camera feed
    QSlider* touchSensorSlider = new QSlider;
    touchSensorSlider->setObjectName(QString::fromUtf8("touchSensorSlider"));
    touchSensorSlider->setOrientation(Qt::Horizontal);
    touchSensorSlider->setGeometry(QRect(0, 0, 257, 29));
    touchSensorSlider->setMaximum(7);
    touchSensorSlider->setSliderPosition(0);
    touchSensorSlider->setTickPosition(QSlider::TicksAbove);
    touchSensorSlider->setMinimumSize(QSize(257, 29));
    touchSensorSlider->setMaximumSize(QSize(257, 29));

    //Add the Slider to the touchSensorLayout
    touchSensorLayout->addWidget(touchSensorSlider);

    //Create the box that will hold all of the camera controls
    QGroupBox* touchSensorBox = new QGroupBox;
    touchSensorBox->setStyleSheet(groupStyleSheet);
    touchSensorBox->setTitle("Touch Sensors");
    touchSensorBox->setLayout(touchSensorLayout);

    //======================================================
    //===== Build the Overall Tab                      =====
    //======================================================

    connect(jointAnglesSlider, SIGNAL(valueChanged(int)), 
            this,              SLOT(handleJointAngles(int)));

    connect(forceSensorSlider, SIGNAL(valueChanged(int)), 
            this,              SLOT(handleForceSensor(int)));

    connect(accelGryoSlider, SIGNAL(valueChanged(int)), 
            this,            SLOT(handleAccelGryo(int)));

    connect(touchSensorSlider, SIGNAL(valueChanged(int)), 
            this,              SLOT(handleTouchSensor(int)));

    QVBoxLayout* masterCTLayout = new QVBoxLayout;
    masterCTLayout->addWidget(jointAnglesBox);
    masterCTLayout->addWidget(forceSensorBox);
    masterCTLayout->addWidget(accelGryoBox);
    masterCTLayout->addWidget(touchSensorBox);

    //Make a new apply button
    state_apply_button_ = new QPushButton;
    state_apply_button_->setText("Apply");
    masterCTLayout->addWidget(state_apply_button_, 0, Qt::AlignCenter);

    connect(state_apply_button_, SIGNAL(released(void)),
            this,              SLOT(handleStateApply(void)));

    robotStateFeedTab = new QWidget;
    robotStateFeedTab->setLayout(masterCTLayout);
}

}
