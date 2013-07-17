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

void HuboSensorControlWidget::initializeSensorRequestTab(void){

    narrowToggle = true;
    wideToggle = false;
    sparseToggle = true;
    denseToggle = false;

    //======================================================
    //===== Pointcloud Request Box                     =====
    //======================================================  

    //Create the layout for the specific Camera Slider Item
    QVBoxLayout* pointcloudLayout = new QVBoxLayout;

    //Create two horizontal layouts to contain the first two buttons
    QHBoxLayout* topButtonsLayout = new QHBoxLayout;
    QHBoxLayout* bottomButtonsLayout = new QHBoxLayout;
    QHBoxLayout* pointcloudButton = new QHBoxLayout;

    //Create two button groups so they can be exclusive
    sparseDenseCmd = new QButtonGroup(this);
    narrowWideCmd = new QButtonGroup(this);

    //Make them exclusive
    sparseDenseCmd->setExclusive(true);
    narrowWideCmd->setExclusive(true);

    //Create the buttons that are needed
    QRadioButton* sparse = new QRadioButton;
    QRadioButton* dense = new QRadioButton;
    QRadioButton* wide = new QRadioButton;
    QRadioButton* narrow = new QRadioButton;

    sparse->setText("Sparse");
    dense->setText("Dense");
    wide->setText("Wide");
    narrow->setText("Narrow");

    sparseDenseCmd->addButton(sparse);
    sparseDenseCmd->addButton(dense);
    
    narrowWideCmd->addButton(narrow);
    narrowWideCmd->addButton(wide);

    //Add the Buttons to their respective layouts

    topButtonsLayout->addWidget(sparse);
    topButtonsLayout->addWidget(narrow);

    bottomButtonsLayout->addWidget(dense);
    bottomButtonsLayout->addWidget(wide);

    pointcloudLayout->addLayout(topButtonsLayout);
    pointcloudLayout->addLayout(bottomButtonsLayout);

    //Make the button
    QPushButton* getCloud = new QPushButton;
    getCloud->setMaximumSize(QSize(141, 27));
    getCloud->setText("Get Point Cloud");

    pointcloudButton->addWidget(getCloud, 0, Qt::AlignCenter);
    
    pointcloudLayout->addLayout(pointcloudButton);

    //Signals and Slots Here

    connect(sparse, SIGNAL(toggled(bool)), 
            this,   SLOT(handleSparsePress(bool)));

    connect(dense, SIGNAL(toggled(bool)), 
            this,  SLOT(handleDensePress(bool)));

    connect(narrow, SIGNAL(toggled(bool)), 
            this,   SLOT(handleNarrowPress(bool)));

    connect(wide, SIGNAL(toggled(bool)), 
            this, SLOT(handleWidePress(bool)));

    connect(getCloud, SIGNAL(released()), 
            this,     SLOT(handleGetCloud()));

    //Make the box to hold the layout
    QGroupBox* pointcloudBox = new QGroupBox;
    pointcloudBox->setStyleSheet(groupStyleSheet);
    pointcloudBox->setTitle("Request Point Cloud");
    pointcloudBox->setLayout(pointcloudLayout);
    pointcloudBox->setMaximumSize(QSize(300, 100));

    QVBoxLayout* masterCTLayout = new QVBoxLayout;
    masterCTLayout->addWidget(pointcloudBox, 0, Qt::AlignTop);

    sensorRequestTab = new QWidget;
    sensorRequestTab->setLayout(masterCTLayout);

}

}
