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

void HuboSensorControlWidget::initializeRobotVisionFeedTab() {

    //======================================================
    //=====             Camera Feed Box                =====
    //======================================================  

    //Create the layout for the specific Camera Slider Item
    QVBoxLayout* cameraFeedLayout = new QVBoxLayout;

    //Create a label for the tickmarks on the slider
    QLabel* cameraFeedSliderTicks = new QLabel;
    cameraFeedSliderTicks->setSizePolicy
        (QSizePolicy::Maximum, QSizePolicy::Maximum);
    cameraFeedSliderTicks->setText
        ("XX      .1      .3       .5       1        5       10     oo (Hz)");

    //Add the Label to the cameraFeedLayout
    cameraFeedLayout->addWidget(cameraFeedSliderTicks, 0, Qt::AlignLeft);
    
    //Create the slider for the camera feed
    QSlider* cameraFeedSlider = new QSlider;
    cameraFeedSlider->setObjectName(QString::fromUtf8("cameraFeedSlider"));
    cameraFeedSlider->setOrientation(Qt::Horizontal);
    cameraFeedSlider->setGeometry(QRect(0, 0, 257, 29));
    cameraFeedSlider->setMaximum(7);
    cameraFeedSlider->setSliderPosition(3);
    cameraFeedSlider->setTickPosition(QSlider::TicksAbove);
    cameraFeedSlider->setMinimumSize(QSize(257, 29));
    cameraFeedSlider->setMaximumSize(QSize(257, 29));

    //Add the Slider to the cameraFeedLayout
    cameraFeedLayout->addWidget(cameraFeedSlider);

    //Add the checkbox for Greyscale
    QCheckBox* cameraFeedGrayscale = new QCheckBox;
    cameraFeedGrayscale->setText("Grayscale"); 

    //Add the checkbox to the cameraFeedLayout
    cameraFeedLayout->addWidget(cameraFeedGrayscale);

    //Create the box that will hold all of the camera controls
    QGroupBox* cameraFeedBox = new QGroupBox;
    cameraFeedBox->setStyleSheet(groupStyleSheet);
    cameraFeedBox->setTitle("Head Camera Feed");
    cameraFeedBox->setLayout(cameraFeedLayout);
    cameraFeedBox->setMaximumSize(QSize(325, 100));

    //======================================================
    //=====           Planar Laser Scanner             =====
    //======================================================  

    //Create the layout for the specific Camera Slider Item
    QVBoxLayout* planarLaserLayout = new QVBoxLayout;

    //Create a label for the tickmarks on the slider
    QLabel* planerLaserSliderTicks = new QLabel;
    planerLaserSliderTicks->setSizePolicy
        (QSizePolicy::Maximum, QSizePolicy::Maximum);
    planerLaserSliderTicks->setText
        ("XX      1        5      10      25     50    100    oo (Hz)");

    //Add the Label to the planarLaserLayout
    planarLaserLayout->addWidget(planerLaserSliderTicks, 0, Qt::AlignLeft);
    
    //Create the slider for the camera feed
    QSlider* planarLaserSlider = new QSlider;
    planarLaserSlider->setObjectName(QString::fromUtf8("planarLaserSlider"));
    planarLaserSlider->setOrientation(Qt::Horizontal);
    planarLaserSlider->setGeometry(QRect(0, 0, 257, 29));
    planarLaserSlider->setMaximum(7);
    planarLaserSlider->setSliderPosition(2);
    planarLaserSlider->setTickPosition(QSlider::TicksAbove);
    planarLaserSlider->setMinimumSize(QSize(257, 29));
    planarLaserSlider->setMaximumSize(QSize(257, 29));

    //Add the Slider to the planarLaserLayout
    planarLaserLayout->addWidget(planarLaserSlider);

    //Create the box that will hold all of the camera controls
    QGroupBox* planarLaserBox = new QGroupBox;
    planarLaserBox->setStyleSheet(groupStyleSheet);
    planarLaserBox->setTitle("Planar Laser Scanner");
    planarLaserBox->setLayout(planarLaserLayout);
    planarLaserBox->setMaximumSize(QSize(325, 100));

    QVBoxLayout* masterCTLayout = new QVBoxLayout;
    masterCTLayout->addWidget(cameraFeedBox, 0, Qt::AlignTop);
    masterCTLayout->addWidget(planarLaserBox, 0, Qt::AlignTop);

    //Make a new apply button
    vision_apply_button_ = new QPushButton;
    vision_apply_button_->setText("Apply");

    masterCTLayout->addWidget(vision_apply_button_, 0, Qt::AlignCenter);

    //======================================================
    //=====           Build the Overall Tab            =====
    //======================================================

    robotVisionFeedTab = new QWidget;
    robotVisionFeedTab->setLayout(masterCTLayout);
}

}
