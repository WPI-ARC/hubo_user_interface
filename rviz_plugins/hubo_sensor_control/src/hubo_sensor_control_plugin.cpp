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

//Constructor for the Actual RVIZ Panel that we are creating
HuboSensorControlPanel::HuboSensorControlPanel(QWidget *parent) : rviz::Panel(parent) {

    //Create the widget itself
    content = new HuboSensorControlWidget;

    //Create a new boxlayout for the panel
    QHBoxLayout* panelLayout = new QHBoxLayout;

    //Add the widget to the panel
    panelLayout->addWidget(content);

    //Set it up, or do something, I don't really know yet ...
    setLayout(panelLayout);
}

//Deconstructor for the actual widget
HuboSensorControlWidget::~HuboSensorControlWidget()
{

    //Kill the refresh manager
    refreshManager->alive = false;
    refreshManager->quit();
    refreshManager->wait();
}


//Constructor for the actual Hubo Widget
HuboSensorControlWidget::HuboSensorControlWidget(QWidget *parent) : QTabWidget(parent) {

    //Setup the stylesheet that will be used throughout the code
    //This is mostly used for the boxes that contain other elements.
    //It creates a pleasant box with rounded corners in order to contain
    //The other elements of the GUI.
    groupStyleSheet = "QGroupBox {"
                      "border: 1px solid gray;"
                      "border-radius: 9px;"
                      "margin-top: 0.5em;"
                      "}"
                      "QGroupBox::title {"
                      "subcontrol-origin: margin;"
                      "left: 10px;"
                      "padding: 0 3px 0 3px;"
                      "}";

    //The First tab controls data about the robots' state
    initializeRobotStateFeedTab();
    std::cerr << "State Bandwidth Tab Loaded" << std::endl;
    addTab(robotStateFeedTab, "Hubo State");

    //The Second tab controls data about the robots' vision
    initializeRobotVisionFeedTab();
    std::cerr << "Vision Bandwidth Tab Loaded" << std::endl;
    addTab(robotVisionFeedTab, "Hubo Vision");

    //The Second tab controls data about the robots' vision
    initializeSensorRequestTab();
    std::cerr << "Sensor Request Tab Loaded" << std::endl;
    addTab(sensorRequestTab, "Data Req.");

    //The Second tab controls data about the robots' vision
    initializeConfigTab();
//    configTab = new QWidget;
    std::cerr << "Configuration Tab Loaded" << std::endl;
    addTab(configTab, "Config");

    refreshManager = new HuboSensorControlRefreshManager;
    refreshManager->parentWidget = this;
//    connect(this, SIGNAL(sendWaitTime(int)), refreshManager, SLOT(getWaitTime(int)));
    refreshManager->start();
}

double HuboSensorControlWidget::lookupHzOne2OneHundred(int loc){

    if (loc == 0) return 0;
    else if (loc == 1) return 1;
    else if (loc == 2) return 5;
    else if (loc == 3) return 10;
    else if (loc == 4) return 25;
    else if (loc == 5) return 50;
    else if (loc == 6) return 100;
    else if (loc == 7) return 1000;

}

//Run the fresh manager which runs at a given frequency.
//For right now it appears to be 1Hz.
void HuboSensorControlRefreshManager::run()
{
    alive = true;
    waitTime = 250;
    connect(this, SIGNAL(signalRefresh()), parentWidget, SLOT(RefreshComms()));
    while(alive)
    {
        emit signalRefresh();
        this->msleep(waitTime);
    }
    emit finished();
}

//Get the wait time for the refresh manager
void HuboSensorControlRefreshManager::getWaitTime(int t)
{
    waitTime = t;
}

void HuboSensorControlPanel::save(rviz::Config config) const
{

}

void HuboSensorControlPanel::load(const rviz::Config &config)
{

}


} // End hubo_init_space

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( DRC_Hubo_Interface::HuboSensorControlPanel,rviz::Panel )
PLUGINLIB_EXPORT_CLASS( DRC_Hubo_Interface::HuboSensorControlWidget, QTabWidget )
PLUGINLIB_EXPORT_CLASS( DRC_Hubo_Interface::HuboSensorControlRefreshManager, QThread )
