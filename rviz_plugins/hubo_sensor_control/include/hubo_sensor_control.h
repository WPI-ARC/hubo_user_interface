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

#ifndef HUBO_SENSOR_CONTROL_H_
#define HUBO_SENSOR_CONTROL_H_

//Ros Specific Includes
#include <ros/ros.h>
#include <rviz/panel.h>

//C++ Includes
#include <vector>
#include <stdio.h>

//QT Gui Includes
#include <QApplication>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QRadioButton>
#include <QSpinBox>
#include <QLabel>
#include <QProcess>
#include <QGroupBox>
#include <QButtonGroup>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QClipboard>
#include <QPalette>
#include <QColor>
#include <QThread>
#include <QSlider>
#include <QCheckBox>
#include <QPixmap>

//Namespace for the project/plugin
namespace DRC_Hubo_Interface
{

//We will call the class something obvious ROBOT/TYPEOFWIDGET
class HuboSensorControlWidget;

//This class will handle refreshing for the widget. As far as I can tell it
//Is responsible for waking up at a given time and updating the interface
//Even if nothing has been touched or interacted with by the user.
class HuboSensorControlRefreshManager : public QThread
{
Q_OBJECT
public:
    HuboSensorControlWidget* parentWidget;
    bool alive;
    int waitTime;

protected:
    virtual void run();

protected slots:
    void getWaitTime(int t);

signals:
    void signalRefresh();

};

//=============================================================================

// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
class HuboSensorControlWidget: public QTabWidget
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  HuboSensorControlWidget( QWidget* parent = 0 );
  ~HuboSensorControlWidget();

  // Update timer
  HuboSensorControlRefreshManager* refreshManager;
  int getRefreshTime();

  //This will control how a whole bunch of the boxes look in the RVIZ panel
  QString groupStyleSheet;

protected: //Variables go here, ints and such

    //Variables for the robot state
    double jointAnglesHz;
    double forceSensorHz;
    double accelGryoHz;
    double touchSensorHz;

    //Variables for the robot vision
    double cameraFeedHz;
    bool grayscale;
    double planarHz;

    //Variables for data request
    bool narrowToggle;
    bool wideToggle;
    bool sparseToggle;
    bool denseToggle;

    

//These slots will be connected to the signals generated from the GUI in order
//To perform the various actions that are needed
protected Q_SLOTS:

    //General Q_SLOTS
    void RefreshComms(void);

    //Handles for the robot state
    void handleJointAngles(int v);
    void handleForceSensor(int v);
    void handleAccelGryo(int v);
    void handleTouchSensor(int v);

    //Handles for the robot vision
    void handleCameraFeed(int v);
    void handleGrayscale(int v);
    void handlePlanar(int v);

    //Handles for the request tab
    void handleNarrowPress(bool toggle);
    void handleWidePress(bool toggle);
    void handleSparsePress(bool toggle);
    void handleDensePress(bool toggle);
    void handleGetCloud();

private:

    //General Use
    double lookupHzOne2OneHundred(int loc);

    //===== Robot State Tab =====
    void initializeRobotStateFeedTab();
    QWidget* robotStateFeedTab;
    QSlider* jointAnglesSlider;
    QSlider* forceSensorSlider;
    QSlider* accelGryoSlider;
    QSlider* touchSensorSlider;

    //===== Robot Vision Tab =====
    void initializeRobotVisionFeedTab();
    QWidget* robotVisionFeedTab;
    QSlider* cameraFeedSlider;
    QCheckBox* cameraFeedGrayscale;
    QSlider* planarLaserSlider;
    

    //===== Sensor Request Tab =====
    void initializeSensorRequestTab();
    QWidget* sensorRequestTab;
    QButtonGroup* sparseDenseCmd;
    QButtonGroup* narrowWideCmd;
    QRadioButton* sparse;
    QRadioButton* dense;
    QRadioButton* wide;
    QRadioButton* narrow;
    QPushButton* getCloud;

    //===== Configuration Tab =====
    void initializeConfigTab();
    void DrawConfigTab(int loc);
    QWidget* configTab;
    QLabel* myLabel;

    std::vector<QPixmap*> meter;


};


class HuboSensorControlPanel : public rviz::Panel
{
Q_OBJECT
public:
    HuboSensorControlPanel(QWidget *parent = 0);

    // Now we declare overrides of rviz::Panel functions for saving and
    // loading data from the config file.  Here the data is the
    // topic name.
    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

private:

    HuboSensorControlWidget* content;

};

} // end namespace DRC_Hubo_Interface


#endif //HUBO_SENSOR_CONTROL_H_
