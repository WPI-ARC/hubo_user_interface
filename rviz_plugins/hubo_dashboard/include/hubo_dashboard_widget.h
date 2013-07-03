/*****************************************************************************
 * Copyright (c) 2013, Worcester Polytechnic Institute                       *
 * All rights reserved.                                                      *
 *                                                                           *
 * Author: Nicholas Alunni <nick.alunni@gmail.com>                           *
 * Date: July 2nd, 2013                                                      *
 *                                                                           *
 * Human Interaction in Virtual Enviroments (HIVE) Lab                       *
 * Worcester Polytechnic Institute, Worcester Massachusetts                  *
 * Director: Robert W. Lindeman     <gogo@wpi.edu>                           *
 * Website: http://web.cs.wpi.edu/~hive/                                     *
 *                                                                           *
 *                                                                           *
 * Contributors:                                                             *
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

#ifndef HUBO_DASHBOARD_WIDGET_H_
#define HUBO_DASHBOARD_WIDGET_H_

#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <QWidget>
#include <QThread>
#include <QLabel>

namespace DRC_Hubo_Interface
{

class HuboDashboardWidget;

//This class will handle refreshing for the widget. As far as I can tell it
//Is responsible for waking up at a given time and updating the interface
//Even if nothing has been touched or interacted with by the user.
class HuboDashboardRefreshManager : public QThread
{
Q_OBJECT
public:
    HuboDashboardWidget* parentWidget;
    bool alive;
    int waitTime;

protected:
    virtual void run();

protected slots:
    void getWaitTime(int t);

signals:
    void signalRefresh();

};

//This class holds the hubo_dashboard widget that will actually contain
//The different gui elements for alerting the user about the hubo's current
// state. From initial creation it will contain a comm indicator, a battery indicator,
// and a joint state indicator.
class HuboDashboardWidget: public QWidget { Q_OBJECT

public:
  // This class is not instantiated by pluginlib::ClassLoader, so the
  // constructor has no restrictions.
  HuboDashboardWidget( QWidget* parent = 0 );

  //This class will handle waking up and doing what is necessary at a given interval
  HuboDashboardRefreshManager* refreshManager;

  // Override sizeHint() to give the layout managers some idea of a
  // good size for this.
//  virtual QSize sizeHint() const { return QSize( 600, 150 ); }
//  virtual QSize maximumSize() const { return QSize( 600, 150 ); }
//  virtual QSize minimumSize() const { return QSize( 600, 150 ); }


protected:

  //A ROS node to do the ROS work
  ros::NodeHandle nh_;

  //Methods and Properties for the Joint Indicator
  void setupJointIndicator(void);

  QLabel* joint_indicator_;
  QPixmap* body_;

  //======================================================================

  //Methods and Properties for the Comm Indicator
  void setupCommIndicator(void);
  void comm_indicator_update(std_msgs::Empty input);
  void refreshCommIndicator(void);

  QLabel* comm_indicator_;
  std::vector<QPixmap*> meter_;
  int meter_id_;
  bool update_received_;

  //Subscriber for the comm indicator
  ros::Subscriber comm_indicator_update_;

  //======================================================================

  //Methods and Properties for the Battery Indicator
  void setupBatteryIndicator(void);

  QLabel* battery_indicator_;
  QPixmap* battery_;

signals:

protected slots:

    //Method to refresh the widget's appearance at a given interval
    void RefreshDashboard(void);

};

} // end namespace DRC_Hubo_Interface


#endif // HUBO_DASHBOARD_WIDGET_H_
