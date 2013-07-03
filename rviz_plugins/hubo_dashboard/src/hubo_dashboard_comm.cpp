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

#include "hubo_dashboard_widget.h"

#include <iostream>

namespace DRC_Hubo_Interface
{

void HuboDashboardWidget::setupCommIndicator(void){

    std::cout << "Comm Indicator Setup Started" << std::endl;

    comm_indicator_ = new QLabel(this);

    meter_id_ = 0;

    update_received_ = false;

//    meter_.push_back(new QPixmap("../resources/Meter_1.png"));
//    meter_.push_back(new QPixmap("../resources/Meter_2.png"));
//    meter_.push_back(new QPixmap("../resources/Meter_3.png"));
//    meter_.push_back(new QPixmap("../resources/Meter_4.png"));
//    meter_.push_back(new QPixmap("../resources/Meter_5.png"));
//    meter_.push_back(new QPixmap("../resources/Meter_6.png"));
//    meter_.push_back(new QPixmap("../resources/Meter_7.png"));
//    meter_.push_back(new QPixmap("../resources/Meter_8.png"));
//    meter_.push_back(new QPixmap("../resources/Meter_9.png"));
//    meter_.push_back(new QPixmap("../resources/Meter_10.png"));
//    meter_.push_back(new QPixmap("../resources/Meter_11.png"));
//    meter_.push_back(new QPixmap("../resources/Meter_12.png"));

      meter_.push_back(new QPixmap("resources/Meter_1.png"));
      meter_.push_back(new QPixmap("resources/Meter_2.png"));
      meter_.push_back(new QPixmap("resources/Meter_3.png"));
      meter_.push_back(new QPixmap("resources/Meter_4.png"));
      meter_.push_back(new QPixmap("resources/Meter_5.png"));
      meter_.push_back(new QPixmap("resources/Meter_6.png"));
      meter_.push_back(new QPixmap("resources/Meter_7.png"));
      meter_.push_back(new QPixmap("resources/Meter_8.png"));
      meter_.push_back(new QPixmap("resources/Meter_9.png"));
      meter_.push_back(new QPixmap("resources/Meter_10.png"));
      meter_.push_back(new QPixmap("resources/Meter_11.png"));
      meter_.push_back(new QPixmap("resources/Meter_12.png"));

//    meter_.push_back(new QPixmap("rviz_plugins/hubo_dashboard/resources/Meter_1.png"));
//    meter_.push_back(new QPixmap("rviz_plugins/hubo_dashboard/resources/Meter_2.png"));
//    meter_.push_back(new QPixmap("rviz_plugins/hubo_dashboard/resources/Meter_3.png"));
//    meter_.push_back(new QPixmap("rviz_plugins/hubo_dashboard/resources/Meter_4.png"));
//    meter_.push_back(new QPixmap("rviz_plugins/hubo_dashboard/resources/Meter_5.png"));
//    meter_.push_back(new QPixmap("rviz_plugins/hubo_dashboard/resources/Meter_6.png"));
//    meter_.push_back(new QPixmap("rviz_plugins/hubo_dashboard/resources/Meter_7.png"));
//    meter_.push_back(new QPixmap("rviz_plugins/hubo_dashboard/resources/Meter_8.png"));
//    meter_.push_back(new QPixmap("rviz_plugins/hubo_dashboard/resources/Meter_9.png"));
//    meter_.push_back(new QPixmap("rviz_plugins/hubo_dashboard/resources/Meter_10.png"));
//    meter_.push_back(new QPixmap("rviz_plugins/hubo_dashboard/resources/Meter_11.png"));
//    meter_.push_back(new QPixmap("rviz_plugins/hubo_dashboard/resources/Meter_12.png"));

    comm_indicator_->setPixmap(*meter_[meter_id_]);

    comm_indicator_->setGeometry(QRect(200, 0, 200, 150));
    comm_indicator_->show();

    std::cout << "Comm Indicator Setup Finished" << std::endl;

}

void HuboDashboardWidget::comm_indicator_update(std_msgs::Empty input){

    std::cout << "Comm Indicator Update Received" << std::endl;

    update_received_ = true;

}

void HuboDashboardWidget::refreshCommIndicator(void){

//    meter_id_++;
//    if (meter_id_ > 11) meter_id_ = 0;

    if (update_received_ == true) {
        meter_id_ = 0;
        update_received_ = false;
    } else {meter_id_++;}

    if (meter_id_ < 0) meter_id_ = 0;
    if (meter_id_ > 11) meter_id_ = 11;


    comm_indicator_->setPixmap(*meter_[meter_id_]);

}

}
