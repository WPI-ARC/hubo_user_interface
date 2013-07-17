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
#include <QImage>
#include <resource_retriever/retriever.h>
#include <stdlib.h>
#include <sstream>

namespace DRC_Hubo_Interface
{

void HuboDashboardWidget::setupCommIndicator(void){

    comm_indicator_ = new QLabel(this);

    meter_id_ = 0;

    update_received_ = false;

    r_ = new resource_retriever::Retriever;
    mem_r_ = new resource_retriever::MemoryResource;
    tempImage_ = new QImage();
    std::string baseFilepath = "package://hubo_dashboard/resources/Meter_";
    int last_meter = 12;

    for (int image_num = 1; image_num <= last_meter; image_num++) {

        std::ostringstream convert;

        //Create the string needed to read the file from memory
        convert << image_num;
        std::string meterNumber = convert.str() + ".png";
        std::string filepath = baseFilepath.c_str() + meterNumber;

        //Attempt to access the file and print an error if it cannot be done
        try { *mem_r_ = r_->get(filepath); }
        catch (resource_retriever::Exception& e)
            { ROS_ERROR("Failed to retrieve file: %s", e.what()); }

        //Save the loaded image into an array of Pixmaps for display
        tempImage_->loadFromData(mem_r_->data.get(), mem_r_->size);
        QPixmap* tempMap = new QPixmap();
        tempMap->convertFromImage(*tempImage_);
        meter_.push_back(tempMap);

    }

    //Set the image on the screen to be the first image and set it's size
    comm_indicator_->setPixmap(*meter_[meter_id_]);
    comm_indicator_->setGeometry(QRect(200, 0, 200, 150));
    comm_indicator_->show();

}

void HuboDashboardWidget::comm_indicator_update(std_msgs::Empty input){

    std::cout << "Comm Indicator Update Received" << std::endl;

    update_received_ = true;

}

void HuboDashboardWidget::refreshCommIndicator(void){

//    meter_id_ ++;
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
