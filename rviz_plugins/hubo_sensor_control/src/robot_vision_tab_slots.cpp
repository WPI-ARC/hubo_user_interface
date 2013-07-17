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

//======================================
// Robot State Slots
//======================================
void HuboSensorControlWidget::handleCameraFeed(int v){
   cameraFeedHz = lookupHzPointOne2Ten(v);
   std::cerr << "(Debug) Camera Rate Switched to " << cameraFeedHz << " (Hz)" << std::endl;
}

void HuboSensorControlWidget::handlePlanar(int v){
   planarHz = lookupHzPointOne2Ten(v);
   std::cerr << "(Debug) Planar Rate Switched to " << planarHz << " (Hz)" << std::endl;
}

void HuboSensorControlWidget::handleVisionApply(void){

    std::cout << "\n(Debug) Apply Pressed on Robot Vision Tab!" << std::endl;

    teleop_msgs::RateControl camera_srv;
    camera_srv.request.Rate = cameraFeedHz;
    std::cout << "(Debug) Calling " << HUBO_CAMERA_SERVICE << " called with " << cameraFeedHz << " (Hz) ... ";
    if (vision_camera_client_.call(camera_srv)) std::cout << " Response = " << camera_srv.response.State << std::endl;
    else std::cout << "FAILED" << std::endl;

    teleop_msgs::RateControl planar_srv;
    planar_srv.request.Rate = planarHz;
    std::cout << "(Debug) Calling " << HUBO_PLANAR_SERVICE << " called with " << planarHz << " (Hz) ... ";
    if(vision_planar_client_.call(planar_srv)) std::cout << " Response = " << planar_srv.response.State << std::endl;
    else std::cout << "FAILED" << std::endl;

    std::cout << "(Debug) All services have returned, continuing! \n" << std::endl;

}

}
