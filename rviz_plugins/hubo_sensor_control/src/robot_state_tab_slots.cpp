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
void HuboSensorControlWidget::handleJointAngles(int v){
   jointAnglesHz = lookupHzOne2OneHundred(v);
   std::cerr << "(Debug) Joint Angles Rate Switched to " << jointAnglesHz << " (Hz)" << std::endl;
}

void HuboSensorControlWidget::handleForceSensor(int v){
   forceSensorHz = lookupHzOne2OneHundred(v);
   std::cerr << "(Debug) Force Sensors Rate Switched to " << forceSensorHz << " (Hz)" << std::endl;
}

void HuboSensorControlWidget::handleAccelGryo(int v){
   accelGryoHz = lookupHzOne2OneHundred(v);
   std::cerr << "(Debug) Accel and Gryo Rate Switched to " << accelGryoHz << " (Hz)" << std::endl;
}

void HuboSensorControlWidget::handleTouchSensor(int v){
   touchSensorHz = lookupHzOne2OneHundred(v);
   std::cerr << "(Debug) Touch Sensors Rate Switched to " << touchSensorHz << " (Hz)" << std::endl;
}

void HuboSensorControlWidget::handleStateApply(void){

    std::cout << "\n(Debug) Apply Pressed on Robot State Tab!" << std::endl;

    teleop_msgs::RateControl joint_srv;
    joint_srv.request.Rate = jointAnglesHz;
    std::cout << "(Debug) Calling " << HUBO_JOINT_STATES_SERVICE << " called with " << jointAnglesHz << " (Hz) ... ";
    if (state_joints_client_.call(joint_srv)) std::cout << " Response = " << joint_srv.response.State << std::endl;
    else std::cout << "FAILED" << std::endl;

    teleop_msgs::RateControl forces_srv;
    forces_srv.request.Rate = forceSensorHz;
    std::cout << "(Debug) Calling " << HUBO_FORCE_SENSOR_SERVICE << " called with " << forceSensorHz << " (Hz) ... ";
    if (state_forces_client_.call(forces_srv)) std::cout << " Response = " << forces_srv.response.State  << std::endl;
    else std::cout << "FAILED" << std::endl;

    teleop_msgs::RateControl accel_srv;
    accel_srv.request.Rate = accelGryoHz;
    std::cout << "(Debug) Calling " << HUBO_ACCEL_GRYO_SERVICE << " called with " << accelGryoHz << " (Hz) ... ";
    if (state_accel_client_.call(accel_srv)) std::cout << " Response = " << accel_srv.response.State  << std::endl;
    else std::cout << "FAILED" << std::endl;

    teleop_msgs::RateControl touch_srv;
    touch_srv.request.Rate = touchSensorHz;
    std::cout << "(Debug) Calling " << HUBO_TOUCH_SENSORS_SERVICE << " called with " << touchSensorHz << " (Hz) ... ";
    if (state_touch_client_.call(touch_srv)) std::cout << " Response = " << touch_srv.response.State  << std::endl;
    else std::cout << "FAILED" << std::endl;

    std::cout << "(Debug) All services have returned, continuing! \n" << std::endl;

}

}
