/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "hubo_dashboard_widget.h"
#include "hubo_dashboard_panel.h"

namespace DRC_Hubo_Interface
{

HuboDashboardWidget::HuboDashboardWidget( QWidget* parent ) : QWidget( parent ) {

    std::string topic = "/hubo_dashboard/comm_indicator/update";

    comm_indicator_update_ = nh_.subscribe(topic.c_str(), 1, &HuboDashboardWidget::comm_indicator_update, this);

    //Setup the Joint Indicator
//    setupJointIndicator();

    //Setup the Comm Indicator
    setupCommIndicator();

    //Setup the Battery Indicator
//    setupBatteryIndicator();

    //Setup the refresh manager to run
    refreshManager = new HuboDashboardRefreshManager;
    refreshManager->parentWidget = this;
    refreshManager->start();

}

void HuboDashboardWidget::RefreshDashboard(void){

    refreshCommIndicator();

}

//============================================================================
//============================================================================
// HuboDashboardRefreshManager Class
//============================================================================
//============================================================================


//Run the fresh manager which runs at a given frequency.
//For right now it appears to be 1Hz.
void HuboDashboardRefreshManager::run()
{
    alive = true;
    waitTime = 250;
    connect(this, SIGNAL(signalRefresh()), parentWidget, SLOT(RefreshDashboard()));
    while(alive)
    {
        emit signalRefresh();
        this->msleep(waitTime);
    }
    emit finished();
}

//Get the wait time for the refresh manager
void HuboDashboardRefreshManager::getWaitTime(int t)
{
    waitTime = t;
}

} // end namespace DRC_Hubo_Interface
