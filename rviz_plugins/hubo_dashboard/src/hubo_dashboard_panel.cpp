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

// BEGIN_TUTORIAL
// Here is the implementation of the TeleopPanelTwo class.  TeleopPanelTwo
// has these responsibilities:
//
// - Act as a container for GUI elements DriveWidgetTwo and QLineEdit.
// - Publish command velocities 10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
HuboDashboardPanel::HuboDashboardPanel( QWidget* parent ) : rviz::Panel( parent ) {

  this->setMinimumSize(QSize(600, 150));
  this->setMaximumSize(QSize(600, 150));

  // Then create the control widget.
  hubo_dashboard_widget_ = new HuboDashboardWidget;

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget( hubo_dashboard_widget_ );
  setLayout( layout );

}


// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void HuboDashboardPanel::save( rviz::Config config ) const
{

}

// Load all configuration data for this panel from the given Config object.
void HuboDashboardPanel::load( const rviz::Config& config )
{

}

} // end namespace DRC_Hubo_Interface

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(DRC_Hubo_Interface::HuboDashboardPanel, rviz::Panel)
PLUGINLIB_EXPORT_CLASS(DRC_Hubo_Interface::HuboDashboardWidget, QWidget )
PLUGINLIB_EXPORT_CLASS(DRC_Hubo_Interface::HuboDashboardRefreshManager, QThread )
// END_TUTORIAL
