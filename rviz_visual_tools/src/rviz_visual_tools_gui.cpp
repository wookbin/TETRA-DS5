/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, PickNik Consulting
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Consulting nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Rviz display panel for controlling and debugging ROS applications
*/

#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>

#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QString>
#include <QDebug>
#include <QFile>
#include <QDir>
#include <QTextStream>
#include <QFileDialog>
#include <QDirIterator>

#include "rviz_visual_tools_gui.h"

QString qstring_text;
QString qstring_text2;
std::string strLocation_name;
std::string strGoto_location_name;
std::string strGUI_btn;

QDir dir("C:/home/tetra/DATA/"); //file path

namespace rviz_visual_tools
{
RvizVisualToolsGui::RvizVisualToolsGui(QWidget* parent) : rviz::Panel(parent)
{
  //////////////////////////////////////////////////////////////////////////
  //Create LineEdit
  line_edit_ = new QLineEdit(this);
  line_edit_->setText("Save location name...");

  // Create a push button
  btn_location_save_ = new QPushButton(this);
  btn_location_save_->setText("SAVE");
  connect(btn_location_save_, SIGNAL(clicked()), this, SLOT(SetLocation()));

  // Create a push button
  btn_refresh_ = new QPushButton(this);
  btn_refresh_->setText("Refresh");
  connect(btn_refresh_, SIGNAL(clicked()), this, SLOT(SetRefresh()));

  //list box
  list_box_ = new QListWidget(this);

  // Create a push button
  btn_HOME_ = new QPushButton(this);
  btn_HOME_->setText("HOME");
  connect(btn_HOME_, SIGNAL(clicked()), this, SLOT(moveHOME()));

  // Create a push button
  btn_GOTO_ = new QPushButton(this);
  btn_GOTO_->setText("GOTO");
  connect(btn_GOTO_, SIGNAL(clicked()), this, SLOT(moveGoto()));

  // Create a push button
  btn_STOP_ = new QPushButton(this);
  btn_STOP_->setText("STOP");
  connect(btn_STOP_, SIGNAL(clicked()), this, SLOT(moveStop()));

  // Horizontal Layout
  auto* hlayout0 = new QHBoxLayout;
  auto* hlayout1 = new QHBoxLayout;
  auto* hlayout2 = new QHBoxLayout;

  hlayout0->addWidget(line_edit_);
  hlayout0->addWidget(btn_location_save_);

  hlayout1->addWidget(list_box_);
  hlayout1->addWidget(btn_refresh_);

  hlayout2->addWidget(btn_HOME_);
  hlayout2->addWidget(btn_GOTO_);
  hlayout2->addWidget(btn_STOP_);

  // Verticle layout
  auto* layout = new QVBoxLayout;
  layout->addLayout(hlayout0);
  layout->addLayout(hlayout1);
  layout->addLayout(hlayout2);

  setLayout(layout);

}

void RvizVisualToolsGui::moveHOME()
{
  remote_reciever_.publishHOME();
}

void RvizVisualToolsGui::moveGoto()
{
  //qDebug()<< list_box_->currentItem()->text();
  qstring_text2 = list_box_->currentItem()->text();
  strGoto_location_name = qstring_text2.toStdString().c_str();
  remote_reciever_.publishGoto(strGoto_location_name);
}

void RvizVisualToolsGui::moveStop()
{
  remote_reciever_.publishStop();
}

//add...
void RvizVisualToolsGui::SetLocation()
{
  qstring_text = line_edit_->text();
  //qDebug()<<qstring_text;
  strLocation_name = qstring_text.toStdString().c_str();
  remote_reciever_.publishSetLocation(strLocation_name);
}

void RvizVisualToolsGui::SetRefresh()
{
  QStringList strFilters;
  strFilters += "*.txt";
  QDirIterator iterDir("/home/tetra/DATA/", strFilters, QDir::Files | QDir::NoSymLinks, QDirIterator::Subdirectories);
  
  list_box_->clear();
  QString filename;
  QString filename2;

  while (iterDir.hasNext())
  {
    iterDir.next();
    filename = iterDir.fileName();
    filename2 = filename.split(".",QString::SkipEmptyParts).at(0);
    list_box_->addItem(filename2);
  }
}

//File Open
void RvizVisualToolsGui::Fileread(QString filename)
{
    QFile file(filename);
    if(!file.open(QFile::ReadOnly | QFile::Text))
    {
        qDebug() << " Could not open the file for reading";
        return;
    }

    QTextStream in(&file);
    QString myText = in.readAll();
    qDebug() << myText;

    file.close();
}


void RvizVisualToolsGui::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void RvizVisualToolsGui::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}
}  // end namespace rviz_visual_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_visual_tools::RvizVisualToolsGui, rviz::Panel)
