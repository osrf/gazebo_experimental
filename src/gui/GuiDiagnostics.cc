/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <algorithm>
#include <iostream>
#include <ignition/common/PluginMacros.hh>
#include <ignition/common/Image.hh>
#include <ignition/common/Time.hh>
#include <map>

#include "GuiDiagnostics.hh"

namespace ign_msgs = ignition::msgs;
namespace gzgui = gazebo::gui;
using namespace gzgui;


struct DiagInfo
{
  /// \brief name of timer
  QString name;
  /// \brief when timer started
  ignition::common::Time startTime;
  /// \brief when timer stopped
  ignition::common::Time endTime;
  /// \brief how long the timer took
  ignition::common::Time elapsed;
};

/////////////////////////////////////////////////
/// \brief Converts a diagnostics message into an easier to use struct
std::vector<DiagInfo> Convert(const ign_msgs::Diagnostics &_msg)
{
  std::vector<struct DiagInfo> timingInfo;
  for (int idx = 0; idx < _msg.time_size(); ++idx)
  {
    auto diagTime = _msg.time(idx);
    DiagInfo info;
    info.name = QString::fromStdString(diagTime.name());
    info.endTime.sec = diagTime.wall().sec();
    info.endTime.nsec = diagTime.wall().nsec();
    info.elapsed.sec = diagTime.elapsed().sec();
    info.elapsed.nsec = diagTime.elapsed().nsec();
    info.startTime = info.endTime - info.elapsed;
    timingInfo.push_back(info);
  }
  return timingInfo;
}


//////////////////////////////////////////////////
bool CompareStartTime(const DiagInfo &_d1, const DiagInfo &_d2)
{
  return _d1.startTime < _d2.startTime;
}

//////////////////////////////////////////////////
bool CompareEndTime(const DiagInfo &_d1, const DiagInfo &_d2)
{
  return _d1.endTime < _d2.endTime;
}

//////////////////////////////////////////////////
/// \brief Filters out messages such that the result is messages
///        from a single sim time with the most possible information
std::vector<DiagInfo> Condense(const std::vector<ign_msgs::Diagnostics> &_msgs)
{
  // Maps a simulation time to diagnostic info rx'd for it
  std::map<ignition::common::Time, std::vector<DiagInfo> > simTimeInfo;

  // Sort info into buckets according to their sim time
  for (auto const &msg : _msgs)
  {
    ignition::common::Time simTime;
    simTime.sec = msg.sim_time().sec();
    simTime.nsec = msg.sim_time().nsec();
    std::vector<DiagInfo> info = Convert(msg);

    auto iter = simTimeInfo.find(simTime);
    if (iter == simTimeInfo.end())
      simTimeInfo[simTime] = info;
    else
      iter->second.insert(iter->second.end(), info.begin(), info.end());
  }
  if (simTimeInfo.empty())
    return std::vector<DiagInfo>();

  // figure out which bucket is the biggest
  ignition::common::Time biggestTime(0,0);
  std::size_t biggestSize = 0;
  for (auto &kv : simTimeInfo)
  {
    if (kv.second.size() > biggestSize)
    {
      biggestSize = kv.second.size();
      biggestTime = kv.first;
    }
  }
  return simTimeInfo[biggestTime];
}

/////////////////////////////////////////////////
GuiDiagnostics::GuiDiagnostics()
  : Plugin()
{
  // Subscribe to get images
  std::string topic = "diagnostics";
  if (!node.Subscribe(topic, &GuiDiagnostics::OnDiagRx, this))
  {
    std::cerr << "Unable to subscribe to diagnostics" << std::endl;
  }
}

/////////////////////////////////////////////////
std::string GuiDiagnostics::Title()
{
  return "Simulation Diagnostics";
}

/////////////////////////////////////////////////
GuiDiagnostics::~GuiDiagnostics()
{
}

//////////////////////////////////////////////////
void GuiDiagnostics::paintEvent(QPaintEvent *_event)
{
  std::lock_guard<std::mutex> lock(this->mtx);
  // TODO draw an image to match the received diagnostics
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // coordinate system 0-1000.0
  const float sWidth = 1000.0f;
  const float sHeight = 1000.0f;
  const float padding = 10.0f;
  const float dividerWidth = 1.0f;
  const QColor dividerColor = qRgb(50, 50, 50);
  const QColor dataColor = qRgb(10, 100, 10);
  const QColor bgColor = qRgb(255, 255, 255);
  const QColor timeTextColor = qRgb(255,255,255);
  const float textAreaWidth = sWidth * 0.25;
  const float dataAreaWidth = sWidth - textAreaWidth - dividerWidth;
  const float max_label_height = 100.0;
  const float timeTextHeight = max_label_height / 2.0;

  // Keep the data centered and square
  float vMargin = 0.0f;
  float hMargin = 0.0f;
  float h = height();
  float w = width();

  if (w < h)
  {
    vMargin = (h * sWidth) / w - sWidth;
  }
  else
  {
    hMargin = (w * sHeight) / h - sHeight;
  }

  painter.scale(w / (sWidth + hMargin), h / (sHeight + vMargin));
  painter.translate(hMargin / 2.0f, vMargin / 2.0f);

  // Draw background color
  QRect bgRect(0.0f, 0.0f, sWidth, sHeight);
  painter.fillRect(bgRect, bgColor);
  painter.drawRect(bgRect);

  // Draw a divider between the text and the data
  QPainterPath dividerPath;
  dividerPath.moveTo(textAreaWidth, 0.0);
  dividerPath.lineTo(textAreaWidth, sHeight);
  dividerPath.closeSubpath();
  painter.setPen(QPen(dividerColor, dividerWidth, Qt::SolidLine,
        Qt::FlatCap, Qt::MiterJoin));
  painter.drawPath(dividerPath);

  std::vector<struct DiagInfo> timingInfo = Condense(this->msgs);
  this->msgs.clear();
  if (timingInfo.empty())
    return;

  const int num_labels = timingInfo.size();
  const float label_height = (sHeight / num_labels > max_label_height)
    ? max_label_height : (sHeight / num_labels);

  std::sort(timingInfo.begin(), timingInfo.end(), &CompareEndTime);
  const ignition::common::Time maxTime = timingInfo.back().endTime;
  std::sort(timingInfo.begin(), timingInfo.end(), &CompareStartTime);

  const ignition::common::Time minTime = timingInfo.front().startTime;
  const float timeWidth = (maxTime - minTime).Float();

  int dataIdx = -1;
  for (const DiagInfo &info : timingInfo)
  {
    ++dataIdx;

    // Draw text labels for all adata
    QString labelText = info.name;
    // Determine font size to fit text to text area
    QFont font = painter.font();
    font.setPointSize(label_height);
    painter.setFont(font);
    for (int i = 0; i < 3; ++i)
    {
      float w = painter.fontMetrics().boundingRect(labelText).width();
      if (w > textAreaWidth - padding)
      {
        float scale = (textAreaWidth - padding) / w;
        QFont font = painter.font();
        font.setPointSizeF(font.pointSizeF() * scale);
        painter.setFont(font);
      }
    }
    QRect rect(0, dataIdx * label_height, textAreaWidth, label_height);
    const int flags = Qt::AlignCenter;
    painter.setPen(QPen(dividerColor, dividerWidth, Qt::SolidLine,
        Qt::FlatCap, Qt::MiterJoin));
    painter.drawText(rect, flags, labelText);
    painter.drawRect(rect);

    // Draw box indicating how long the event took
    float dataStartX = ((info.startTime - minTime).Float() / timeWidth) *
      (dataAreaWidth - padding);
    float dataWidthX = info.elapsed.Float() / timeWidth *
      (dataAreaWidth - padding);
    QRect dataRect(textAreaWidth + dividerWidth + padding + dataStartX,
        dataIdx * label_height + (padding / 2.0f), dataWidthX,
        label_height - padding);
    painter.fillRect(dataRect, dataColor);

    // Draw white text over a black box with the time in nanoseconds
    const float microseconds = info.elapsed.sec * 1e6 + info.elapsed.nsec * 1e-3;
    QString timeText;
    timeText.setNum(microseconds);
    timeText.append(" micro seconds");
    // Determing font size to fit text to text area
    font = painter.font();
    font.setPointSize(timeTextHeight);
    painter.setFont(font);
    for (int i = 0; i < 3; ++i)
    {
      float h = painter.fontMetrics().boundingRect(timeText).height();
      if (h > timeTextHeight - padding)
      {
        float scale = (timeTextHeight - padding) / h;
        QFont font = painter.font();
        font.setPointSizeF(font.pointSizeF() * scale);
        painter.setFont(font);
      }
    }
    // black box
    const float timeTextWidth = painter.fontMetrics().boundingRect(timeText)
      .width();
    const float timeTextHeight = painter.fontMetrics().boundingRect(timeText)
      .height();
    const float cy = dataIdx * label_height + label_height / 2.0;
    const float cx = textAreaWidth + dividerWidth + padding +
      dataAreaWidth / 2.0;
    QRect blackBoxRect(cx - timeTextWidth / 2.0 - padding,
        cy - timeTextHeight / 2.0 - padding,
        timeTextWidth + padding * 2.0,
        timeTextHeight + padding * 2.0);
    painter.fillRect(blackBoxRect, qRgb(0, 0, 0));
    // white text
    painter.setPen(QPen(timeTextColor, dividerWidth, Qt::SolidLine,
        Qt::FlatCap, Qt::MiterJoin));
    painter.drawText(blackBoxRect, flags, timeText);
  }
}

/////////////////////////////////////////////////
Q_SLOT void GuiDiagnostics::SignalDiagRx()
{
  // Ask to be repainted
  this->update();
}

/////////////////////////////////////////////////
void GuiDiagnostics::OnDiagRx(const ign_msgs::Diagnostics &_msg)
{
  std::lock_guard<std::mutex> lock(this->mtx);
  this->msgs.push_back(_msg);
  // Signal to GUI (main) thread that diagnostics are in
  QMetaObject::invokeMethod(this, "SignalDiagRx");
}

//////////////////////////////////////////////////
QSize GuiDiagnostics::sizeHint() const
{
  return QSize(512,512);
}

//////////////////////////////////////////////////
QSize GuiDiagnostics::minimumSizeHint() const
{
  return QSize(64, 64);
}

// Register this plugin
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::gui::GuiDiagnostics,
                                  ignition::gui::Plugin);
