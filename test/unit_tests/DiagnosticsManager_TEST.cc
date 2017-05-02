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

#include <gtest/gtest.h>
#include <ignition/transport.hh>
#include <ignition/msgs.hh>

#include "gazebo/util/DiagnosticsManager.hh"

namespace gzutil = gazebo::util;

//////////////////////////////////////////////////
/// \brief test fixture for subscribig to diagnostics topic
class DiagnosticsManagerTest : public ::testing::Test
{
  /// \brief Create subscription
  protected: virtual void SetUp()
  {
    this->node.Subscribe("diagnostics", &DiagnosticsManagerTest::OnMsgRx,
        this);
  }

  /// \brief End subscription
  protected: virtual void TearDown()
  {
  }

  /// \brief called when a message is received
  protected: void OnMsgRx(const ignition::msgs::Diagnostics &_msg)
  {
    this->num++;
    this->msg = _msg;
  }

  /// \brief tools for setting up a subscriber
  protected: ignition::transport::Node node;

  /// \brief last received message
  protected: ignition::msgs::Diagnostics msg;

  /// \brief number of msgs received
  protected: int num = 0;
};

//////////////////////////////////////////////////
TEST_F(DiagnosticsManagerTest, PublishEmptyDiagnostics)
{
  gzutil::DiagnosticsManager mgr;
  ASSERT_TRUE(mgr.Init("PublishEmptyDiagnostics"));

  ignition::common::Time simTime;
  mgr.UpdateBegin(simTime);
  mgr.UpdateEnd();

  ASSERT_EQ(1, this->num);
}

//////////////////////////////////////////////////
TEST_F(DiagnosticsManagerTest, NotInitDontPublish)
{
  gzutil::DiagnosticsManager mgr;

  ignition::common::Time simTime;
  mgr.UpdateBegin(simTime);
  mgr.UpdateEnd();

  ASSERT_EQ(0, this->num);
}

//////////////////////////////////////////////////
TEST_F(DiagnosticsManagerTest, PublishSomeDiagnostics)
{
  gzutil::DiagnosticsManager mgr;
  ASSERT_TRUE(mgr.Init("PublishSomeDiagnostics"));

  ignition::common::Time simTime;
  mgr.UpdateBegin(simTime);
  mgr.StartTimer("asdf");
  mgr.StopTimer("asdf");
  mgr.UpdateEnd();

  ASSERT_EQ(1, this->num);
  ASSERT_EQ(1, this->msg.time_size());
  EXPECT_EQ("PublishSomeDiagnostics:asdf", this->msg.time(0).name());
}

//////////////////////////////////////////////////
TEST_F(DiagnosticsManagerTest, DiagnosticsClearedEveryUpdate)
{
  gzutil::DiagnosticsManager mgr;
  ASSERT_TRUE(mgr.Init("DiagnosticsClearedEveryUpdate"));

  ignition::common::Time simTime;
  mgr.UpdateBegin(simTime);
  mgr.StartTimer("asdf");
  mgr.StopTimer("asdf");
  mgr.UpdateEnd();
  ASSERT_EQ(1, this->num);

  mgr.UpdateBegin(simTime);
  mgr.StopTimer("asdf");
  mgr.UpdateEnd();
  ASSERT_EQ(2, this->num);
  EXPECT_EQ(0, this->msg.time_size());
}

int main(int argc, char **argv)
{

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
