/*
 * Copyright 2019 Steve Kwon <steve@libv2x.org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * DLUnitDataXIndication WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rclcpp/rclcpp.hpp"

#include "libv2x.hpp"

using std::placeholders::_1;

using libv2x_msgs::msg::MsgFrameIndication;
using libv2x_msgs::msg::PotiPosAndTimeIndication;

class SaeJ2945dot1 : public rclcpp::Node
{
public:
  SaeJ2945dot1() : Node("saej2945dot1"), m_qos(rclcpp::KeepLast(10))
  {
    m_msg_sub = this->create_subscription<MsgFrameIndication>(
      "SAEJ2735/MsgFrame/Indication",
      m_qos, std::bind(&SaeJ2945dot1::j2735_ind, this, _1));
    m_poti_sub = this->create_subscription<PotiPosAndTimeIndication>(
      "POTI/PosAndTime/Indication",
      m_qos, std::bind(&SaeJ2945dot1::poti_ind, this, _1));
  }

private:
  rclcpp::Subscription<MsgFrameIndication>::SharedPtr m_msg_sub;
  rclcpp::Subscription<PotiPosAndTimeIndication>::SharedPtr m_poti_sub;
  rclcpp::SensorDataQoS m_qos;

  void j2735_ind(const MsgFrameIndication::SharedPtr ind) const
  {
    RCLCPP_DEBUG(this->get_logger(), "j2735 %ld.%09ld", ind->msg_header.ts.sec,
      ind->msg_header.ts.nanosec);
  }

  void poti_ind(const PotiPosAndTimeIndication::SharedPtr ind) const
  {
    RCLCPP_DEBUG(this->get_logger(), "poti %ld.%09ld", ind->msg_header.ts.sec,
      ind->msg_header.ts.nanosec);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SaeJ2945dot1>());
  rclcpp::shutdown();

  return 0;
}
