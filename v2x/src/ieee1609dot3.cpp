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

#include "libv2x_msgs/msg/dl_unit_data_x_indication.hpp"
#include "libv2x_msgs/msg/wsm_wave_short_message_indication.hpp"

#include "libv2x.hpp"

using std::placeholders::_1;

using libv2x_msgs::msg::DLUnitDataXIndication;
using libv2x_msgs::msg::WSMWaveShortMessageIndication;

using libv2x::ieee1609dot3
  ::DLUnitDataXIndication_To_WSMWaveShortMessageIndication;

class Ieee1609Dot3 : public rclcpp::Node
{
public:
  Ieee1609Dot3() : Node("ieee1609dot3"), m_qos(rclcpp::KeepLast(10))
  {
    m_dl_sub = this->create_subscription<DLUnitDataXIndication>(
      "IEEE1609Dot3/DLUnitDataX/Indication",
      m_qos, std::bind(&Ieee1609Dot3::dot3_ind, this, _1));

    m_wsm_pub = this->create_publisher<WSMWaveShortMessageIndication>(
      "IEEE1609Dot3/WSMWaveShortMessage/Indication", m_qos);
  }

private:
  rclcpp::Subscription<DLUnitDataXIndication>::SharedPtr m_dl_sub;
  rclcpp::Publisher<WSMWaveShortMessageIndication>::SharedPtr m_wsm_pub;
  rclcpp::SensorDataQoS m_qos;

  void dot3_ind(const DLUnitDataXIndication::SharedPtr ind) const
  {
    RCLCPP_DEBUG(this->get_logger(), "%ld.%09ld", ind->msg_header.ts.sec,
      ind->msg_header.ts.nanosec);

    auto msg = WSMWaveShortMessageIndication();

    if (DLUnitDataXIndication_To_WSMWaveShortMessageIndication(ind, msg))
    {
      RCLCPP_DEBUG(this->get_logger(), "%u %" PRIu64 " %zu",
        msg.wsmp_version, msg.provider_service_identifier, msg.data.size());

      msg.msg_header = ind->msg_header;
      m_wsm_pub->publish(msg);
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ieee1609Dot3>());
  rclcpp::shutdown();

  return 0;
}
