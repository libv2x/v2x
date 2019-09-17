#
# Copyright 2019 Steve Kwon <steve@libv2x.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
libv2x_msgs/MsgHeader msg_header
# WSM-WaveShortMessage.indication
uint8         wsmp_version
uint8[<=1]    channel_number
uint8[<=1]    data_rate # 0x2-0x7F, increments of 500 kb/s, IEEE802.11-2016
int8[<=1]     transmit_power_used
uint8[<=8]    channel_load # undefined yet
uint8         user_priority # 0-7
# uint16        length # 1 to WsmMaxLength – h, WsmMaxLength: def=1400, max=2302
uint8[<=2302] data # length is included
uint8[6]      peer_mac_address # EUI48
uint64        provider_service_identifier
