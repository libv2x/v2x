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
# DL-UNITDATA indication
uint8[6]      source_address
uint8[6]      destination_address
uint8[<=2302] data
uint8         priority
