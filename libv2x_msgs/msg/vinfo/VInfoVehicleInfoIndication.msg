#
# Copyright 2020 Steve Kwon <steve@libv2x.org>
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
# VInfo-VehicleInfo.indication
MsgHeader   msg_header
uint32      id
uint8       CAN_MSGTYPE_STANDARD=0   # 11 bit standard identifier
uint8       CAN_MSGTYPE_RTR=1        # remote request
uint8       CAN_MSGTYPE_EXTENDED=2   # 29 bit extended identifier
uint8       CAN_MSGTYPE_SRR=4        # self receive request
uint8       CAN_MSGTYPE_SINGLESHOT=8 # single shot (no retransmit on error)
uint8       CAN_MSGTYPE_FDF=16       # FD format frame
uint8       CAN_MSGTYPE_BRS=32       # bitrate switch
uint8       CAN_MSGTYPE_ESI=64       # error state indicator
uint16      type
uint8       reserved
uint8       dlc
uint8[<=64] data
