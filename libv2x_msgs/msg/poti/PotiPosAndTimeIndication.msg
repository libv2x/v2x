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
# Poti-PosAndTime.indication
uint8  fix
uint32 msec
uint32 nsec
int32  lat
int32  lon
int32  elev
uint8  acc_major
uint8  acc_minor
uint16 acc_orient
int16  speed
uint16 heading
int16  accel_lat
int16  accel_lon
int8   accel_vert
int16  accel_yaw
