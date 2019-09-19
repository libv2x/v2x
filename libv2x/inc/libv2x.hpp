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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _LIBV2X_HPP_
#define _LIBV2X_HPP_

#include "libv2x_msgs/msg/dl_unit_data_x_indication.hpp"
#include "libv2x_msgs/msg/wsm_wave_short_message_indication.hpp"
#include "libv2x_msgs/msg/sec_unsecured_data_indication.hpp"

#include "ShortMsgNpdu.h"
#include "Ieee1609Dot2Data.h"

namespace libv2x
{

namespace ieee1609dot3
{

uint64_t VarLengthNumber_To_UInt64(
    const VarLengthNumber_t * varLengthNumber);

bool DLUnitDataXIndication_To_WSMWaveShortMessageIndication(
    const libv2x_msgs::msg::DLUnitDataXIndication::SharedPtr ind,
    libv2x_msgs::msg::WSMWaveShortMessageIndication & msg);

} // namespace ieee1609dot3

namespace ieee1609dot2
{

bool WSMWaveShortMessageIndication_To_SecUnsecuredDataIndication(
    const libv2x_msgs::msg::WSMWaveShortMessageIndication::SharedPtr ind,
    libv2x_msgs::msg::SecUnsecuredDataIndication & msg);

} // namespace ieee1609dot2

} // namespace lIBv2x

#endif /* _LIBV2X_HPP_ */

