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

#include "libv2x.hpp"

using libv2x_msgs::msg::DLUnitDataXIndication;
using libv2x_msgs::msg::WSMWaveShortMessageIndication;

namespace libv2x
{

namespace ieee1609dot3
{

uint64_t VarLengthNumber_To_UInt64(const VarLengthNumber_t * varLengthNumber)
{
  uint64_t psid;

  if (varLengthNumber->present == VarLengthNumber_PR_extension)
  {
    const Ext1_t * ext1 = &varLengthNumber->choice.extension;

    if (ext1->present == Ext1_PR_extension)
    {
      const Ext2_t * ext2 = &ext1->choice.extension;

      if (ext2->present == Ext2_PR_extension)
      {
        psid = ext2->choice.extension;
      }
      else
      {
        psid = ext2->choice.content;
      }
    }
    else
    {
      psid = ext1->choice.content;
    }
  }
  else
  {
    psid = varLengthNumber->choice.content;
  }

  return psid;
}

bool DLUnitDataIndication_To_WSMWaveShortMessageIndication(
    const DLUnitDataXIndication::SharedPtr ind,
    WSMWaveShortMessageIndication & msg)
{
  ShortMsgNpdu_t * dot3 = nullptr;

  int ret;
  char errbuf[64];
  size_t errlen = sizeof(errbuf);

  asn_dec_rval_t rval = uper_decode_complete(NULL, &asn_DEF_ShortMsgNpdu,
      (void **)&dot3, &ind->data[0], ind->data.size());
  if (rval.code != RC_OK)
  {
    ASN_STRUCT_FREE(asn_DEF_ShortMsgNpdu, dot3);
    return false;
  }

  ret = asn_check_constraints(&asn_DEF_ShortMsgNpdu,
      dot3, errbuf, &errlen);
  if (ret)
  {
    ASN_STRUCT_FREE(asn_DEF_ShortMsgNpdu, dot3);
    return false;
  }

  const NullNetworking_t * nullNetworking =
    &dot3->subtype.choice.nullNetworking;
  msg.wsmp_version = nullNetworking->version;

  if (nullNetworking->nExtensions)
  {
    const ShortMsgNextensions_t * shortMsgNextensions =
      nullNetworking->nExtensions;

    for (int i = 0; i < shortMsgNextensions->list.count; i++)
    {
      const ShortMsgNextension_t * shortMsgNextension =
        (ShortMsgNextension_t * )shortMsgNextensions->list.array[i];

      if (shortMsgNextension->value.present ==
          ShortMsgNextension__value_PR_TXpower80211)
      {
        msg.transmit_power_used.push_back(
            shortMsgNextension->value.choice.TXpower80211);
      }
      else if (shortMsgNextension->value.present ==
          ShortMsgNextension__value_PR_ChannelNumber80211)
      {
        msg.data_rate.push_back(
            shortMsgNextension->value.choice.DataRate80211);
      }
      else if (shortMsgNextension->value.present ==
          ShortMsgNextension__value_PR_ChannelNumber80211)
      {
        msg.channel_number.push_back(
            shortMsgNextension->value.choice.ChannelNumber80211);
      }
      // TODO: not specified in std.
      // msg.channel_load;
    }
  }

  msg.user_priority = ind->priority;
  msg.data.resize(dot3->body.size);
  memcpy(&msg.data[0], dot3->body.buf, dot3->body.size);
  msg.peer_mac_address = ind->destination_address;

  const VarLengthNumber_t * varLengthNumber =
    &dot3->transport.choice.bcMode.destAddress;

  msg.provider_service_identifier =
    VarLengthNumber_To_UInt64(varLengthNumber);

  ASN_STRUCT_FREE(asn_DEF_ShortMsgNpdu, dot3);
  return true;
}

} // namespace ieeedot3

} // namespace lIBv2x

