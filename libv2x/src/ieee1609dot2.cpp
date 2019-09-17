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

namespace libv2x
{

namespace ieee1609dot2
{

bool WSMWaveShortMessageIndication_To_SecUnsecuredDataIndication(
    const libv2x_msgs::msg::WSMWaveShortMessageIndication::SharedPtr ind,
    libv2x_msgs::msg::SecUnsecuredDataIndication & msg)
{
  Ieee1609Dot2Data_t * dot2 = nullptr;

  int ret;
  char errbuf[64];
  size_t errlen = sizeof(errbuf);

  asn_dec_rval_t rval = oer_decode(NULL, &asn_DEF_Ieee1609Dot2Data,
    (void **)&dot2, &ind->data[0], ind->data.size());
  if (rval.code != RC_OK)
  {
    ASN_STRUCT_FREE(asn_DEF_Ieee1609Dot2Data, dot2);
    return false;
  }

  ret = asn_check_constraints(&asn_DEF_Ieee1609Dot2Data,
      dot2, errbuf, &errlen);
  if (ret)
  {
    ASN_STRUCT_FREE(asn_DEF_Ieee1609Dot2Data, dot2);
    return false;
  }

  msg.protocol_version = dot2->protocolVersion;

  if (dot2->content->present == Ieee1609Dot2Content_PR_unsecuredData)
  {
    const Opaque_t *opaque = &dot2->content->choice.unsecuredData;
    msg.unsecured_data.resize(opaque->size);
    memcpy(&msg.unsecured_data[0], opaque->buf, opaque->size);
  }
  else if (dot2->content->present == Ieee1609Dot2Content_PR_signedData)
  {
    const SignedData_t * signedData = dot2->content->choice.signedData;
    const Ieee1609Dot2Data_t * ieee1609Dot2Data =
      signedData->tbsData->payload->data;
    const Opaque_t * opaque = &ieee1609Dot2Data->content->choice.unsecuredData;
    msg.unsecured_data.resize(opaque->size);
    memcpy(&msg.unsecured_data[0], opaque->buf, opaque->size);
  }
  // EncryptedData & SignedCertificateRequest
  // else { }

  ASN_STRUCT_FREE(asn_DEF_Ieee1609Dot2Data, dot2);
  return true;
}

} // namespace ieeedot2

} // namespace lIBv2x

