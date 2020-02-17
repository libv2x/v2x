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

namespace saej2735
{

bool DLUnitDataXIndication_To_MsgFrameIndication(
    const libv2x_msgs::msg::DLUnitDataXIndication::SharedPtr ind,
    libv2x_msgs::msg::MsgFrameIndication & msg)
{
  ShortMsgNpdu_t * dot3 = nullptr;
  Ieee1609Dot2Data_t * dot2 = nullptr;

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

  rval = oer_decode(NULL, &asn_DEF_Ieee1609Dot2Data,
      (void **)&dot2, dot3->body.buf, dot3->body.size);
  if (rval.code != RC_OK)
  {
    ASN_STRUCT_FREE(asn_DEF_Ieee1609Dot2Data, dot2);
    ASN_STRUCT_FREE(asn_DEF_ShortMsgNpdu, dot3);
    return false;
  }

  ret = asn_check_constraints(&asn_DEF_Ieee1609Dot2Data,
      dot2, errbuf, &errlen);
  if (ret)
  {
    ASN_STRUCT_FREE(asn_DEF_Ieee1609Dot2Data, dot2);
    ASN_STRUCT_FREE(asn_DEF_ShortMsgNpdu, dot3);
    return false;
  }

  msg.msg_version = 5;

  if (dot2->content->present == Ieee1609Dot2Content_PR_unsecuredData)
  {
    const Opaque_t * opaque = &dot2->content->choice.unsecuredData;
    if (opaque->size < 2)
    {
      ASN_STRUCT_FREE(asn_DEF_Ieee1609Dot2Data, dot2);
      ASN_STRUCT_FREE(asn_DEF_ShortMsgNpdu, dot3);
      return false;
    }

    msg.dot2_header.signer_identifier_type = 0;

    msg.msg_id = ((opaque->buf[0] & 0x7F) << 8) | opaque->buf[1];
    msg.msg_frame.resize(opaque->size);
    memcpy(&msg.msg_frame[0], opaque->buf, opaque->size);
  }
  else if (dot2->content->present == Ieee1609Dot2Content_PR_signedData)
  {
    const SignedData_t * signedData = dot2->content->choice.signedData;
    const Ieee1609Dot2Data_t * ieee1609Dot2Data =
      signedData->tbsData->payload->data;
    const Opaque_t * opaque = &ieee1609Dot2Data->content->choice.unsecuredData;
    if (opaque->size < 2)
    {
      ASN_STRUCT_FREE(asn_DEF_Ieee1609Dot2Data, dot2);
      ASN_STRUCT_FREE(asn_DEF_ShortMsgNpdu, dot3);
      return false;
    }

    if (signedData->signer.present == SignerIdentifier_PR_digest) {
      msg.dot2_header.signer_identifier_type = 1;
    }
    else if (signedData->signer.present == SignerIdentifier_PR_certificate) {
      msg.dot2_header.signer_identifier_type = 2;
    }
    else { // SignerIdentifier_PR_self
      msg.dot2_header.signer_identifier_type = 3;
    }

    msg.msg_id = ((opaque->buf[0] & 0x7F) << 8) | opaque->buf[1];
    msg.msg_frame.resize(opaque->size);
    memcpy(&msg.msg_frame[0], opaque->buf, opaque->size);
  }
  // EncryptedData & SignedCertificateRequest
  // else { }

  ASN_STRUCT_FREE(asn_DEF_Ieee1609Dot2Data, dot2);
  ASN_STRUCT_FREE(asn_DEF_ShortMsgNpdu, dot3);
  return true;
}

bool WSMWaveShortMessageIndication_To_MsgFrameIndication(
    const libv2x_msgs::msg::WSMWaveShortMessageIndication::SharedPtr ind,
    libv2x_msgs::msg::MsgFrameIndication & msg)
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

  msg.msg_version = 5;

  if (dot2->content->present == Ieee1609Dot2Content_PR_unsecuredData)
  {
    const Opaque_t * opaque = &dot2->content->choice.unsecuredData;
    if (opaque->size < 2)
    {
      ASN_STRUCT_FREE(asn_DEF_Ieee1609Dot2Data, dot2);
      return false;
    }

    msg.dot2_header.signer_identifier_type = 0;

    msg.msg_id = ((opaque->buf[0] & 0x7F) << 8) | opaque->buf[1];
    msg.msg_frame.resize(opaque->size);
    memcpy(&msg.msg_frame[0], opaque->buf, opaque->size);
  }
  else if (dot2->content->present == Ieee1609Dot2Content_PR_signedData)
  {
    const SignedData_t * signedData = dot2->content->choice.signedData;
    const Ieee1609Dot2Data_t * ieee1609Dot2Data =
      signedData->tbsData->payload->data;
    const Opaque_t * opaque = &ieee1609Dot2Data->content->choice.unsecuredData;
    if (opaque->size < 2)
    {
      ASN_STRUCT_FREE(asn_DEF_Ieee1609Dot2Data, dot2);
      return false;
    }

    if (signedData->signer.present == SignerIdentifier_PR_digest) {
      msg.dot2_header.signer_identifier_type = 1;
    }
    else if (signedData->signer.present == SignerIdentifier_PR_certificate) {
      msg.dot2_header.signer_identifier_type = 2;
    }
    else { // SignerIdentifier_PR_self
      msg.dot2_header.signer_identifier_type = 3;
    }

    msg.msg_id = ((opaque->buf[0] & 0x7F) << 8) | opaque->buf[1];
    msg.msg_frame.resize(opaque->size);
    memcpy(&msg.msg_frame[0], opaque->buf, opaque->size);
  }
  // EncryptedData & SignedCertificateRequest
  // else { }

  ASN_STRUCT_FREE(asn_DEF_Ieee1609Dot2Data, dot2);
  return true;
}

bool SecUnsecuredDataIndication_To_MsgFrameIndication(
    const libv2x_msgs::msg::SecUnsecuredDataIndication::SharedPtr ind,
    libv2x_msgs::msg::MsgFrameIndication & msg)
{
  size_t len = ind->unsecured_data.size();
  if (len < 2)
  {
    return false;
  }

  msg.msg_version = 5;
  msg.msg_id = ((ind->unsecured_data[0] & 0x7F) << 8) | ind->unsecured_data[1];
  msg.msg_frame.resize(len);
  memcpy(&msg.msg_frame[0], &ind->unsecured_data[0], len);

  return true;
}

} // namespace saej2735

} // namespace lIBv2x
