#!/bin/sh

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

CODEC_PATH="thirdparty/codec"

if [ ! -d ${CODEC_PATH} ]
then
  mkdir -p ${CODEC_PATH}
  cd ${CODEC_PATH}
  asn1c -fcompound-names ../../asn1/1609dot3all.asn ../../asn1/1609dot2all.asn ../../asn1/J2735_201603DA.asn
  make -f converter-example.mk -j4
  cd ../../
else
  echo "Directory '${CODEC_PATH}' ALREADY exists."
fi
