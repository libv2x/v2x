#!/bin/sh

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
