#!/usr/bin/bash
cd /data/openpilot/system/qcomgpsd/
curl -O http://xtrapath3.izatcloud.net/xtra3grc.bin
QCOM_ALT_ASSISTANCE_PATH=/data/openpilot/system/qcomgpsd/xtra3grc.bin
export QCOM_ALT_ASSISTANCE_PATH