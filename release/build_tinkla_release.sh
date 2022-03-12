#!/usr/bin/env bash
set -e

export GIT_COMMITTER_NAME="BogGyver"
export GIT_COMMITTER_EMAIL="bogdan.butoi@gmail.com"
export GIT_AUTHOR_NAME="BogGyver"
export GIT_AUTHOR_EMAIL="bogdan.butoi@gmail.com"
export GIT_SSH_COMMAND="ssh -i /data/gitkey"

# set CLEAN to build outside of CI

# Create folders
rm -rf /data/openpilot || true
mkdir -p /data/openpilot
cd /data/openpilot

if [ -f /TICI ]; then
  FILES_SRC="release/files_tici"
  RELEASE_BRANCH=tesla_unity_releaseC3
elif [ -f /EON ]; then
  FILES_SRC="release/files_eon"
  RELEASE_BRANCH=tesla_unity_releaseC2
else
  exit 0
fi

# Create git repo
git init
git remote add origin git@github.com:boggyver/openpilot.git
git fetch origin tesla_unity_beta


git fetch origin $RELEASE_BRANCH

# Create tesla_unity_release with no history
git checkout --orphan $RELEASE_BRANCH origin/tesla_unity_beta


VERSION=$(cat selfdrive/common/version.h | awk -F[\"-]  '{print $2}')
TINKLAVERSION=$(cat selfdrive/common/tinkla_version.h | awk -F[\"-]  '{print $2}')
echo "#define COMMA_VERSION \"$VERSION-release\"" > selfdrive/common/version.h
echo "#define TINKLA_VERSION \"$TINKLAVERSION-release\"" > selfdrive/common/tinkla_version.h

git commit -m "Tesla OpenPilot $TINKLAVERSION (openpilot v$VERSION)"

# Build signed panda firmware
cd /data/openpilot
pushd panda/board
CERT=/data/openpilot/panda/certs/debug RELEASE=0 scons -u
CERT=/data/openpilot/panda/certs/debug RELEASE=0 PEDAL=1 scons -u
CERT=/data/openpilot/panda/certs/debug RELEASE=0 PEDAL=1 PEDAL_USB=1 scons -u
mv obj/panda.bin.signed /tmp/panda.bin.signed
mv obj/pedal.bin.signed /tmp/pedal.bin.signed
mv obj/bootstub.panda.bin /tmp/bootstub.panda.bin 
mv obj/bootstub.pedal.bin /tmp/bootstub.pedal.bin
mv obj/bootstub.pedal_usb.bin /tmp/bootstub.pedal_usb.bin
mv obj/pedal_usb.bin.signed /tmp/pedal_usb.bin.signed 
popd

# Build stuff
ln -sfn /data/openpilot /data/pythonpath
export PYTHONPATH="/data/openpilot:/data/openpilot/pyextra"
SCONS_CACHE=1 scons -j3


# Cleanup
find . -name '*.a' -delete
find . -name '*.o' -delete
find . -name '*.os' -delete
find . -name '*.pyc' -delete
find . -name '__pycache__' -delete
rm -rf panda/board panda/certs panda/crypto
rm -rf .sconsign.dblite Jenkinsfile release/
rm models/supercombo.dlc

# Move back signed panda fw
cp -r /data/openpilot/release/panda_files/board /data/openpilot/panda/
mkdir -p /data/openpilot/panda/board/obj
mv /tmp/panda.bin.signed /data/openpilot/panda/board/obj/panda.bin.signed
mv /tmp/pedal.bin.signed /data/openpilot/panda/board/obj/pedal.bin.signed
mv /tmp/bootstub.panda.bin /data/openpilot/panda/board/obj/bootstub.panda.bin
mv /tmp/bootstub.pedal.bin /data/openpilot/panda/board/obj/bootstub.pedal.bin
mv /tmp/bootstub.pedal_usb.bin /data/openpilot/panda/board/obj/bootstub.pedal_usb.bin
mv /tmp/pedal_usb.bin.signed /data/openpilot/panda/board/obj/pedal_usb.bin.signed

# Restore third_party
git checkout third_party/

# Mark as prebuilt release
touch prebuilt

# Add built files to git
git add -f .
git commit --amend -m "Tesla OpenPilot $TINKLAVERSION (openpilot v$VERSION)"

# Print committed files that are normally gitignored
git status --ignored


git remote set-url origin git@github.com:boggyver/openpilot.git

# Push to tesla_unity_release
git push -f origin $RELEASE_BRANCH
