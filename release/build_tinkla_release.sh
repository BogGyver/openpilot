#!/usr/bin/env bash
set -e

SOURCE_DIR=/data/openpilot_beta
TARGET_DIR=/data/openpilot_release

export GIT_COMMITTER_NAME="BogGyver"
export GIT_COMMITTER_EMAIL="bogdan.butoi@gmail.com"
export GIT_AUTHOR_NAME="BogGyver"
export GIT_AUTHOR_EMAIL="bogdan.butoi@gmail.com"
export GIT_SSH_COMMAND="ssh -i /data/gitkey"

# set CLEAN to build outside of CI

# Create folders
rm -rf $TARGET_DIR || true
mkdir -p $TARGET_DIR
cd $TARGET_DIR

if [ -f /TICI ]; then
  FILES_SRC="$SOURCE_DIR/release/files_tici"
  RELEASE_BRANCH=tesla_unity_releaseC3
elif [ -f /EON ]; then
  FILES_SRC="$SOURCE_DIR/release/files_eon"
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
TINKLAVERSION=$(cat selfdrive/common/tinkla_version.h | awk -F[\"]  '{print $2}')

git commit -m "Tesla Unity v$TINKLAVERSION"

cd $TARGET_DIR

# Build stuff
ln -sfn $TARGET_DIR /data/pythonpath
export PYTHONPATH="$TARGET_DIR:$TARGET_DIR/pyextra"
SCONS_CACHE=1 scons -j3

# Build signed panda firmware
pushd panda/board
CERT=$TARGET_DIR/panda/certs/debug RELEASE=1 scons -u
CERT=$TARGET_DIR/panda/certs/debug RELEASE=1 PEDAL=1 scons -u
CERT=$TARGET_DIR/panda/certs/debug RELEASE=1 PEDAL=1 PEDAL_USB=1 scons -u
mv obj/panda.bin.signed /tmp/panda.bin.signed
mv obj/pedal.bin.signed /tmp/pedal.bin.signed
mv obj/bootstub.panda.bin /tmp/bootstub.panda.bin 
mv obj/bootstub.pedal.bin /tmp/bootstub.pedal.bin
mv obj/bootstub.pedal_usb.bin /tmp/bootstub.pedal_usb.bin
mv obj/pedal_usb.bin.signed /tmp/pedal_usb.bin.signed 
popd

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
cp -r $SOURCE_DIR/release/panda_files/board $TARGET_DIR/panda/
mkdir -p $TARGET_DIR/panda/board/obj
mv /tmp/panda.bin.signed $TARGET_DIR/panda/board/obj/panda.bin.signed
mv /tmp/pedal.bin.signed $TARGET_DIR/panda/board/obj/pedal.bin.signed
mv /tmp/bootstub.panda.bin $TARGET_DIR/panda/board/obj/bootstub.panda.bin
mv /tmp/bootstub.pedal.bin $TARGET_DIR/panda/board/obj/bootstub.pedal.bin
mv /tmp/bootstub.pedal_usb.bin $TARGET_DIR/panda/board/obj/bootstub.pedal_usb.bin
mv /tmp/pedal_usb.bin.signed $TARGET_DIR/panda/board/obj/pedal_usb.bin.signed

# Restore third_party
git checkout third_party/

# Add version
#update version files
echo "#define COMMA_VERSION \"$TINKLAVERSION\"" > $TARGET_DIR/selfdrive/common/version.h

# Mark as prebuilt release
touch prebuilt

# Add built files to git
git add -f .
git commit --amend -m "Tesla Unity v$TINKLAVERSION"

# Print committed files that are normally gitignored
git status --ignored


git remote set-url origin git@github.com:boggyver/openpilot.git

# Push to tesla_unity_release
git push -f origin $RELEASE_BRANCH
