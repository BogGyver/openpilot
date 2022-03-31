#!/usr/bin/bash -e

# git diff --name-status origin/release3-staging | grep "^A" | less

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

cd $DIR

BUILD_DIR=/data/openpilot
SOURCE_DIR="$(git rev-parse --show-toplevel)"

if [ -f /TICI ]; then
  FILES_SRC="release/files_tici"
  RELEASE_BRANCH=tesla_unity_releaseC3
elif [ -f /EON ]; then
  FILES_SRC="release/files_eon"
  RELEASE_BRANCH=tesla_unity_releaseC2
else
  exit 0
fi

# set git identity
source $DIR/identity.sh

echo "[-] Setting up repo T=$SECONDS"
rm -rf $BUILD_DIR
mkdir -p $BUILD_DIR
cd $BUILD_DIR
git init
git remote add origin git@github.com:boggyver/openpilot.git
git fetch origin $RELEASE_BRANCH
git checkout --orphan $RELEASE_BRANCH

# do the files copy
echo "[-] copying files T=$SECONDS"
cd $SOURCE_DIR
cp -pR --parents $(cat release/files_common) $BUILD_DIR/
cp -pR --parents $(cat $FILES_SRC) $BUILD_DIR/

# in the directory
cd $BUILD_DIR

rm -f panda/board/obj/panda.bin.signed

TINKLAVERSION=$(cat selfdrive/common/tinkla_version.h | awk -F[\"]  '{print $2}')
echo "#define COMMA_VERSION \"$TINKLAVERSION\"" > $TARGET_DIR/selfdrive/common/version.h


echo "[-] committing version $TINKLAVERSION T=$SECONDS"
git add -f .
git commit -a -m "Tesla Unity v$TINKLAVERSION"
git branch --set-upstream-to=origin/$RELEASE_BRANCH

# Build panda firmware
pushd panda/
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

# Build
export PYTHONPATH="$BUILD_DIR"
scons -j$(nproc)

# Ensure no submodules in release
if test "$(git submodule--helper list | wc -l)" -gt "0"; then
  echo "submodules found:"
  git submodule--helper list
  exit 1
fi
git submodule status

# Cleanup
find . -name '*.a' -delete
find . -name '*.o' -delete
find . -name '*.os' -delete
find . -name '*.pyc' -delete
find . -name 'moc_*' -delete
find . -name '__pycache__' -delete
rm -rf panda/board panda/certs panda/crypto
rm -rf .sconsign.dblite Jenkinsfile release/
rm models/supercombo.dlc

# Move back signed panda fw
mkdir -p panda/board/obj
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

# Mark as prebuilt release
touch prebuilt

# Add built files to git
git add -f .
git commit --amend -m "Tesla Unity v$TINKLAVERSION"

# Run tests
TEST_FILES="tools/"
cd $SOURCE_DIR
cp -pR -n --parents $TEST_FILES $BUILD_DIR/
cd $BUILD_DIR
RELEASE=1 selfdrive/test/test_onroad.py
#selfdrive/manager/test/test_manager.py
selfdrive/car/tests/test_car_interfaces.py
rm -rf $TEST_FILES

if [ ! -z "$PUSH" ]; then
  echo "[-] pushing T=$SECONDS"
  git push -f origin $RELEASE_BRANCH
fi

echo "[-] done T=$SECONDS"
