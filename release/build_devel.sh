#!/usr/bin/bash -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

TARGET_DIR=/data/openpilot
SOURCE_DIR="$(git rev-parse --show-toplevel)"
TINKLA_BETA_NUMBER="$1"

cd $DIR
git pull
git submodule init
git submodule update

# set git identity
source $DIR/identity.sh
rm -rf $TARGET_DIR
echo "[-] Setting up repo T=$SECONDS"
if [ ! -d "$TARGET_DIR" ]; then
  mkdir -p $TARGET_DIR
  cd $TARGET_DIR
  git init
  git remote add origin git@github.com:boggyver/openpilot.git
fi

echo "[-] bringing tesla_unity_dev and tesla_unity_beta in sync T=$SECONDS"
cd $TARGET_DIR
git prune || true
git remote prune origin || true
git fetch origin tesla_unity_dev
git fetch origin tesla_unity_beta

git checkout -f --track origin/tesla_unity_dev
git reset --hard tesla_unity_dev
git checkout tesla_unity_dev
git reset --hard origin/tesla_unity_beta
git clean -xdf

# remove everything except .git
echo "[-] erasing old openpilot T=$SECONDS"
find . -maxdepth 1 -not -path './.git' -not -name '.' -not -name '..' -exec rm -rf '{}' \;

# reset source tree
cd $SOURCE_DIR
git clean -xdf

# do the files copy
echo "[-] copying files T=$SECONDS"
cd $SOURCE_DIR
cp -pR --parents $(cat release/files_common) $TARGET_DIR/
cp -pR --parents $(cat release/files_tici) $TARGET_DIR/
if [ ! -z "$EXTRA_FILES" ]; then
  cp -pR --parents $EXTRA_FILES $TARGET_DIR/
fi

# append source commit hash and build date to version
GIT_HASH=$(git --git-dir=$SOURCE_DIR/.git rev-parse --short HEAD)
DATETIME=$(date '+%Y-%m-%dT%H:%M:%S')
VERSION=$(cat selfdrive/common/version.h | awk -F\" '{print $2}')
TINKLAVERSION=$(cat selfdrive/common/tinkla_version.h | awk -F[\"-]  '{print $2}')

echo "#define COMMA_VERSION \"$VERSION-Beta$TINKLA_BETA_NUMBER\"" > $TARGET_DIR/selfdrive/common/version.h
echo "#define TINKLA_VERSION \"$VERSION-$TINKLA_BETA_NUMBER\"" > $TARGET_DIR/selfdrive/common/tinkla_version.h

# in the directory
cd $TARGET_DIR
rm -f panda/board/obj/panda.bin.signed
rm -f panda/board/obj/ivs.bin.signed
git clean -xdf

#cleanup rednose
rm -rf rednose/helpers/*.o
rm -rf rednose/helpers/*.os
rm -rf rednose/helpers/__pycache__

echo "[-] committing version $VERSION T=$SECONDS"
git add -f .
git status
git commit -a -m "Tesla Unity v$VERSION-Beta$TINKLA_BETA_NUMBER"

echo "[-] Pushing to $PUSH T=$SECONDS"
git remote set-url origin git@github.com:boggyver/openpilot.git
git push -f origin tesla_unity_dev:tesla_unity_beta
rm -rf $TARGET_DIR
echo "[-] done T=$SECONDS"
