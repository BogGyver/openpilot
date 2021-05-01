#!/usr/bin/env bash
set -e

export GIT_COMMITTER_NAME="BogGyver"
export GIT_COMMITTER_EMAIL="bogdan.butoi@gmail.com"
export GIT_AUTHOR_NAME="BogGyver"
export GIT_AUTHOR_EMAIL="bogdan.butoi@gmail.com"
export GIT_SSH_COMMAND="ssh -i /data/gitkey"

# set CLEAN to build outside of CI

# Create folders
rm -rf /data/openpilot_release
mkdir -p /data/openpilot_release
cd /data/openpilot_release

# Create git repo
git init
git remote add origin git@github.com:boggyver/openpilot.git
git fetch origin tesla_unity_beta


git fetch origin tesla_unity_release

# Create tesla_unity_release with no history
git checkout --orphan tesla_unity_release origin/tesla_unity_beta


VERSION=$(cat selfdrive/common/version.h | awk -F[\"-]  '{print $2}')
TINKLAVERSION=$(cat selfdrive/common/tinkla_version.h | awk -F[\"-]  '{print $2}')
echo "#define COMMA_VERSION \"$VERSION-release\"" > selfdrive/common/version.h
echo "#define TINKLA_VERSION \"$TINKLAVERSION-release\"" > selfdrive/common/tinkla_version.h

git commit -m "Tesla OpenPilot $TINKLAVERSION (openpilot v$VERSION)"

# Build signed panda firmware
pushd panda/
CERT=/data/pandaextra/certs/debug RELEASE=0 scons -u .
mv board/obj/panda.bin.signed /tmp/panda.bin.signed
popd

# Build stuff
ln -sfn /data/openpilot /data/pythonpath
export PYTHONPATH="/data/openpilot:/data/openpilot/pyextra"
SCONS_CACHE=1 scons -j3

# Run tests
tmux kill-session -t comma || true
python selfdrive/manager/test/test_manager.py
selfdrive/car/tests/test_car_interfaces.py

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
mkdir -p panda/board/obj
mv /tmp/panda.bin.signed panda/board/obj/panda.bin.signed

# Restore phonelibs
git checkout phonelibs/

# Mark as prebuilt release
touch prebuilt

# Add built files to git
git add -f .
git commit --amend -m "Tesla OpenPilot $TINKLAVERSION (openpilot v$VERSION)"

# Print committed files that are normally gitignored
#git status --ignored


git remote set-url origin git@github.com:boggyver/openpilot.git

# Push to tesla_unity_release
git push -f origin tesla_unity_release

