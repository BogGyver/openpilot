#!/usr/bin/bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

if [ -z "$PASSIVE" ]; then
  export PASSIVE="1"
fi

. /data/openpilot/selfdrive/car/tesla/readconfig.sh


function launch {
  # apply update
  if [ $do_auto_update == "True" ]; then
    if [ "$(git rev-parse HEAD)" != "$(git rev-parse @{u})" ]; then
      git reset --hard @{u} &&
      git clean -xdf &&

      # Touch all files on release2 after checkout to prevent rebuild
      BRANCH=$(git rev-parse --abbrev-ref HEAD)
      if [[ "$BRANCH" == "release2" ]]; then
          touch **
      fi

      exec "${BASH_SOURCE[0]}"
    fi
  fi
  
  # no cpu rationing for now
  echo 0-3 > /dev/cpuset/background/cpus
  echo 0-3 > /dev/cpuset/system-background/cpus
  echo 0-3 > /dev/cpuset/foreground/boost/cpus
  echo 0-3 > /dev/cpuset/foreground/cpus
  echo 0-3 > /dev/cpuset/android/cpus

  # handle pythonpath
  ln -s /data/openpilot /data/pythonpath
  export PYTHONPATH="$PWD"

  # start manager
  cd selfdrive
  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
