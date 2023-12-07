#!/bin/bash

while read hash submodule ref; do
  git -C $submodule fetch --depth 1000 origin tesla_unity_devC3
  git -C $submodule branch -r --contains $hash | grep "origin/tesla_unity_devC3"
  if [ "$?" -eq 0 ]; then
    echo "$submodule ok"
  else
    echo "$submodule: $hash is not on tesla_unity_dev"
    exit 1
  fi
done <<< $(git submodule status --recursive)
