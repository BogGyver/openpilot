#!/bin/bash

while read hash submodule ref; do
  git -C $submodule fetch --depth 100 origin tesla_unity_dev
  git -C $submodule branch -r --contains $hash | grep "origin/tesla_unity_dev"
  if [ "$?" -eq 0 ]; then
    echo "$submodule ok"
  else
    echo "$submodule: $hash is not on tesla_unity_dev"
    exit 1
  fi
done <<< $(git submodule status --recursive)
