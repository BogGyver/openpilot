if [ -z "$BASEDIR" ]; then
  BASEDIR="/data/openpilot"
fi
CFG_FILE="$BASEDIR/../bb_openpilot.cfg"
CFG_CONTENT=$(cat $CFG_FILE | sed -r "s/'/SINGLE_Q/" | sed -r '/[^=]+=[^=]+/!d' | sed -r 's/\s+=\s/=/g' | sed -e 's/[[:space:]]*\=[[:space:]]*/=/g' \
        -e 's/#.*$//' \
        -e 's/[[:space:]]*$//' \
        -e 's/^[[:space:]]*//' \
        -e "s/^\(.*\)=\([^\"']*\)$/\1=\"\2\"/" | sed -r "s/SINGLE_Q/'/" )
eval "export $CFG_CONTENT"
