CFG_FILE=/data/bb_openpilot.cfg
CFG_CONTENT=$(cat $CFG_FILE | sed -r '/[^=]+=[^=]+/!d' | sed -r 's/\s+=\s/=/g')
eval "export $'CFG_CONTENT'"
