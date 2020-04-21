#!/bin/bash
echo "***********************************"
echo "*     Keras2ONXX                  *"
echo "***********************************"
WORKING_DIR=/data/intermediate
echo " "
echo " "
echo " Converting $1 to $2"
echo " "
echo " Cleaning intermediate data..."
echo " "
if [ -d "$WORKING_DIR" ]; then rm -Rf $WORKING_DIR; fi
echo " "
echo " Converting OP Keras to Saved_Model... "
echo " "
/data/openpilot/tools/keras/save_interm.py $1 $WORKING_DIR
echo " "
echo "  Converting Saved_Model to ONXX"
echo " "
python -m tf2onnx.convert --saved-model $WORKING_DIR --output $2
echo " "
echo " Cleaning up..."
echo " "
if [ -d "$WORKING_DIR" ]; then rm -Rf $WORKING_DIR; fi
echo " "
echo " Conversion completed! "
echo " "

