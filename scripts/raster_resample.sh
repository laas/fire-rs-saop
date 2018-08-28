#!/usr/bin/env bash

FILES=$(ls *.tif)
RESULT_DIR="$(pwd)/converted/"

OUT_RES=5

mkdir -p "converted"

for FILE in ${FILES}; do
    OUTFILE=${FILE}
    if [ -f ${RESULT_DIR}${OUTFILE} ]; then
        echo "Skipping ${FILE}"
    else
        gdalwarp -co compress=lzw -of GTiff -tr ${OUT_RES} ${OUT_RES} ${FILE} ${RESULT_DIR}${OUTFILE}
    fi

done


