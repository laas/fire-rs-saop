FILES=$(ls *.asc)

for FILE in $FILES; do
    OUTFILE=$(basename "$FILE" ".asc").tif
    if [ -f $OUTFILE ]; then
        echo "Skipping $FILE"
    else
        gdal_translate -of GTiff -a_srs "EPSG:2154" $FILE $OUTFILE
    fi
done


