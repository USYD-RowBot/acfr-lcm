#!/bin/sh
# 20040206 OP Created

# tif2png.sh converts all tif images in a directory into 8 bit png
# and places them in a texture directory

# check that directory exists, if not create it
if [ ! -d texture ]
then
    mkdir texture
fi


# change to a normalized, smaller png
for file in *.tif
do
    convert $file -depth 8 -normalize -resize 640x512 texture/${file%%tif}png
    printf " Processing image %s \r" $file 
done



