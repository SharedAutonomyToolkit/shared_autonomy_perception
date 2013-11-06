#! /usr/bin/env bash

dir=$1
if [[ -z "$dir" ]]; then
    echo "Usage: extract_all.sh [dir]"
    echo "      will grab an image file from each bag file in dir and save it alongside the bag"
    exit
fi

for filename in $(echo $dir/*.bag); do
    imgfile=$(echo "$filename" | sed s/'.bag'/'.pgm'/)
    if [[ ! -e "$imgfile" ]]; then
        ./extract_images.py -f $filename
    fi
done