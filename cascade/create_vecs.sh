#!/usr/bin/env bash
i=0;
mkdir -p vecs
for img in pos/*jpg
do
    opencv_createsamples -img $img -vec vecs/$i.vec -num 100 -bg neg.txt
    let "i=i+1"
done
