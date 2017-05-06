#!/usr/bin/env bash
# generate vecs from small data set

i=0
mkdir -p vecs
for img in $1/*
do
    opencv_createsamples -img $img -vec vecs/$i.vec -num 500 -bg bg.txt 
    let "i=i+1"
done
