#!/usr/bin/env bash

if $2; then
    mkdir -p cascade
    rm -fr cascade/*
fi


echo $1
if test -f $1/opencv_traincascade
then
    echo use not std opencv
    exec $1/opencv_traincascade -data cascade -vec pos.vec -bg neg.txt -numPos 2500 -numNeg 2000  -maxWeakCount 300 -numThreads 8
else
    echo use std opencv
    opencv_traincascade -data cascade -vec pos.vec -bg neg.txt -numPos 2500 -numNeg 2000  -maxWeakCount 300 -precalcIdxBufSize 2048 -precalcValBufSize 2048
fi