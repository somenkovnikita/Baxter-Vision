#!/usr/bin/env bash

if $2; then
    mkdir -p cascade
    rm -fr cascade/*
fi


echo $1
if test -f $1/opencv_traincascade
then
    echo use not std opencv
    exec $1/opencv_traincascade -data cascade -vec pos.vec -bg neg.txt -numPos 10000 -numNeg 10000  -maxWeakCount 300
else
    echo use std opencv
    opencv_traincascade -data cascade -vec pos.vec -bg neg.txt -numPos 10000 -numNeg 10000  -maxWeakCount 300
fi