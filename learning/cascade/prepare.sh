#!/usr/bin/env bash

base_dir=../../assets/cascade/training_set

if test -f neg.txt; then
    find $base_dir/neg/* > neg.txt
    find $base_dir/bg/* >> neg.txt
fi
python2 make_pos_list.py $base_dir/pos > pos.txt

./create_vecs.sh $base_dir/pos
python2 mergevec.py -v vecs -o pos.vec
