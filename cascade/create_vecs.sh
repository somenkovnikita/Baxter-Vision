i=0;
for img in pos/*jpg
do
    opencv_createsamples -img $img -vec vecs/$i.vec -num 10 -bg bg.txt -maxxangle 0.5  -maxyangle 0.5
    let "i=i+1"
done
