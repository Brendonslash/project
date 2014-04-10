#!/bin/bash

PROJECT_ROOT=../
EXE=$PROJECT_ROOT/mybins/cs296_06_exe
DATA=$PROJECT_ROOT/data
CSVI=$DATA/g06_lab05data_02.csv
DATO=$DATA/temp_col.dat

rm -f $DATO
touch $DATO

iter=20

let "s = ($iter - 1) * 20 + 1"
let "e = $iter * 20"
old_IFS=$IFS
IFS=$'\n'
lines=($(sed -n "$s,$e"'p' $CSVI |cat)) # array
IFS=$old_IFS
for a in {0..49}
do
 old_IFS=$IFS
 IFS=$'\,'
 temp=(${lines[$a]}) # array
 IFS=$old_IFS
 echo "${temp[2]}" >>$DATO
done
exit 0
