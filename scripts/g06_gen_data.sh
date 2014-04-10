#!/bin/bash

PROJECT_ROOT=../
EXE=$PROJECT_ROOT/mybins/cs296_06_exe
DATA=$PROJECT_ROOT/data

mkdir -p $DATA

for a in {1..40}
do
 for b in {1..20}
 do
  touch $DATA/g06out-$a-$b.txt
  $EXE $a > $DATA/g06out-$a-$b.txt
 done
done 

exit 0
