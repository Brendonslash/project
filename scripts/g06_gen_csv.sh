#!/bin/bash

PROJECT_ROOT=../
EXE=$PROJECT_ROOT/mybins/cs296_06_exe
DATA=$PROJECT_ROOT/data
CSV=$DATA/g06_lab05data_01.csv

rm -f $CSV
touch $CSV

mkdir -p $DATA

for a in {1..40}
do
 for b in {1..20}
 do
  old_IFS=$IFS
  IFS=$'\n'
  lines=($(cat $DATA/g06out-$a-$b.txt)) # array
  IFS=$old_IFS
  temp="$a,$b,${lines[1]#*: },${lines[2]#*: },${lines[3]#*: },${lines[4]#*: },${lines[5]#*: }"
  echo ${temp// ms/} >> $CSV
 done
done 

exit 0
