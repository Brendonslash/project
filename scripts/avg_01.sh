#!/bin/bash

PROJECT_ROOT=../
EXE=$PROJECT_ROOT/mybins/cs296_06_exe
DATA=$PROJECT_ROOT/data
CSVI=$DATA/g06_lab05data_01.csv
DATO=$DATA/temp_avg_01.dat

rm -f $DATO
touch $DATO

mkdir -p $DATA

for a in {1..40}
do
 let "s = ($a - 1) * 20 + 1"
 let "e = $a * 20"
 lines=$(sed -n "$s,$e"'p' $CSVI|cat)
 i=$(awk -v var="$lines" 'BEGIN{split(var,list,"\n"); 
	len=50;
	for(i=2;i<=6;i++) array[i] = 0;
	for(i=1;i<=len;i++) {
		split(list[i],temp,",");
		array[2] = array[2] + temp[3];
		array[3] = array[3] + temp[7];
		array[4] = array[4] + temp[4];
		array[5] = array[5] + temp[5];
		array[6] = array[6] + temp[6];
	}
	for(i=2;i<=6;i++) print (array[i] / len);}')
 i=($i)
 echo "$a ${i[0]} ${i[1]} ${i[2]} ${i[3]} ${i[4]}" >> $DATO
done 

exit 0
