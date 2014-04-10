#!/bin/bash

PROJECT_ROOT=../
EXE=$PROJECT_ROOT/mybins/cs296_06_exe
DATA=$PROJECT_ROOT/data
CSVI=$DATA/g06_lab05data_01.csv
DATO=$DATA/temp_ebars.dat

rm -f $DATO
touch $DATO

mkdir -p $DATA

for a in {1..40}
do
 let "s = ($a - 1) * 20 + 1"
 let "e = $a * 20"
 lines=$(sed -n "$s,$e"'p' $CSVI |cat)
 i=$(awk -v var="$lines" 'BEGIN{split(var,list,"\n");
	len=20;
	ma=-1;
	mi=1000;
	for(i=1;i<=len;i++) {
		split(list[i],temp,",");
		step = step + temp[3];
		if(mi > temp[3]) mi = temp[3];
		if(ma < temp[3]) ma = temp[3];
	}
	print (step / len);
	print (ma - mi);}')
 i=($i)
 echo "$a ${i[0]} ${i[1]}" >> $DATO
done 

exit 0
