#!/bin/bash

PROJECT_ROOT=../
EXE=$PROJECT_ROOT/mybins/cs296_06_exe
DATA=$PROJECT_ROOT/data
CSVI=$DATA/g06_lab05data_01.csv
CSVIR=$DATA/g06_lab05data_random.csv
DATO=$DATA/temp_avg_05.dat

rm -f $DATO
touch $DATO
mkdir -p $DATA

for a in {1..40}
do
 let "s = ($a - 1) * 20 + 1"
 let "e = $a * 20"
 let "sr = ($a - 1) * 10 + 1"
 let "er = $a * 10"

 lines=$(sed -n "$s,$e"'p' $CSVI |cat)
 linesR=$(sed -n "$sr,$er"'p' $CSVIR |cat)
 
 i=$(awk -v var="$lines" 'BEGIN{split(var,list,"\n"); 
	len=20;
	for(i=1;i<=len;i++) {
		split(list[i],temp,",");
		ans = ans + temp[3];}
	print (ans / len);}')
 iR=$(awk -v var="$linesR" 'BEGIN{split(var,list,"\n"); 
	len=10;
	for(i=1;i<=len;i++) {
		split(list[i],temp,",");
		ans = ans + temp[3];}
	print (ans / len);}')
 echo "$a $i $iR" >> $DATO
done 

exit 0
