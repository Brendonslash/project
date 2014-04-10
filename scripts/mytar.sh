#!/bin/bash

PROJECT_ROOT=../
SRC=$PROJECT_ROOT/src
DATA=$PROJECT_ROOT/data

if [ $1 == "1" ]
then
        tar czf $DATA/src.tar.gz $SRC
elif [ $1 == "2" ]
then 
        for file in $SRC/*; do gzip < "$file" > "$file.gz"; done
	tar -cf $DATA/src.tar $(ls $SRC/*.gz)
	rm $SRC/*.gz
else
        echo "Invalid/insufficient arguments provided."
fi
