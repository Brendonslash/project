#!/bin/bash

PROJECT_ROOT=../
SRC=$PROJECT_ROOT/src

if [ $1 = 1 ]; then
list=$(ls $SRC/*.cpp)
for l in $list
do
mv $l $l.bak
done
elif [ $1 = 2 ]; then
cd $SRC; ls *.cpp| sed -e "p;s/\.cpp$/\.cpp\.bak/"|xargs -n2 mv
fi

exit 0
