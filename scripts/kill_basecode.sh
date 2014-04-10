#!/bin/bash

PROJECT_ROOT=../
EXE=$PROJECT_ROOT/mybins/cs296_06_exe
Ex=cs296_06_exe

PID=($(ps u -a| grep $Ex | cat))
 
usr=${PID[0]}
PID=${PID[1]}

Cusr=$(id -un)

echo "Process id: $PID User id: $usr"

if [ "$Cusr" == "$usr" ]
then
 read -p "Kill basecode (y/n)" inp
 if [ "$inp" == "y" ]
 then
  kill $PID
 fi
fi

exit 0




