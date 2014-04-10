#!/bin/bash

old_IFS=$IFS
IFS=$'\n'
lines=($(du -all $1 | cat)); # array
IFS=$old_IFS

let "NoF = ${#lines[@]} - 1"
 
a=0
NoF_ans=0

while [ "$a" -lt "$NoF" ]
do
 temp=${lines["$a"]}
 temp=($temp)
 temp=${temp[1]}
 temp=${temp//*'/'/}
 temp=${temp//.*/.}
 if [ "$temp" != "." ]
 then
  let "NoF_ans = $NoF_ans + 1"
 fi
 let "a = $a + 1"
done

echo "Number of files and folders: $NoF_ans"
a=0

while [ "$a" -lt "$NoF" ]
do
 temp=${lines["$a"]}
 temp=($temp)
 temp=${temp[1]}
 flag=$( file $temp | grep executable)
 if [[ -x "$temp" ]] && [ "$flag" != "" ]
 then
  temp=${temp//*'/'/}
  echo "$temp"
 fi
 let "a = $a + 1"
done

exit 0
