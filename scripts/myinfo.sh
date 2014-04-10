#!/bin/bash


day_now=$(date +"%A")
date_now=$(date +"%d")
month_now=$(date +"%B")
year_now=$(date +"%Y")
echo "Today is "$day_now"," $date_now $month_now"," $year_now"."
# Today is Monday, 11 February, 2014.

let "year_next = $year_now + 1"

uptime=$(</proc/uptime)
uptime=${uptime%%.*}

seconds=$(( uptime%60 ))
minutes=$(( uptime/60%60 ))
hours=$(( uptime/60/60%24 ))
days=$(( uptime/60/60/24 ))


date1=$(date -d"$year_now-10-16" +"%s") #sundeep 
date2=$(date -d"$year_now-7-5" +"%s")  #sashank
date3=$(date -d"$year_now-9-30" +"%s") #manik

datenow=$(date +"%s") #now

if [ "$datenow" -gt "$date1" ]
then
date1=$(date -d"$year_next-10-16" +"%s")
fi

if [ "$datenow" -gt "$date2" ]
then
date2=$(date -d"$year_next-7-5" +"%s")
fi

if [ "$datenow" -gt "$date3" ]
then
date3=$(date -d"$year_next-9-30" +"%s")
fi

diffsec1=$((date1-datenow))
diffweeks1=$(($diffsec1/(86400*7)))
diffdays1=$((($date1-($datenow+$diffweeks1*7*86400))/(86400)))
diffhours1=$((($date1-(($datenow+$diffweeks1*7*86400)+($diffdays1*86400)))/3600))

diffsec2=$((date2-datenow))
diffweeks2=$(($diffsec2/(86400*7)))
diffdays2=$((($date2-($datenow+$diffweeks2*7*86400))/(86400)))
diffhours2=$((($date2-(($datenow+$diffweeks2*7*86400)+($diffdays2*86400)))/3600))

diffsec3=$((date3-datenow))
diffweeks3=$(($diffsec3/(86400*7)))
diffdays3=$((($date3-($datenow+$diffweeks3*7*86400))/(86400)))
diffhours3=$((($date3-(($datenow+$diffweeks3*7*86400)+($diffdays3*86400)))/3600))

echo "There are "$diffweeks1" weeks "$diffdays1" days and "$diffhours1" hours left for the birthday of Sundeep Routhu"
echo "There are "$diffweeks2" weeks "$diffdays2" days and "$diffhours2" hours left for the birthday of Sashank Gondala"
echo "There are "$diffweeks3" weeks "$diffdays3" days and "$diffhours3" hours left for the birthday of Manik"

echo "Thank you for asking, $USER"

echo "Your system has been running for "$days" days, "$hours":"$minutes" hours."

homelocation=$(df)
homelocation=($homelocation)
echo "The current disk on which your home folder is located is "${homelocation[7]}" and is "${homelocation[11]}" full"

version=$(lsb_release --release | cut -f2)
ubuntuname=$(lsb_release -c | awk '{print$2}')
# $(uname -o)
kernalname=$(uname -r)
echo "You are running "$ubuntuname" and "$version" with Kernel "$kernalname

awk '/MemTotal:/{total=$2} \
       /MemFree:/{free=$2} \
       END{ \
        print "Your machine has "(total/1024/1024)" GB RAM. Of which "((total-free)*100/total)"% is in use." \
       }' /proc/meminfo

numchars=$(wc -c < ./myinfo.sh)
numlines=$(wc -l < ./myinfo.sh)
numwords=$(wc -w < ./myinfo.sh)

echo "This script has "$numwords" words, "$numlines" lines and "$numchars" characters (without counting whitespace)"
