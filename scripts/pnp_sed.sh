#!/bin/bash

#Fitzwilliam - Sashank
#Darcy - Manik
#Elizabeth - Sundeep
PROJECT_ROOT=../
DATA=$PROJECT_ROOT/data
IN=$DATA/pnp_austen.txt
OUT=$DATA/pnp_austen_cs296.txt

sed -e s/\'\'/\"/g -e s/Fitzwilliam/Sashank/g -e s/Darcy/Manik/g -e s/Elizabeth/Sundeep/g < $IN > $OUT

exit 0



