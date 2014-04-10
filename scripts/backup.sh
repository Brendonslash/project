#! /bin/bash

cp -R -u src $(df -P | awk 'END{print$6}')/$1

#rsync -R -u -v src $(df -P | awk 'END{print$6}')/$1
