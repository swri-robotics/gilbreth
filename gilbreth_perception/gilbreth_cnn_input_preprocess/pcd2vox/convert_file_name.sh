#!/bin/bash

while read -r old new
do
  mv "vox/${old}.binvox" "vox/${new}.${old}.01.binvox"
done < map.txt
