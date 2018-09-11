#!/bin/bash

FILES=pcd/*.pcd
for f in $FILES
do
  echo "Processing $f file..."
  OUTFN="$(basename $f .pcd).binvox"
  pcd2binvox -d 32 $f "vox/$OUTFN"
  echo "Save to vox/$OUTFN"
done
