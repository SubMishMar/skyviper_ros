#!/bin/bash
for i in `seq 1 20`;
do
 echo marker: $i
 ./aruco_marker_generator "image$i.png" -d=10 -id=$i -ms=400
done    
