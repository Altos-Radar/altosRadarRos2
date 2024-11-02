# altosRadarRos2

## Build and Run
1. git clone https://github.com/Altos-Radar/altosRadarRos2.git
2. cd altosRadarRos2
3. colcon build 
4. bash start.sh

## Details of Output pointcloud structure
|Field | Value|
|---------|---------------|
|x |x-axis coordinate in radar frame|
|y |y-axis coordinate in radar frame|　
|z |z-axis coordinate in radar frame|　
|h |doppler of point|　
|s |RCS of point|　
|v |direction of point (-1:opposite 0:static 1:same)|　