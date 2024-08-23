# map_changer  
Change map with waypoint  
## Install  
```bash
git clone https://github.com/kazukishirasu/map_changer.git
catkin build map_changer
source ~/.bashrc
```
## Usage  
#### waypoint placement  
Place a waypoint at the robot's position after the map change, like waypoint number 2.  
<img width="500" alt="" src="https://github.com/kazukishirasu/map_changer/blob/master/img/before_change_map.png">
<img width="500" alt="" src="https://github.com/kazukishirasu/map_changer/blob/master/img/after_change_map.png">  
  
#### configuration file:[map_changer/config/test1.yaml](https://github.com/kazukishirasu/map_changer/blob/master/config/test1.yaml)  
```yaml
config:
    # Specify the waypoint_id when you want to change the map
  - waypoint_id: registed_0_1252124000000
    # Specify the full path to the map file you want to change without the extension
    map_file: /home/kazuki/tsukuba_ws/src/orne-box/orne_box_navigation_executor/maps/mymap2
  - waypoint_id: registed_0_1609011000000
    map_file: /home/kazuki/tsukuba_ws/src/orne-box/orne_box_navigation_executor/maps/mymap1
```
  
#### launch file:[map_changer/launch/map_changer.launch](https://github.com/kazukishirasu/map_changer/blob/master/launch/map_changer.launch)  
```xml
<launch>
<!-- Set configuration file -->
<arg name="file_path"   default="$(find map_changer)/config/test1.yaml"/>
<!-- The number of seconds after arriving at a waypoint before the map changes -->
<arg name="wait_time"   default="10.0"/>

<node pkg="map_changer" type="map_changer_node" name="map_changer_node" output="screen">
    <param name="file_path" value="$(arg file_path)"/>
    <param name="wait_time" value="$(arg wait_time)"/>
</node>
</launch>
```
  
#### launch  
```bash
roslaunch map_changer map_changer.launch
```
