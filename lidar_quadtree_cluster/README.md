# Lidar Quadtree Cluster - 3D LiDAR based Object Detection

  Autoware package for core_perception
  Using one cpu. Not support GPU

  * purpose 
    * Making accurate and fast clustering algorithms that can be mounted on Self-driving system

    * <img src = "https://user-images.githubusercontent.com/19897466/89732964-3a34bc80-da8d-11ea-870a-b8008e1410fe.png" width="600" height="400">

  * core idea
    * Detects only the areas necessary for the vehicle to drive.
    * Detect only objects that exist on the road.
    * Quadtree segmentation clustering is performed using pointcloud data converted to 2D.
    * All objects detected in driving are obstacles to be avoided and need not be labelled separately.

## Input / Output Topics [ Subscribe / Publish Topic ]

  * `road_extractor` node's Input Topic
    * `/points_map` topic -> `map` frame
    * `/vector_map_info/point` topic -> `map` frame
    * `/localizer_pose` topic -> `map` frame

  * `road_extractor` node's Output Topic
    * `/local_road_points` topic

  * `quadtree_segmentation` node's Input Topic
    * `/points_raw` topic -> `velodyne` frame
    * `/local_road_points` topic  ( optional )
  
  * `quadtree_segmentation` node's Output Topic
    * `/obb_boxes` topic -> `velodyne` frame
    * `/transformed_obb_boxes` topic -> `map` frame
    * `/detection/lidar_detector/cloud_clusters` topic -> `map` frame
    * `/detection/lidar_detector/objects` topic -> `velodyne` frame

## Requirements

  * `road_extractor` node - optional
  * `/tf` topic `velodyne` to `map` - Necessarily

## Paramters

  ### road_extractors

| Paramter | Type | Description |
| -------- | ---- | ----------- |
| publish_topic_name | *String* | The topic name of road points data. (sensor_msgs::PointCloud2). default = `/local_road_points` |
| points_map_subscribe_name | *String* | The topic name of points map published by `/points_map_loader node`. (map frame) (sensor_msgs::PointCloud2) |
| localizer_pose_subscribe_name | *String* | The topic name of pose data published by `/ndt_matching node`. (map frame) (vector_map_msgs::PointArray) |
| road_width | *Int* | The value means the width of the road area you are looking for. |
| find_road_radius| *Double* | The value means the radius of the find area you are wanted to |

  ### quadtree_segmentation

| Paramter | Type | Description |
| -------- | ---- | ----------- |
| points_raw_topic | *String* | The topic name of points raw receiving from LiDAR sensor (sensor_msgs::PointCloud2). default = `/points_raw` |
| road_points_topic | *String* | The topic name of road points published by `/road_extractor node`. (velodyne frame) |
| height_from_ground_to_lidar | *Double* | The value means height from ground to lidar. |
| x_limit_forward | *Double* | The value means X+ axis area limit you want clustering to be desired |
| x_limit_backward | *Double* | The value means X- axis area limit you want clustering to be desired |
| y_limit | *Double* | The value means Y axis area limit you want clustering to be desired |
| z_high_limit | *Double* | The value means Z axis limit you want clustering to be desired |
| erase_road_threshold | *Double* | The value means threshold when erasing road points by using `/local_road_points` topic and `/points_raw` topic |
| box_threshold | *Double* | The value means threshold when you cluster boxes after quadtree segmentation |
| my_car_width | *Double* | The value means autodrive car's width. because LiDAR sensor often get my car's point data, we don't cluster the points |
| my_car_height | *Double* | The value means autodrive car's height. because LiDAR sensor often get my car's point data, we don't cluster the points |
| max_cluster_size | *Double* | The value means limit of number in one cluster |

## How to launch

  * From a sourced terminal:

    * `roslaunch lidar_quadtree_segmentation road_extractor`
    * `roslaunch lidar_quadtree_segmentation quadtree_segmentation`

## Process

if you get topics ( `/points_map` , `/vector_map_info/points`, `/localizer_pose` ), you put road_extractor node into practice.
so if you can get `/local_road_points` topic, you can see "ROAD based Version" in the terminal.
if you don't get `/local_raod_points` topic, you can see "FULL Version" in the terminal.

* road_extractor node
  1. subscribe points map and vector map and store datas in class member variable at constructor.
  2. Extract point map and vector map that exist in a certain area around localizer_pose x,y,z. ( local points map & local vector map(local dtlane point) ).
  3. As a result of combining local points map and local vector map, Extract road points. 
  4. publish topic (`/local_road_points`).

* quadtree_segmentation node

  * `ROAD based Version` Ordered
    1. Subcribe road points and points raw.
    2. Make a estimated road plane for converting 3D point cloud data to 2D point cloud data using points raw.
    3. Remove road points from points raw so we get filtered points raw.
    4. Project filtered points raw to estimated road plane (convert 3D point cloud data to 2D point cloud data).
    5. Perform quadtree segmentation using 2D point cloud data.
    6. clustering.
    7. Each cluster is restored to its z axis value.
    8. Create polygons and Oriented Bounding boxes for each cluster.
    9. Topic Publish.

  * `FULL Version` Ordered
    1. Subcribe points raw.
    2. Make a estimated road plane for converting 3D point cloud data to 2D point cloud data using points raw.
    3. Project points raw to estimated road plane (convert 3D point cloud data to 2D point cloud data).
    4. Perform quadtree segmentation using 2D point cloud data.
    5. clustering.
    6. Each cluster is restored to its z axis value.
    7. Create polygons and Oriented Bounding boxes for each cluster.
    8. Topic Publish.


* Extract Road points

    <img src = "https://user-images.githubusercontent.com/19897466/89732532-7ca8ca00-da8a-11ea-8451-ec0cc9147ed7.png" width="600" height="400">

* Make road plane using RANSAC

    <img src = "https://user-images.githubusercontent.com/19897466/89732543-8cc0a980-da8a-11ea-8fa9-2af90255b646.png" width="400" height="400">

* projection points raw to estimated road plane

    <img src = "https://user-images.githubusercontent.com/19897466/89732591-bb3e8480-da8a-11ea-9734-fde8b6feeec9.png" width="400" height="400">

    <img src = "https://user-images.githubusercontent.com/19897466/89732575-a19d3d00-da8a-11ea-947c-95a2f39e6b0d.png" width="600" height="400">

* quadtree

    <img src = "https://user-images.githubusercontent.com/19897466/89732601-c7c2dd00-da8a-11ea-89d0-8c090c8a083f.png" width="400" height="400">

* After clustering

    <img src = "https://user-images.githubusercontent.com/19897466/89732608-d5786280-da8a-11ea-89fd-e1b72ab8c0b2.png" width="400" height="400">

* result

    <img src = "https://user-images.githubusercontent.com/19897466/89732666-4750ac00-da8b-11ea-911a-e510c32a6747.png" width="400" height="400">

## Characteristic

  * Maximum Hz is 47Hz ( covering spped is up to four times the speed of rosbag )

## Video
  * 3D LiDAR based object detection ( with open planner )
  * https://www.youtube.com/watch?v=QCueXoDqrzw 