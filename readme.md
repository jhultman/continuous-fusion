## Continuous fusion

Implementation of continuous fusion layer
for projecting FPV image into BEV using LIDAR.

![Fusion](/images/demo.gif)

Comments on coordinate systems:

    Points x_velo in velodyne coords are sent to points x_image 
    in the image plane of camera 2 using (7) in Geiger et al.:

        x_image = P * R * T * x_velo, 
        
    where:

        x_velo  - point in velodyne coords (x, y, z, 1).
        T       - velodyne coords to unrectified cam0 coords.
        R       - unrectified cam0 coords to rectified cam 0 coords,
        P       - rectified cam0 coords to image plane of cam2,
        x_image - point in image of cam2 (u, v, 1).

    Note that in this module we use left-multiplying active 
    (alibi) coordinate transformations to maintain consistency 
    with KITTI. ROS uses the opposite passive (alias) 
    transformation convention.
