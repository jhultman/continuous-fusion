## Continuous fusion

Implementation of continuous fusion layer
for projecting FPV image into BEV using LIDAR.

Compile with:
`g++ -o makebag main.cpp `pkg-config opencv --cflags --libs``

Execute with:
`./makebag ~/Downloads/kitti_data/2011_09_26/2011_09_26_drive_0005_sync/`


Note on coordinate systems:

    Following (7) in Geiger et al., points x_velo 
    in velodyne coords are sent to points x_image 
    in the image plane of camera 2 using: 

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
