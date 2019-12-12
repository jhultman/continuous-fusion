## Continuous fusion

C++ implementation of the continuous fusion operation
from the ECCV 2018 paper
[Deep Continuous Fusion for Multi-Sensor 3D Object Detection](http://openaccess.thecvf.com/content_ECCV_2018/papers/Ming_Liang_Deep_Continuous_Fusion_ECCV_2018_paper.pdf).

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

    (We use left-multiplying active/alibi coordinate transformations.)

Diagram borrowed from paper (we do not implement MLP):

![Fusion](/images/fusion.png)
