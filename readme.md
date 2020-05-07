## Continuous fusion

C++ implementation of continuous fusion from [Deep Continuous Fusion for Multi-Sensor 3D Object Detection](http://openaccess.thecvf.com/content_ECCV_2018/papers/Ming_Liang_Deep_Continuous_Fusion_ECCV_2018_paper.pdf).

A fast, vectorized SciPy version is available if there is any interest.

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

Diagram borrowed from paper (we skip MLP):

![Fusion](/images/fusion.png)
