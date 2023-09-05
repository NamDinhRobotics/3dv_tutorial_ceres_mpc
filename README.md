## An Invitation to 3D Vision (_update new examples_) : A Tutorial for Everyone
_An Invitation to 3D Vision_ is an introductory tutorial on 3D vision (a.k.a. geometric vision or visual geometry or multi-view geometry).
It aims to make beginners understand basic theory of 3D vision and implement their own applications using [OpenCV][].
In addition to tutorial slides, example codes are provided in the purpose of education. They include simple but interesting and practical applications. The example codes are written as short as possible (mostly __less than 100 lines__) to be clear and easy to understand.

* Download [tutorial slides](https://github.com/sunglok/3dv_tutorial/releases/download/misc/3dv_slides.pdf)
* Download [example codes in a ZIP file](https://github.com/sunglok/3dv_tutorial/archive/master.zip)
* Read [how to run example codes](https://github.com/sunglok/3dv_tutorial/blob/master/HOWTO_RUN.md)
* **Update new examples**: _fisheye un-distortion, Ceres Solver: tutorials and simple mpc controller_
  * fisheye un-distortion: [fisheye_undistortion.cpp] 
  * ceres_tutorial: [ceres_tut1, ceres_tut2, ceres_tut3]
  * simple mpc controller: [ceres_mpc.cpp]
### What does its name come from?
* The main title, _An Invitation to 3D Vision_, came from [a legendary book by Yi Ma, Stefano Soatto, Jana Kosecka, and Shankar S. Sastry](http://vision.ucla.edu/MASKS/). We wish that our tutorial will be the first gentle invitation card for beginners to 3D vision and its applications.
* The subtitle, _for everyone_, was inspired from [Prof. Kim's online lecture](https://hunkim.github.io/ml/) (in Korean). Our tutorial is also intended not only for students and researchers in academia, but also for hobbyists and developers in industries. We tried to describe important and typical problems and their solutions in [OpenCV][]. We hope readers understand it easily without serious mathematical background.

### Examples
* __Single-view Geometry__
  * Camera Projection Model
    * Object Localization and Measurement: [object_localization.cpp][] (result: [image](https://drive.google.com/open?id=10Lche-1HHazDeohXEQK443ruDTAmIO4E))
    * Image Formation: [image_formation.cpp][] (result: [image0](https://drive.google.com/file/d/0B_iOV9kV0whLY2luc05jZGlkZ2s/view), [image1](https://drive.google.com/file/d/0B_iOV9kV0whLS3M4S09ZZHpjTkU/view), [image2](https://drive.google.com/file/d/0B_iOV9kV0whLV2dLZHd0MmVkd28/view), [image3](https://drive.google.com/file/d/0B_iOV9kV0whLS1ZBR25WekpMYjA/view), [image4](https://drive.google.com/file/d/0B_iOV9kV0whLYVB0dm9Fc0dvRzQ/view))
    * Geometric Distortion Correction: [distortion_correction.cpp][] (result: [video](https://www.youtube.com/watch?v=HKetupWh4V8))
  * General 2D-3D Geometry
    * Camera Calibration: [camera_calibration.cpp][] (result: [text](https://drive.google.com/file/d/0B_iOV9kV0whLZ0pDbWdXNWRrZ00/view))
    * Camera Pose Estimation (Chessboard): [pose_estimation_chessboard.cpp][] (result: [video](https://www.youtube.com/watch?v=4nA1OQGL-ig))
    * Camera Pose Estimation (Book): [pose_estimation_book1.cpp][]
    * Camera Pose Estimation and Calibration: [pose_estimation_book2.cpp][]
    * Camera Pose Estimation and Calibration w/o Initially Given Camera Parameters: [pose_estimation_book3.cpp][] (result: [video](https://www.youtube.com/watch?v=GYp4h0yyB3Y))
* __Two-view Geometry__
  * Planar 2D-2D Geometry (Projective Geometry)
    * Perspective Distortion Correction: [perspective_correction.cpp][] (result: [original](https://drive.google.com/file/d/0B_iOV9kV0whLVlFpeFBzYWVadlk/view), [rectified](https://drive.google.com/file/d/0B_iOV9kV0whLMi1UTjN5QXhnWFk/view))
    * Planar Image Stitching: [image_stitching.cpp][] (result: [image](https://drive.google.com/file/d/0B_iOV9kV0whLOEQzVmhGUGVEaW8/view))
    * 2D Video Stabilization: [video_stabilization.cpp][] (result: [video](https://www.youtube.com/watch?v=be_dzYicEzI))
  * General 2D-2D Geometry (Epipolar Geometry)
    * Visual Odometry (Monocular, Epipolar Version): [vo_epipolar.cpp][]
    * Triangulation (Two-view Reconstruction): [triangulation.cpp][]
* __Multi-view Geometry__
  * Bundle Adjustment
    * Global Version: [bundle_adjustment_global.cpp][]
    * Incremental Version: [bundle_adjustment_inc.cpp][]
  * Structure-from-Motion
    * Global SfM: [sfm_global.cpp][]
    * Incremental SfM: [sfm_inc.cpp][]
  * Feature-based Visual Odometry and SLAM
    * Visual Odometry (Monocular, Epipolar Version): [vo_epipolar.cpp][]
    * Visual Odometry (Stereo Version)
    * Visual Odometry (Monocular, PnP and BA Version)
    * Visual SLAM (Monocular Version)
  * Direct Visual Odometry and SLAM
    * Visual Odometry (Monocular, Direct Version)
  * c.f. The above examples need [Ceres Solver][] for bundle adjustment.
* __Correspondence Problem__
  * Line Fitting with RANSAC: [line_fitting_ransac.cpp][]
  * Line Fitting with M-estimators: [line_fitting_m_est.cpp][]
* **Appendix**
  * Line Fitting
  * Planar Homograph Estimation
  * Fundamental Matrix Estimation




### Dependencies
* [OpenCV][] (> 3.0.0, 3-clause BSD License)
  * _OpenCV_ is a base of all example codes for basic computer vision algorithms, linear algebra, image/video manipulation, and GUI.
* [Ceres Solver][] (3-clause BSD License): A numerical optimization library
  * _Ceres Solver_ is additionally used by m-estimator, bundle adjustment, structure-from-motion, and visual odometry/SLAM.

---
### __New Version! Python Examples Added!__
  * You can see the same examples as above in the following [script](scripts).
  * We recommend running the examples in Anaconda.


### Dependencies
It was tested in the following version.

* [OpenCV][] (4.2.0v)
* [Scipy](https://scipy.org/) (1.8.0v)
  * SciPy (pronounced “Sigh Pie”) is an open-source software for mathematics, science, and engineering.
* [Numpy](https://numpy.org/) (1.22.3v)
  * The fundamental package for scientific computing with Python
* [Open3D](http://www.open3d.org/) (0.13.0v)
  * Open3D is an open-source library that supports rapid development of software that deals with 3D data.

---
### License
* [Beerware](http://en.wikipedia.org/wiki/Beerware)

### Authors
* [Sunglok Choi](http://sites.google.com/site/sunglok/) (sunglok@hanmail.net)
* [JunHyeok Choi](https://mint-lab.github.io/members/) (dkwnsgur12@gmail.com)

### Acknowledgement
The authors thank the following contributors and projects.

* [Jae-Yeong Lee](https://sites.google.com/site/roricljy/): He motivated many examples.
* [Giseop Kim](https://sites.google.com/view/giseopkim): He contributed the initial version of SfM codes with [Toy-SfM](https://github.com/royshil/SfM-Toy-Library) and [cvsba](https://www.uco.es/investiga/grupos/ava/node/39).
* [The KITTI Vision Benchmark Suite](http://www.cvlibs.net/datasets/kitti/): The KITTI odometry dataset #07 was used to demonstrate visual odometry and SLAM.
* [Russell Hewett](https://courses.engr.illinois.edu/cs498dh3/fa2013/projects/stitching/ComputationalPhotograph_ProjectStitching.html): His two hill images were used to demonstrate image stitching.
* [Kang Li](http://www.cs.cmu.edu/~kangli/code/Image_Stabilizer.html): His shaking CCTV video was used to demonstrate video stabilization.
* [Richard Blais](http://www.richardblais.net/): His book cover and video in [the OpenCV tutorial](http://docs.opencv.org/3.1.0/dc/d16/tutorial_akaze_tracking.html) were used to demonstrate camera pose estimation and augmented reality.

[OpenCV]: http://opencv.org/
[Ceres Solver]: http://ceres-solver.org/

[object_localization.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/object_localization.cpp
[image_formation.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/image_formation.cpp
[distortion_correction.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/distortion_correction.cpp
[camera_calibration.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/camera_calibration.cpp
[pose_estimation_chessboard.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/pose_estimation_chessboard.cpp
[pose_estimation_book1.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/pose_estimation_book1.cpp
[pose_estimation_book2.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/pose_estimation_book2.cpp
[pose_estimation_book3.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/pose_estimation_book3.cpp
[perspective_correction.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/perspective_correction.cpp
[image_stitching.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/image_stitching.cpp
[video_stabilization.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/video_stabilization.cpp
[vo_epipolar.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/vo_epipolar.cpp
[triangulation.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/triangulation.cpp
[bundle_adjustment_global.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/bundle_adjustment_global.cpp
[bundle_adjustment_inc.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/bundle_adjustment_inc.cpp
[sfm_global.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/sfm_global.cpp
[sfm_inc.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/sfm_inc.cpp
[line_fitting_ransac.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/line_fitting_ransac.cpp
[line_fitting_m_est.cpp]: https://github.com/sunglok/3dv_tutorial/blob/master/examples/line_fitting_m_est.cpp
