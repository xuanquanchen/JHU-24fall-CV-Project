# JHU-24fall-CV-Project

## Project Overview

This repository implements the Fall 2024 Computer Vision assignment at Johns Hopkins University. Its goal is to reconstruct the 3D geometry of a tabletop scene from RGB‑D data and then extract the central axis of a cylindrical object (a cup) to inform downstream robotic grasp planning.

### Summary:
Leveraging an RGB‑D sensor and open‑source tools, this pipeline reconstructs a dense 3D point cloud of a tabletop scene by converting depth maps into point clouds and fusing multiple views ([Open3D][1])([Signal Processing Stack Exchange][2]). After isolating the cup via segmentation, we apply the game‑theoretic cylinder extraction algorithm of Bergamasco *et al.* to robustly estimate the central axis through clustering of axis hypotheses ([Iris][3], [CoLab][4]). Finally, the resulting origin and unit direction are exported as a ROS `moveit_msgs::Grasp` message to drive a MoveIt‑based pick‑and‑place routine ([ROS Docs][5]).

## Pipeline Components

### 1. RGB‑D Data Acquisition & Point Cloud Reconstruction

* **Capture & Registration:** Acquire synchronized RGB and depth frames from an RGB‑D sensor and register them into a common camera frame. ([Open3D][1])
* **Depth‑to‑Point‑Cloud Conversion:** Convert each depth map into a 3D point cloud using intrinsic camera parameters and inverse projection. ([Medium][6])
* **Multi‑View Fusion:** Align and merge partial point clouds from different viewpoints via pairwise registration and volumetric averaging, yielding a unified, dense cloud of the scene. ([Signal Processing Stack Exchange][2])

### 2. Region‑of‑Interest Extraction

Segment the fused cloud to isolate the cup. Common approaches include color‑based thresholding in RGB space or Euclidean cluster extraction on the geometric data. Filter and downsample the resulting cup point set to suppress noise and reduce computation.

### 3. Cylinder Axis Estimation

* **Clustering Framing:** We employ Bergamasco *et al.* (2020), which casts cylinder detection as clustering in the space of axis hypotheses, eliminating the need for surface normals ([Iris][3]).
* **Hypothesis Generation:** Randomly oriented slicing planes intersect the cup surface in ellipses; each slice yields a candidate axis defined by its center and direction via ellipse fitting ([CoLab][4]).
* **Pairwise Similarity:** The similarity between any two hypotheses is measured by the rigid‑motion distance in SE(3), encoded as the length of the shortest geodesic on the unit dual‑quaternion manifold ([CoLab][4]).
* **Game‑Theoretic Inlier Selection:** Treat hypotheses as players in an evolutionary game; iteratively apply replicator dynamics on the payoff matrix of pairwise similarities until convergence, revealing the inlier set ([CoLab][4]).
* **Axis Refinement:** Fit a least‑squares line through the centers of inlier hypotheses to obtain the cylinder’s origin and unit direction ([Iris][3]). Optionally, refine these parameters via RANSAC‑based optimization followed by nonlinear least‑squares (e.g., Levenberg–Marquardt) for enhanced robustness to noise ([GitHub][7]).

### 4. Robotic Grasp Planning Interface (Future work)

Export the estimated axis as a `moveit_msgs::Grasp` message in ROS, defining the approach vector and grasp posture. This message can be fed into a MoveIt‑based pick‑and‑place pipeline to execute reliable cylindrical grasps on a robotic arm ([ROS Docs][5]).

[1]: https://www.open3d.org/docs/latest/tutorial/Basic/rgbd_image.html?utm_source=chatgpt.com "RGBD images — Open3D latest (664eff5) documentation"
[2]: https://dsp.stackexchange.com/questions/70250/making-a-3d-point-cloud-from-multiple-rgb-d-images?utm_source=chatgpt.com "Making a 3D point cloud from multiple RGB-D images"
[3]: https://iris.unive.it/retrieve/e4239dde-0321-7180-e053-3705fe0a3322/1-s2.0-S0031320320302466-main.pdf?utm_source=chatgpt.com "[PDF] Cylinders extraction in non-oriented point clouds as a ... - IRIS"
[4]: https://colab.ws/articles/10.1016%2Fj.patcog.2020.107443?utm_source=chatgpt.com "Cylinders extraction in non-oriented point clouds as a clustering ..."
[5]: https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/pick_place/pick_place_tutorial.html?utm_source=chatgpt.com "Pick and Place Tutorial — moveit_tutorials Kinetic documentation"
[6]: https://medium.com/%40mustafaboyuk24/obtaining-point-cloud-from-depth-images-with-intel-realsense-d-435-camera-144e8ef9260d?utm_source=chatgpt.com "Obtaining Point Cloud from Depth Images with Intel RealSense D ..."
[7]: https://github.com/leomariga/pyRANSAC-3D/issues/13?utm_source=chatgpt.com "Cylinder Fitting · Issue #13 · leomariga/pyRANSAC-3D - GitHub"
