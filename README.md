# Reconstruction of office from a set of 3D point clouds

#### Goal:  
Reconstruct an office room from a set of 3D point clouds acquired from an Intel RealSense depth sensor that was moved to scan different views

#### Data: 
The data comes from a set of 3D point clouds acquired by an Intel RealSense depth sensor.
It  consists of N=40 frames of 3D point cloud data, where each frame is from a different viewpoint. 
The data is stored in a cell structure with N cells, each with 2 entries: X.Color which is the RGB value of each point and X.Location, which is the XYZ location of each point (and X is the cell
Index. 
The original RGB or XYZ images are 480 rows by 640 columns by 3 channels but are encoded as a list of points(480*640,3).

---
Image from the left: The image at frame 12, point clouds extracted.
![](https://github.com/weitat95/advisioncw1819/blob/master/images/img1.png)

---
Matching of SIFT descriptors with matched points in green and unmatched points in red.
![](https://github.com/weitat95/advisioncw1819/blob/master/images/img2.png)

---
A complete point cloud model after merging 40 frames.
![](https://github.com/weitat95/advisioncw1819/blob/master/images/img3.png)

Left, front and right wall extraction from the room.
![](https://github.com/weitat95/advisioncw1819/blob/master/images/img4.png)

