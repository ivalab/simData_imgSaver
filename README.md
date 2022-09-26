# simData_imgSaver

ros package for saving simulation images in Gazebo with Kinect.

### Get the package

```
cd ~/catkin_ws/src/

# clone this repo
git clone https://github.com/ivalab/simData_imgSaver.git

# make sure to clone dependencies too
git clone https://github.com/ivalab/simData.git
git clone https://github.com/ivaROS/ivaEdy.git

cd ..
catkin_make
```

The [simData](https://github.com/ivalab/simData.git) package will generate the relevant model files and expose them to gazebo.

### Open Gazebo

```
rosrun gazebo_ros gazebo
```

### Run imgSaver with object

```
python visuomotor_grasp_3D_Box.py
```

the files will be saved to `/dataset`

### Run imgSaver with affordance

```
python visuomotor_grasp_3D_Box_affordance.py
```

it loads corresponding affordance .dae files with dark table and highlight the affordance with specific color
the files will be saved to `/dataset`

### Remove shadow effect

When saving affordance images, the light source causes shadow and affects the color. The light source is not loadable in code (at lease in older version of Gazebo). One quick and effective solution is to add multiple light sources in Gazebo with GUI.

<img src="https://github.com/ivalab/simData_imgSaver/blob/master/imgs/lightSource.png" height="30">

Click on the parallel light source and select a location to place, and click on the icon again to turn it on. Also for tall object such as large mug, it is useful to also put several all-directional light source in front of the object.

### Process real data

To generate bonding boxes for UMD dataset, put `simData_imgSaver/utils/boundingBoxGenerator.m` in `YOUR/PATH/TO/UMD_affordance/part-affordance-dataset/` and run the script.

We segment the blue lazy Susan table and binarize the output to get the bounding boxes in HSV space. You can modify the value to adjust to your case.

```
hsv(:,:,3) > INTENSITY_T; % filter out black color
hsv(:,:,2) > 0.33; % filter out white color
```

Real data with bounding box
<img src="https://github.com/ivalab/simData_imgSaver/blob/master/imgs/seg_hammer_01_00000007_rgb.jpg" height="360">
Sim data
<img src="https://github.com/ivalab/simData_imgSaver/blob/master/imgs/hammer_01_7.png" height="360">
Sim data with bounging box
<img src="https://github.com/ivalab/simData_imgSaver/blob/master/imgs/seg_hammer_01_7.png" height="360">
Sim data with grasp affordance
<img src="https://github.com/ivalab/simData_imgSaver/blob/master/imgs/hammer_01_gt_1_7.png" height="360">
Sim data with hit affordance
<img src="https://github.com/ivalab/simData_imgSaver/blob/master/imgs/hammer_01_gt_5_7.png" height="360">

### Citation

If you find it helpful for your research, please consider citing:

    @inproceedings{chu2019learning,
      title = {Learning Affordance Segmentation for Real-world Robotic Manipulation via Synthetic Images},
      author = {F. Chu and R. Xu and P. A. Vela},
      journal = {IEEE Robotics and Automation Letters},
      year = {2019},
      volume = {4},
      number = {2},
      pages = {1140-1147},
      DOI = {10.1109/LRA.2019.2894439},
      ISSN = {2377-3766},
      month = {April}
    }

### Contact

If you encounter any questions, please contact me at fujenchu[at]gatech[dot]edu
