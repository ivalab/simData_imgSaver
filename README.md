# simData_imgSaver
ros package for saving simulation images in Gazebo with Kinect 


### Get the package

```
cd ~/catkin_ws/src/
git clone https://github.gatech.edu/fchu9/simData_imgSaver.git
cd ..
catkin_make
```

### Clone the simData
```
cd ~/catkin_ws/src/
git clone https://github.gatech.edu/fchu9/simData_v2.git 
```
rename the `/3Dwarehouse/tools` to `/warehouse/models` 

### Preprocess data
in `utils/move_model_files.m`
modify the `dataRoot`
```
dataRoot = '/home/YOUR/PATH/TO/catkin_ws/src/simData_v2/warehouse/models';
```
run `move_model_files.m` in MATLAB

### Copy models
copy all models to `/home/YOURNAME/.gazebo/models/`

NOTE:   
you can add `export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/YOUR/PATH/TO/catkin_ws/src/simData_v2/warehouse/` in `~/.bashrc` 
then you can might skip this step    
but change line 34:
```
fprintf(fsdf, ['            <uri>file://models/' FolderName '/mesh/' filename '.dae</uri>\n']);
```

### Open Gazebo
```
gazebo_ros gazebo
```

### Run imgSaver with object
in `visuomotor_grasp_3D_Box.py`    
modify `MODEL_DIR` to `/home/YOUR/PATH/TO/catkin_ws/src/simData_v2/warehouse/models`
```
python visuomotor_grasp_3D_Box.py
```
the files will be saved to `/dataset`

### Run imgSaver with affordance
in `visuomotor_grasp_3D_Box_affordance.py`    
modify `MODEL_DIR` to `/home/YOUR/PATH/TO/catkin_ws/src/simData_v2/warehouse/models`
```
python visuomotor_grasp_3D_Box_affordance.py
```
it loads corresponding affordance .dae files with dark table and highlight the affordance with specific color   
the files will be saved to `/dataset`

### Remove shadow effect
When saving affordance images, the light source causes shadow and affects the color. The light source is not loadable in code (at lease in older version of Gazebo). One quick and effective solution is to add multiple light sources in Gazebo with GUI. Click on the parallel light source and select a location to place, and click on the icon again to turn it on. Also for tall object such as large mug, it is useful to also put several all-directional light source in front of the object.  


