# dvrk_structure_light

package for structure light research and devolopment

# Nodes

The nodes in this package are categorized as test nodes, data collection nodes, decoding nodes, and hybrid nodes

## Test nodes

- `pattern_display_test`

  hase a python and cp version are importan for calibrating the area where the pattern is projected. The python version is more elaborate, up to date and can easily be modified and costumized for development. Modify the path for a test pattern, and modiffy the projection size, location, etc.

- `gray_code_pattern_out_test`

  Tests the computation and projection of graycode patterns on the surgical scene.

## Data Collection Nodes

- `gray_code_pattern_capture`

  Code written based on the tutorial from open cv. <https://docs.opencv.org/4.x/db/d56/tutorial_capture_graycode_pattern.html>

- `stereo_gray_code_pattern`

  For computing the structured light using the stereo cameras in da vinci endoscope

- `single_camera_position#`

  Three nodes for recording the gray patern codes from three camera positions. This is worth cleaning and combining into a single code, which would be my next step.

## Decoding Nodes

- `gray_code_pattern_decode`

  For decoding structured light, computing discrepency image, and 3D geometry in opencv, inspired by : <https://docs.opencv.org/4.x/dc/da9/tutorial_decode_graycode_pattern.html>

- `stereo_camera_decode` & `single_camera_decode`

  Self explainatory, for decoding SL in the stereo and single camera cases


## Hybrid Nodes

- `structured_light_combined`

  Node that combines the captureing and decoding methods as a mean for practical SL computation in a sirgical scene

- `lau_calibration_less`

  A nonfunctional node that tests Lau simple calibration procedure for SL
  <https://www.sciencedirect.com/science/article/pii/S0143816614000116?via%3Dihub>

## Example run

```bash
rosrun dvrk_structured_light <node_name>
```

## data collection script file

This file automates moving the camera and recording the data consists of stereo images and positions of the endoscope

To setup the experiment follow these steps:

- Open two terminals and exprot ros master, connecting with the main control computer: ` export ROS_MASTER_URI=http://129.22.143.140:11311`

- In the first terminal start the camera: `roslaunch hw_camera run_camera.launch`

- In the second terminal reset the endoscope to the first position (to udjust the camera focus and seetings), cd to where you want to collect the data and run the bash script. for example:

```bash
rosrun cwru_dvrk_control go_ecm 0.0 -1 0.00 0 1
cd ~/Data
bash ~/ros_ws/src/dvrk_structure_light/data_collection.bash > data.txt
```

## Experiment setup

The steps to prepare the experiment consists of:
- Prepering the experiment scene
- Testing and adjusting the projector
- Testing and adjusting the endoscope
- Running the data collection node

Details of each step is elaborated in the following section:

### Experiment scene:

The experiment scene includes three main elements: Structured Light Projector Endoscope, camera endoscope, and Target object. the three objects are placed so that the target sets on an elevated ground while the camera and projector's endoscopes face the target. See image below:

![Experiment_Scene](https://github.com/cwru-robotics/dvrk_structure_light/blob/main/doc/experiment_setup.png)

As seen above, the camera endoscope is pointing downwords with the camera tilt facing the target, and the projector endoscope is tilted 30 to 40 degrees. 

### Testing the SL projector

To test the projector, run:

```bash
roslaunch hw_camera run_camera.launch 
rosrun dvrk_structure_light pattern_display_test
```

make sure the projection correctly covers the target area. Check below photo:

![projection_example](https://github.com/cwru-robotics/dvrk_structure_light/blob/main/doc/projection_example.jpg).

### Testing and adjusting the vision endoscope

In the davinci control computer (or through an exported terminal from the vision computer) run:

```bash
rosrun cwru_dvrk_control go_ecm 0.0 -1 0.00 0 1
```

This places the endoscope in a virtical posa to the ground 

Then adjust the target so it can be scene from all poses. For that you can run the data collection script to better test the positionings. 

```bash
bash ~/ros_ws/src/dvrk_structure_light/data_collection.bash > data.txt
```

The first and last camera poses should be placed like the images below. The camera start further from the target then gets closer at the end. 

![first_cam_pose](https://github.com/cwru-robotics/dvrk_structure_light/blob/main/doc/first_cam_pose.jpg).
![last_cam_pose](https://github.com/cwru-robotics/dvrk_structure_light/blob/main/doc/last_cam_pose.jpg).

Examples of the data:

![first_cam_image](https://github.com/cwru-robotics/dvrk_structure_light/blob/main/doc/first_cam_image.jpg).
![last_cam_image](https://github.com/cwru-robotics/dvrk_structure_light/blob/main/doc/last_cam_image.jpg).

You might need to reposition the projector and target based on the above tests. Iterate until the positions work well with the projector and camera endoscopes.


### Run data collection

At this point you can start running the data collection as explained [above](https://github.com/cwru-robotics/dvrk_structure_light?tab=readme-ov-file#data-collection-script-file)


