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
- Prepering the experiment environment
- Testing and adjusting the projector
- Testing and adjusting the endoscope
- Running the data collection node

Details of each step is elaborated in the following section:


