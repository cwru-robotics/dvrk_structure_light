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