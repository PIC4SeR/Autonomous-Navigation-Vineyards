> Project: "project name"

> Owner: "project owner name"

> Date: "yyyy:mm"

---

# Title

## Description of the project
Local motion planner for an auotnomous navigation along vineyard rows

## Installation procedure
- ROS
- The installation of the Intel Real Sense Packages is needed. Both the API from Intel and the ROS packages for the camera (wrappers included).
- Tensorflow v1 and other python modules required by the scripts.

## User Guide
The system is subdivided in 2 modules.
- The first one called "deep vineyard" includes the image processing with only Computer Vision, with only the Machine Learning model, or with both. In this last version the ML model is used as a backup solution in case of fault of the computer vision approach (no window respecting the threshold is found).

- The second package is called "jackal commands", and it is simply the script that computes the angular and linear velocity values according to the method use (explain later)

The python code is commented step by step. However, in the following a brief summary for the deep_vineyards package:

- CV approach: it uses the depth map to find the end of the row. That logically it will look like a rectangle. So, using edge detection techniques it finds out all the rectangles, successively it filters out the noise getting rid of the little ones (it keeps the one with the higher area and it checks if it overcomes an empirical threshold). The window/rectangle is found dividing the depth map in near field and far field (common approach in autonomous vechicles navigation systems). To do so, a dynamic threshold is used (x% of the perceived depth). Finally, a proportional controller is implemented. It computes the control values for angular and linear velocity in order to route the mobile platform along the row in the correct path.


- ML approach: a MobileNet-based model is used to perform image classification of the following 3 classes: left, center, right. Therefore, it is able to classify what the mobile platform is pointing at (through the stereocamera). Finally, a control value on angular and linear velocity is given to the motors in order to avoid collisions with the rows.


For more info please check this out: https://www.mdpi.com/2075-1702/8/2/27
