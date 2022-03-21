This directory contains folders of images captured from the Raspberry Pi 
High Quality Camera. These images are used to train the team's RGB and OpenCV
image processing algorithm prototypes that are developed using Python. Each 
folder in the directory contains a batch of images from a particular time
and location. 


Iteration_1: 

Date of Capture: 3/21

Location: House Garage with Moderate Lighting

Quantity: 22 images of candidate and critical targets as overlay on blue
and gray tarp

Image Resolution: Default of 2592 x 1944 (~ 5 MPx)

Notes: Due to height restriction from garage ceiling, many of pictures do
not capture the entire tarp, however, the entire target overlays are captured
in each of the images, and only a small percentage of the surface area of 
the tarp are omitted from each of the images. Additionally, due to positioning
contraints associated with the imaging equipment, it was difficult to capture
images from directly above the centroid of the tarps. Thus, the images depict
the targets and tarp with an angular displacement offset (Azimuth angle) 
from the vertical (directly above target). The angular displacement offset
and the small surface area omission of the tarps in the images due to the
constraints posed by the relative positioning of the imaging equipment, and 
height of ceiling respectively shall be taken into account when evaluating
the accuracy of object detection, recognition, and identification in the 
RGB and OpenCV models. Ultimately, these training images will be read into 
RGB and OpenCV image processing prototypes developed in Python to collect
preliminary results on the effectiveness of these models in identifying
desired features in the targets.   
