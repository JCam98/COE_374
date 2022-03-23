This directory contains folders of images captured from the Raspberry Pi 
High Quality Camera. These images are used to train the team's RGB and OpenCV
image processing algorithm prototypes that are developed using Python. Each 
folder in the directory contains a batch of images from a particular time
and location. Ultimately, these training images will be read into 
RGB and OpenCV image processing prototypes developed in Python to collect
preliminary results on the effectiveness of these models in identifying
desired features in the targets.  


Iteration_1: 

Date of Capture: 3/21

Location: House Garage with Moderate Lighting

Quantity of Images: 22 

Features of Images: Images of candidate and critical targets as overlay on blue
and gray tarp on light-tan cement garage floor

Image Resolution: Default of 4056 x 3040 (~ 12.33 MPx)

Height Above Ground Level (AGL): ~ 7 ft

Percentage of Target Surface Area (including tarp) Captured: 90 - 98 %

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
RGB and OpenCV models.  

Iteration 2: 

Date of Capture: 3/22

Location: 4th Floor Window of Apartment Unit 

Quantity of Images: 125

Features of Images: 125 total

1) Candidate target overlay on blue tarp (Positive Image 1): 50 count 
2) Critical target overlay on blue tarp (Positive Image 2): 50 count
3) Concrete Pavement (Negative Image for Baseline Detection): 25 count

Image Resolution: Default of 4056 x 3040 (~ 12.33 MPx)

Height Above Ground Level (AGL): ~ 50 ft

Percentage of Target Surface Area (including tarp) Captured: 100 %

Notes: The imaging system apparatus was first set up on a desk adjacent to an apartment bedroom window with the same connections as in the setup for the collection of images in the first iteration (garage).The bedroom window overlooks an area of concrete adjacent to the apartment complex. To begin capturing images, two team members placed the blue tarp on the concrete so that it its edges were approximately normal and parallel to the window, and the candidate target mock-up at the centroid of area. Due to moderate crosswinds in the area, weights were placed on top of each of the corners of the tarp for restraint. Then, one team member started up the Raspberry Pi OS and display while another maintained the connections between the interfaces due to their sensitive operation. Once the environment was setup, the team member running the OS opened a terminal window and ran the following command to dynamically view the image that the camera was seeing: "raspistill -t 0". The team member operating the terminal collaborated with the group member holding the camera to effectively change the focus and aperture to minimize blurry effects from the camera, and optimize the light intensity respectively. Once the camera was positioned such that it was as close as vertically above the centroid of the target as attainable due to restrictions from the window, the following command was used to capture 50 still images of the candidate target mock-up on the blue tarp with maximum resolution (~ 12.3 MPx). Then, the candidate target mock-up was replaced with the critical target mock-up and the same process was repeated. Lastly, both the target mock-ups and the tarp were removed from the field of view, and 25 images of the background concrete (negative images for baseline Object Detection) were captured at the same reolution. 

As in the first iteration of images collected in a house garage, positioning
contraints associated with the imaging equipment made it difficult to capture
images from directly above the centroid of the tarps. Thus, the images depict
the targets and tarp with an angular displacement offset (Azimuth angle) 
from the vertical (vertically above centroid of target). The angular displacement offset
and the small surface area omission of the tarps in the images due to the
constraints posed by the relative positioning of the imaging equipment, and 
height of ceiling respectively shall be taken into account when evaluating
the accuracy of object detection, recognition, and identification in the 
RGB and OpenCV models, however, the variable orientation of the targets captured in the images can be considered a model of real-world conditions where the mounted camera will not be at a fixed orientation relative to the potential targets it is scanning. Another important property of many of the images is the varying light intensity both within and between successive images. In particular, as the sun was setting during the procedure, some images depict a small to moderate percentage of the tarp surface area overlayed by shade contrasted with a deep blue appearance in areas exposed to the sun. This differential in light intensity will likely need to be identified as a parameter by the image recognition procedure. At a high level, the objectives of this training image collection procedure were threefold. First, the team desired to capture images from a height above ground level considerably greater than that of the first collection procedure to obtain high resolution images from an altitude that is closer to the search altitude. Second, the team desired to position the camera and the targets such that the entire geometry of the tarp and targets were captured in the camera's field of view in each image. Thirdly, the team desired to capture a much larger volume of images for each target configuration to train prototype OpenCV and RGB algorithms on, which ideally increases the model's ability to detect, recognize, and identify targets correctly. In other words, the greater the volume of images in the model, the greater the image identification accuracy. 
