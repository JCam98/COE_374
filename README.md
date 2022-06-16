# Capstone Design Project: Automatic Target Recognition and Map Generation

## Description

This repository contains the software dependencies for the author's senior capstone design project which involved the design, requirements analysis, testing, and integration of an image processing system for the target recognition and map generation of targets on a designated airfield. 

### Problem Statement

The Austin metropolitan area has experienced large and unpredictable amount of flooding to the degree that the Austin Fire Department (AFD) has received distress calls from individuals stranded on rooftops attempting to seek refuge from the rising waters. In order for the AFD to be more efficient with their search and rescue operations, they need to be able to better determine if, and where, their rescue teams need to be deployed. As such, the AFD has developed a contract for the design and development of an unmanned aircraft system (UAS) that can provide 24/7 search, surveillance, and precise airborne delivery of health-care payloads to aid in these events. 

### Problem Objectives

The contract stipulated that the UAS be equipped with an image processing system with automatic target recognition functionality to autonomously identify the location of, and ultimately distinguish between distressed individuals in need of and not in need of immediate medical attention. Additionally, the contract required that the image processing system be able to stitch together images of the mission area to produce a digital map with overlays containing the GPS locations of the individuals in addition to their distressed classification. Ultimately, this published map would be transmitted directly to the AFD after search and surveillance passes were made through the mission area. Stretch objectives that the design team defined to enhance the robustness of the image processing system’s capabilities included: 

-	An interactive map with a GUI enabling the user to more easily navigate 
-	Implementing features in the target recognition software to enable the identification and classification of human emotions in addition to TOI’s
-	Implementing features in target recognition software to identify targets on variable terrain with other man-made structures and environmental features rather than the base case of an open field
-	Optimizing all algorithms to improve identification accuracy and runtime

To approximate rooftops, the AFD has established the utilization of 2 x 2 meter square tarps colored blue and silver. To resemble individuals in distress, 1 meter diameter circular plywood sheets with one smiley and one frowny face overlay were developed. These products represented non-critical and critical targets of interest (TOI) respectively. The idea behind the two overlays was to distinguish between individuals that are not injured, but stranded on rooftops, and individuals that are in immediate need of medical assistance respectively. 

## Design

The final design selections for the team’s hardware and software subsystems were determined by first documenting designs that met all of the requirements and a majority of the critical criteria set aside by the design team in the Preliminary Design Stage. From here, the designs that met a majority of the critical criteria were downselected into the Feasibility Studies. In the Feasibility Studies, final designs were obtained from the downselection using a linear scoring methodology that ranked criteria from most to least important and assigned corresponding numerical weight values. From here, the designs with the highest scores for each design category were selected as final designs. The hardware subsystem consisted of a Raspberry Pi Model 4B co-processor which was loaded with the target recognition and map generation software and a Raspberry Pi High-Quality-Camera with variable frame capture rate, resolution, and other parameters. Both of these assemblies were connected to the airframe of a small UAS developed by an Aerospace Engineering Manufacturing and Electronics Capstone Design Team. In order for the Raspberry Pi to execute the image recognition and map generation algorithms, it was connected to a Pixhawk system (an onboard flight computer that stored GPS coordinates among other kinematic properties of the vehicle) to receive power. 

The software subsystem (which is fully contained within this repository) involved developing and integrating several python modules together that performed the operations shown in the flowchart in the attached image into a single main python module. This flow was designed based on the ASE team’s mission plan. To enable all features with the exception to the map generation of the software system to be fully autonomous, a bash script that invoked the main python module was developed that was programmed to run shortly after the Raspberry Pi first started receiving power. This main script was capable of controlling the camera and communicating with the Pixhawk using the MavProxy library which implemented the Mavlink communication protocol into Python. 

Before running the target recognition algorithm, the software system connected to the Pixhawk device and starting parameters including flight time, and altitude were pulled from the autopilot and stored. Once the altitude reading exceeded 46 meters, or the elapsed time exceeded 5 minutes, the camera was programmed to start capturing live video of the field for the purpose of storing associated image frames to stitch together for the published map using the `captureMapImages.py` script. These start conditions were used to begin capturing video because it was determined in testing that at the anticipated climbing rate of the UAS during liftoff, running a command to start the camera at an altitude of 46 meters would result in the camera beginning to capture video using that command at approximately the cruise out altitude of 320 ft given a pre-determined delay or lag in communications. Alternatively, an elapsed time of 5 minutes was set as the other start condition because during flight tests, it was determined that there was about a 5-minute delay between the time that the Raspberry Pi was supplied power, and the time that the UAS started liftoff due to a 5 minute pre-flight range test. A predetermined frame capture rate during testing was set to optimize the runtime of the algorithm that stitched together the images to produce a generated map and was approximately 3 frames per second. 

After the images were sliced from the live video, an `undistort_frames.py` script that used the camera’s intrinsic parameters determined during calibration to remove radial and tangential distortion from the camera lens was invoked on each image. From here, a `resize()` function was called that reduced the resolution of the image to a size that was determined to optimize the runtime and accuracy of the image stitching algorithm. From here, the resulting images were written out to a folder on the local file system. Once the lap count over the cruise out waypoints reached 3 indicating that the cruise out was over, the `captureMapImages.py` script was programmed to terminate as the altitude and waypoint path of the UAS would change from 320 ft to 200 ft, and from an area over the perimeter of the field to a more restricted area for the search phase respectively.

From here, the `main.py` module was invoked. A start condition that mirrored that used for initializing the live video for capturing images for the generated map was used to begin capturing live video of the field during the search phase. This main script invoked the function `captureLiveVideoTest()` to connect to the camera and begin recording video. After capturing a frame from the live video, the latitude, longitude, and altitude readings of that frame were pulled from the UAS, and the `undistort_frames.py` and `resize.py` scripts were invoked to remove distortion, and reduce the image size respectively. From here, the `featRecog()` method was invoked to perform automatic target recognition on each frame using the kinematic parameters as correction factors to account for the differences in altitude between the camera and the ground, along with the camera’s extrinsic properties (swath width, field of view) that impacted the raw GPS readings of the field. Automatic Target Recognition part of the software. 

First, an RGB Thresholding approach was used to detect the deep contrast between the blue of the tarp and the green of the ground if a Target of Interest was visible in the frame. If there was a target in view, the thresholding method drew contours around the edges of the blue tarp and filtered out all other unwanted parts of the image. The feature recognition algorithm is then applied to this cropped image to distinguish if the target of interest is a critical target, a frowny face, or a non-critical target, a smiley face or a blank tarp. Specifically, FLANN, a feature detection method, was used to make these decisions. The contoured-cropped frame is passed into both a feature detection method tailored for a smiley face and a frowny face. Based on the number of key-point connections made for each of the two methods, the designation of the target type is made. If the number of key-point connections made between the training image and the test image, or contoured-cropped frame, is less than 4 then the target is classified as a tarp. If the number of connections was greater than four, then the method with the larger number won the designation. 

After this decision was made the Automatic Target recognition algorithm was completed and the software moved on to the calculation of the GPS coordinates of the target using the function `calcGPS()`. In this phase, the pixel location of the identified target, within the original frame from the live video, was transformed into a latitude and longitude coordinate. This was done using the SWATH width of the Raspberry pi camera to determine how many meters are within one pixel of the camera’s view. The distance between the center pixel of the frame and the location of the center of the target was then calculated. This distance in the x and y directions was then converted into meters using the SWATH width and then converted from meters into degrees and tacked onto the latitude and longitude coordinates of the center pixel which are the GPS coordinates pulled from the Pixhawk once the original frame is extracted from the live video. The GPS coordinates were then stored in an array within the code that houses all the GPS coordinates of the identified targets throughout the search phase of the ConOps mission. The identified targets were then grouped with other targets that were within close proximity to each other in a GPS coordinate refinement algorithm contained within the `findClosestTarget.py` module. This list of identified targets was then refined by adding the identification and GPS coordinates into groupings according to their relative latitude and longitude values.

 A threshold during testing was established such that the algorithm checked the distance between each target and the average latitude and longitude coordinates of each grouping of targets, and if the distance of the GPS coordinates is within 11 meters then the current target was grouped with the targets in that group in comparison. This grouping of targets reflects the error-prone nature of the ability of the algorithm to store accurate GPS coordinates according to cushion needed for the calculation of the latitude and longitude from varying camera angles/orientations. This grouping of targets was an iterative process that occurred throughout the search phase of the mission. After the search phase was completed, the software system proceeded to activate the target refinement process in the `refineTargets.py` module which input the array of targets grouped according to their location proximity and removed all groupings where the total number of targets within the group was less than 4. This number was chosen as a threshold based off the results the team acquired during the flight tests that the ATR algorithm was tested on. After the false identifications were removed, the average latitude and longitude for each grouping of targets were evaluated along with the count of each type of identification within that grouping. The target/identification type with the largest count in the group was then stored as the final designation for the refined target/identification in a text file. This effectively marked the end of the refinement process, and the final classifications and GPS coordinates were written out to a text file for the Aerial Drop Payload (ADP) Team's algorithms to use to plan their waypoints for optimized payload delivery during the next phase of the mission. The scripts used to perform these operations are `airdrop_constants.py`, `airdrop_functions.py`, and `run_mission.py`. This effectively marked the end of Image Processing System’s functionality during the flight.

 After landing the plane, the Image Processing Team was responsible for using stored images of the mission area, and associated identifications and GPS coordinates to produce a digital map. The digital map containing the images of the field was developed using an image stitching algorithm contained in `stitching.py`. This algorithm was run manually in a post-processing execution from the command-line of the Raspberry Pi by feeding in images that were stored from the flight. This image stitching algorithm leveraged SIFT feature extraction and FLANN feature matching similar to the behavior of the target recognition algorithm. The algorithm performed these operations on each image in the input image directory and returned an updated stitched map each time a new image was stitched onto the current map. 

After the image stitching was completed, the final step in the software system involved the application of an algorithm that overlayed location pins and text boxes with GPS coordinate identifications on the digital map published from the stitching algorithm. This algorithm was contained within the `gen_published_map.py` module and invoked in `gen_map_main.py` module manually from the command line as a second, and final, post-processing step. This algorithm functioned by first reading in the text files with the GPS coordinates, identification strings, and file path to the stitched map. The algorithm then iterated through each identification in the list, and for each identification, it computed the GPS latitude and longitude transformation factors which rescaled the GPS locations of the identifications to the GPS range of the AOI in the map. From here, the x and y pixel locations of the identifications were approximated by rescaling the GPS coordinates using the image size and the transformation factors. Then, the location pins, and text boxes were overlayed at appropriate pixel locations. Once the algorithm finished iterating through all of the identifications in the text file, the resultant digital map was written out to the file system marking the end of the software system’s functionality. From here, the mission data package containing the Image Processing Team’s results (including all code/scripts used during the mission, TOI images from search phase, text file with identifications and GPS coordinates, mission area map) were uploaded to a flash drive and supplied to our point of contact for scoring. This concluded the ConOps Mission procedure for the Image Processing Team.

## Repository File System

- code  
 - contains OpenFoam files, Python3 / paraview analysis files, and a couple of Bash scripts to run each version of the program. (Like the following...)  
```
├── Source_Files
    └── Development
        └── airdrop_constants.py
        └── airdrop_functions.py
        └── calcGPS.py
        └── gen_map_main.py
        └── captureLiveVideoTest.py
        └── featRecog.py
        └── connection_test.py
        └── findClosestTarget.py
        └── captureImages.py
        └── main.py
        └── gen_published_map.py
        └── png.py
        └── refineTargets.py
        └── resize.py
        └── roll_collection.py
        └── run_mission_parkinglot.py
        └── run_mission.py
        └── stitching.py
        └── startup.py
        └── undistort_frames.py
        └── stitching_old.py
        └── undistort_main.py
        └── images
            └── image_dir
                └── resized10121.jpg
                └── resized10122.jpg
                └── ...
            └── output_dir
                └── 0.JPG
                └── 1.JPG
                └── ...
        └── __pycache__
    └── Test
        └── airdrop_constants.py
        └── airdrop_functions.py
        └── calcGPS.py
        └── gen_map_main.py
        └── captureLiveVideoTest.py
        └── featRecog.py
        └── connection_test.py
        └── findClosestTarget.py
        └── captureImages.py
        └── main.py
        └── gen_published_map.py
        └── png.py
        └── refineTargets.py
        └── resize.py
        └── roll_collection.py
        └── run_mission_parkinglot.py
        └── run_mission.py
        └── stitching.py
        └── startup.py
        └── undistort_frames.py
        └── stitching_old.py
        └── undistort_main.py
        └── __pycache__
    └── Final
        └── airdrop_constants.py
        └── airdrop_functions.py
        └── calcGPS.py
        └── gen_map_main.py
        └── captureLiveVideoTest.py
        └── featRecog.py
        └── connection_test.py
        └── findClosestTarget.py
        └── captureImages.py
        └── main.py
        └── gen_published_map.py
        └── refineTargets.py
        └── resize.py
        └── run_mission.py
        └── stitching.py
        └── startup.py
        └── undistort_frames.py
├── Flight_Test_Data
    └── UAS
        └── April_7th
            └── flight_test_cropped.mp4
            └── flight_test.h264
            └── flight_test.mp4
            └── startup.sh
        └── April_18th
            └── Mission_Conops_Test
                └── 2022-04-18 17-14-02.log
                └── 2022-04-18 17-14-0...745.mat
        └── April_26th
            └── flight_test_Trim.mp4
    └── Quadcopter
    └── Apartment
        └── Collection_of_Training...22.docx
        └── it_1.png
        └── it_2.png
        └── it_3.png
        └── ...
    └── Garage
        └── Building_Mock_up_Targ..._21.docx
        └── image1.png
        └── image2.png
        └── image3.png
        └── ...
├── Ideas_and_Concepts
    └── Concepts_and_Ideas_4-22.docx
    └── Preliminary_Design_Ideas
        └── Preliminary_Design_Information.docx
    └── Feasibility_Studies
        └── Feasibility_Analysis.xlsx
    └── Final_Design_Cuts
        └── Final_Design_Cuts.docx
├── Prototypes
    └── Software
        └── Camera_Calibration
            └── Important Notes_...type.docx
        └── GPS_Coordinate_Calc
            └── Mission_Log_Data
                └── Test_Flights
                    └── April_13th
                        └── 1st_Flight_April_13th.csv
        └── Map_Generation
            └── Procedure for Collecti...ata.docx
        └── Video_Slicing_Software
            └── Python_Libraries
            └── VLC_Media_Player_CLI
                └── flight_18_vel_60_meters_...tion-003
                    └── Image_Frames
                        └── start_time_300s...
                            └── trial_1
                                └── scene00001.png
                                └── scene00002.png
                                └── scene00003.png
                                └── ...
                            └── trial_2
                                └── scene00038.png
                                └── scene00039.png
                                └── scene00040.png
                            └── trial_3
                        └── start_time_516...
                └── VLC_Media_Player_CLI_Vide..ata.docx
        └── Target_Recognition
            └── Thresholding_software_test_runs.docx
├── Deliverables
    └── Project_Description_Lit_Review
        └── Project_Description_and_Literature_Review.pdf
    └── Systems_Overview
        └── Systems_Overview.pdf
    └── Requirements_Definition
        └── Requirements_Definition.pdf
    └── Preliminary_Design
        └── Preliminary_Design_Deliverable.pdf
    └── Project_Proposal
        └── Project_Proposal.pdf
    └── Detailed_Design_Analysis_Plan
        └── Detailed_Design_and_Analysis_Plan.pdf
    └── Preliminary_Results
        └── Preliminary_Results.pdf
    └── Final_Presentation
        └── Capstone_Design_Presentation.pdf
        └── Capstone_Design_Presentation.pptx
    └── Final_Report
        └── FinalReport.pdf
├── Literature_Resources
    └── Academic_Research_Papers
        └── Automatic_Target_Recognition...iew.docx
        └── Improving_UAV-Based...very.pdf
        └── Literature_Summar...tin.docx
        └── Literature_Summar...an.docx
    └── User_Guides
        └── Co-processor_Setup.docx
        └── GitHub
            └── URL_Addresses.docx
        └── Micro_SD_Card
            └── Documentation_for_Micr...eria.docx
        └── Raspberry_Pi
            └── Webpage_URL's.docx
        └── Raspberry_Pi_HQC
            └── hq-camera-getting-started.pdf
            └── Webpage_URL's.docx
        └── Setting_up_Imaging...ent.docx
        └── Technical_Specific...es.docx
        └── Useful_docume_uff.docx
    └── Other
        └── Final_Implemen...allel.docx
        └── Meeting_with_Wilson.docx
        └── Webpage_URL's.docx
├── Project_Documentation
    └── Aircraft_Design_Team_Docs
        └── Subsystems_and_Avionics_Gain.pdf
    └── Contractor_Docs
        └── Aircraft_Design_Standards_Manual
            └── Attachment_3-Aircraft...08.01.pdf
        └── Capability_Develop...current
            └── Attachment_2-ROS_UA...1.16.pdf
        └── Demonstration_Mission...irements
            └── Attachment_4-ROS_UA...1.16.pdf
        └── Request_for_Proposal
            └── ROS_UAS_RFP...1.16.pdf
        └── Statement_of_Work
            └── Attachment_1-ROS_UA...825.pdf
    └── Requirements
        └── Requirements_Ideas
            └── Camera_Requirement_Calculations_Justin.jpg
            └── Camera_Requirement_Calculations_Riley.jpg
            └── Requirements_Definition_Ideas.docx
        └── Preliminary_Requirements...am.docx
        └── Verified_and_Validated_Requirements
            └── Final_Design_Requirements.docx
├── Project_Management
    └── Gantt_Chart
        └── Gantt_Chart.xlsx
    └── Meetings
        └── Joint_Aircraft_Payload_Imaging_Team_Meetings
        └── Meetings_with_Aircraft_Design_Team
        └── Meetings_with_Instructor
        └── Meetings_with_Payload_Design_Team
        └── Meetings_with_Professor_Zwernemann
        └── Team_Meetings
``` 

# Installations & Software Dependencies  

IMPORTANT NOTE: An Ubuntu Linux OS was used to develop and execute source code
on the team's Raspberry Pi 4B co-processor. Thus, all commands presented below
will be specific to the Linux OS.

- `pip` (via `sudo apt-get install python-pip`)
- `python-dev` (via `sudo apt-get install python-dev`)
- `Python 3` (via `sudo apt-get install python3.x`)
- `pymavlink` (via `sudo pip install pymavlink`)
- `dronekit` (via `sudo pip install dronekit`)
- `opencv-python` (via `sudo pip install opencv-python`)

# Contributions

- Project Lead: Justin Campbell; @JCam98
- Technical Co-Lead: Ryan Ylagan; @rylagan229
- Technical Co-Lead: Nicholas Aufiero; @Phart1226
- Communications Co-Lead: Rohan Wariyar
- Communications Co-Lead: Preston Hart; @naufiero




