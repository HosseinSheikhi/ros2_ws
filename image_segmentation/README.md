# Image Segmentation package

## image_segmentation_service


This version subscribes to the overhead images and gives service to the global planner as follows:

1. service receives a request for segmented images from global_planner 
2. feeds the overhead images to the segmentation_networks and returns the results to the global_planner 


## image_segmentation_service_v2
*Note: this version is not complete yet.*
*We have to wait for Future to be complete in send_request function which will lead to a deadlock.*

This version gets overhead images as a client and gives service to the global planner as follows:
1. global_planer_service receives a request for segmented images from global_planner
2. overhead_client sends a request for overhead images to the overhead_cameras_service and after receiving
3. feeds the overhead images to the segmentation_networks and returns the results to the global_planner 
