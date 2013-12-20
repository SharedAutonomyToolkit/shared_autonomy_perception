ROS wrapper around the grabcut_3d library.

### ROS Node API
#### Communication:
* segment_service (shared_autonomy_msgs/Segment.action)
    responds to segmentation requests sent via these actionlib topics
* bbox_service (shared_autonomy_msgs/BoundingBox.action)
    expects this actionlib service to be provided; uses it to obtain bounding box from the user
* pixel_service (shared_autonomy_msgs/EditPixel.action)
    expects this actionlib service to be provided; uses it to obtain segmentation corrections from the user. 

#### Parameters:
* loop_rate - how frequently to check for preemption
* preempt_rate - max time to wait after preempting bbox or pixel service in response to a segmentation preemption, before simply returning
* connect_wait - how long to wait (seconds) for bbox/edit servers to come up before returning segmentaiton failure
* grabcut_iters - passed into grabcut3D. how many times to iterate before returning an estimated segmentation
* min_range - used for converting depth image to [0, 255]. scaled_depth = 255*(depth-min_range)/(max_range-min_range)
* max_range - used for converting depth image to [0, 255]. (range in meters)
* click_radius - how wide of a swath to mark as FG/BG for each point returned by pixel_service
* depth_interpolation_radius - depth image must be smoothed before being passed to grabcut3D; this determines by how much (in pixels)
