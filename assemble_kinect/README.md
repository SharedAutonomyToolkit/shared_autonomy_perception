In order to spoof data from assemble_kinect:

1) Create the log file - need to be running the kinect driver, and assemble_kinect. This will call the assemble_kinect service and save a single response to the given bag file

$ ./save_assembly camera/assemble_kinect:=head_mount_kinect/assemble_kinect -f /home/lil1pal/table_logs/table4.bag -r 5

  * head_mount_kinect is for running on the robot. If using the default openni_launch params, no need to remap
  * -f [filename] specifies where to log to. The directory must already have been created.
  * -r [rate] specifies in Hz how often to log. 


2) Play the log file - this functions in place of the openni_launch & assemble_kinect nodes. It runs a server that responds to KinectAssembly service requests with the single response read in from the bag file. 

$ ./spoof_assembly camera/assemble_kinect:=head_mount_kinect/assemble_kinect -f /home/lil1pal/table_logs/table4.bag -i 0
  * -i [index] indicates which image from the bagfile to use
  
3) In order to determine what each bag file looks like:

$ ./extract_all.sh [dir containing bagfiles]

  * This will open the bag file and use opencv functions to save the every image in it as a pgm.