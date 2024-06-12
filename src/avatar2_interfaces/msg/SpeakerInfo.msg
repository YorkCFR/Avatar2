# Information from the camera
std_msgs/Header header                # lets know when the data came from
uint32 seq                            # sequence number
sensor_msgs/Image  face               # face image
float32 row                           # row in range 0..1 
float32 col                           # col in range 0..1
std_msgs/String info                  # a string holding a json dictionary
