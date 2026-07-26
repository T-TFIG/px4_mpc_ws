inside this offboard_control file the main reason we have this file is that we tryin to setting up the connectivity and setup the drone talking passthrough the PX4 first of all we are going to introduce you the 

1. px4_qos_profile() inside this px4 quality of service we setting up the QoSProfile the main goal here is to setting how the connectivity and how the talking method should be 

the reliability = BEST_EFFORT meaning we do not garuntee the delivery
durability = 