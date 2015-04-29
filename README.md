# ros_depthdense_camera
ROS node to acquire RGB and 3D data from SoftKinetic DepthSense cameras

## Note by Myzhar
If you get an exception error of the type:
```
ERROR: no enumerator found, some dll files are missing"                  
```
you must run this command in a terminal:                                 

###64 bit systems:  
```
$ sudo ln -s /lib/x86_64-linux-gnu/libudev.so.1.3.5 /usr/lib/libudev.so.0 
```
###32 bit systems:                               
```
$ sudo ln –s /lib/i386-linux-gnu/libudev.so.1 /usr/lib/libudev.so.0       
```

To be able to access the video streams you need to add your user to the  
video" usergroup entering the following commands:                        
```
$ sudo adduser ubuntu video [replace "ubuntu" with your current username] 
$ sudo chmod g+rw /dev/video*                                             
```

