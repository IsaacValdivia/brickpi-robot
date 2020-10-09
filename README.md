# Brickpi Robot

<a href="https://ibb.co/Jvn1sfJ"><img src="https://i.ibb.co/Jvn1sfJ/unnamed.png" alt="unnamed" border="0"></a>
<a href="https://ibb.co/kGtT11X"><img src="https://i.ibb.co/kGtT11X/robot-perfil.png" alt="robot-perfil" border="0"></a> 

_Odometry, object tracking (with 2D vision) and dynamic path finding for a BrickPi3 differential drive robot. It's capable of chasing and fetching a red ball, navigating grid maps and recalculating paths to find an exit for them. It uses multiple sensors connected to a Raspberry Pi 3:_ 
* Vertical laser sensor: to keep track of the distance travelled and measuring its movement
* Camera: to detect and chase the red ball. It periodically takes photos, applies a color mask and searches for keypoints in the image using a SimpleBlobDetector from OpenCV. The blob detector filters blobs by area and circularity and returns the most promising keypoint set. Then, the robot rotates and advances accordingly to align the center of the camera with the center of the blob. When the distance between centers becomes negligible, it lowers a basket to capture the ball.  
* Ultrasound sensor: to avoid crashing frontally into walls.

The map to traverse must represent a physical arrangement of walls forming a grid. The robot must be aligned with center of 1 of the cells when starting the travel, and will find the optimal path to the target cell by means of the wavefront algorithm. The robot is able to detect and respond to unplanned walls by updating the map internally and propagating the wave again to find an alternative path.

## Some demos 🎞️

[![Video demo](http://img.youtube.com/vi/r1coLtAZqvQ/0.jpg)](https://www.youtube.com/watch?v=r1coLtAZqvQ "Brickpi Robot Map Travel")
[![Video demo](http://img.youtube.com/vi/HwYpoZOAPf8/0.jpg)](https://www.youtube.com/watch?v=HwYpoZOAPf8 "Brickpi Robot Map Travel")

## Built with 🛠️

* [BrickPi3](https://modbotshop.com/collections/brickpi/products/brickpi)
* Python
* [OpenCV](https://opencv.org/)
* [PiCamera](https://picamera.readthedocs.io/)
* Standard Python libraries e.g.: NumPy, Time, etc.

## Authors ✒️

* **Carla Pascual Real**
* **Pablo Noél Carreras Aguerri**
* **Isaac Valdivia López** - [IsaacValdivia](https://github.com/IsaacValdivia)
