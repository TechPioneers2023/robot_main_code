# Keysight Engineering Challenege 2023

We Team Techpioneers have developed a mobile remote-controlled robot featuring two-wheel differential drive, OpenCV visual processing, TCP-IP protocol and an intuitive control interface. The big picture behind our robot is a network of connected devices, powered by a client-server architecture. A local server is hosted on a Raspberry Pi by running the code from our Remote-Terminal repository, whereas 2 clients are connected to the server: one being the controller client and the other being the robot client. Commands are sent and relayed through the server from controller to robot to execute remote control.

Taking a closer look, on the controller client side, the controller connects to the local server and launches the UI consisting of intuitive buttons through a web browser on a mobile device like a tablet. Presses of these buttons trigger HTTP Post requests which are sent using the TCP/IP protocol to the server. Ther server proceeds to process the requests and extracts the respective command.

On the robot client side, the "brain" of the robot, an Arduino Mega 2560 board, is connected to the server through an UART connection using a physical cable. The server relays its extracted commands to the Arduino board, which is then processed by the Arduino to execute a corresponding action of the robot. The program running on the Arduino is the robot.ino file in this repository. 

In addition to facilitating communication between the controller and robot, the Raspberry Pi also supports the processing of visual footage from a wired camera. Through realising computer vision, the Raspberry Pi checks for obstacles and shapes in the footage and performs distance measurement to improve distance gaauging for the robot controller.

