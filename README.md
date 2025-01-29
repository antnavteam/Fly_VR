# Fly_VR
The aim of the project is to observe flies' behavior in a controlled environment. We have 360° degrees screens
which display a 3D world created from the Unity game engine.

![Alt text](images/presentation_1.png)

The fly is fixed on a sapphire, it can turns on itself and flap its wings. A camera takes pictures of the fly, detects its direction, wings. Unity recovers the camera data and integrates it to the behaviour rig.

## Prerequisites
- OpenCV 4.10.0 - [opencv.org/releases](https://opencv.org/releases/)
- Spinnaker SDK - [teledynevisionsolutions.com/products/spinnaker-sdk/](https://www.teledynevisionsolutions.com/products/spinnaker-sdk/?model=Spinnaker%20SDK&vertical=machine%20vision&segment=iis)

To import correctly the project and libraries, follow instruction from the pdf 'Travail JB janvier 2025'

Here is the code :
## C_chill
Code in C++, connect to camera, analyse image, displays image, save resutls as heading frame rate in csv file
### Instruction 
Press a key to mark a step
Press q to stop program
## camera_acquisition
Creation of a DLL from C++ C_chill, connect to camera, analyse image, displays image, save resutls as heading frame rate in csv file, send this results via callback
### Instruction 
Press a key to mark a step
Press q to stop program
## Camera_unity_C_sharp
Proposition of interface between DLL and C#
## Acquisitiond_v143
Executable from the C++ C_chill
## 3 DLL libraries camera_acquisition_
Use the standard one for normal use. You can also find debug and print and no image DLL.
