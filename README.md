# MultiAxisSlider3

A 3 axis slider powered by stepper motors and the smart ramps 1.6v cnc shield with XYZ populated 
by A4988 (Can be replaced by TMC2130 for quiet motion). 

The controller is a Arduino Due running a SAM3X8E @ 84Mhz. All steppers are controlled by interrupts and two firmware versions are available, one with trapezoidal velocity control and another with serial comunication velocity control, which is ideal to connecto a PC and do some Image processing and control the motors accordingly. 

There is a folder with some python code to track objects and center them automaticly, so one could select a object and have it follow along. 

Here is a video of that working priciple. 


3D rederings of the slider it self. Files are available too under CAD folder.

![Alt text](https://github.com/richaeell/MultiAxisSlider3/blob/master/render.png)

The python tracking algorithm is done by kcf tracker.

[References]

- Some of the code is from iforce2D's video about stepper motors - https://www.youtube.com/watch?v=fHAO7SW-SZI&t=3s
- AVR Guide to stepper motor controll with interrupts - http://ww1.microchip.com/downloads/en/AppNotes/doc8017.pdf
- SAM3X8E Datasheet - http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-11057-32-bit-Cortex-M3-Microcontroller-SAM3X-SAM3A_Datasheet.pdf

