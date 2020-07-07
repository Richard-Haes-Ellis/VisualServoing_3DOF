# Visual Servoing with OPENCV
A 3 axis slider powered by stepper motors and the smart ramps 1.6v cnc shield with XYZ populated 
by A4988 (Can be replaced by TMC2130 for quiet motion). 

The controller is a Arduino Due running a SAM3X8E @ 84Mhz. All steppers are controlled by interrupts and two firmware versions are available, one with trapezoidal velocity control and another with serial comunication velocity control, which is ideal to connecto a PC and do some Image processing and control the motors accordingly. 

There is a folder with some python code to track objects and center them automaticly, so one could select a object and have it follow along. 

Here is a video of that working priciple. 


![Alt text](https://github.com/richaeell/MultiAxisSlider3/blob/master/docs/resources/working.gif)

3D rederings of the slider it self. Files are available too under CAD folder.

![Alt text](https://github.com/richaeell/MultiAxisSlider3/blob/master/docs/resources/render.png)

The python tracking algorithm is done by kcf tracker.

[References]

- Some of the code is from iforce2D's video about stepper motors: https://www.youtube.com/watch?v=fHAO7SW-SZI&t=3s
- AVR Guide to stepper motor controll with interrupts: http://ww1.microchip.com/downloads/en/AppNotes/doc8017.pdf
- SAM3X8E Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-11057-32-bit-Cortex-M3-Microcontroller-SAM3X-SAM3A_Datasheet.pdf
- SAM3X8E API (ASF): https://asf.microchip.com/docs/latest/sam3x/html/group__asfdoc__sam__drivers__tc__group.html

- SAM3X8E Timer example: https://asf.microchip.com/docs/latest/sam3x/html/asfdoc_sam_drivers_tc_qsg.html

- SAM3 Timers Application notes from ATMEL: http://ww1.microchip.com/downloads/en/AppNotes/Atmel-42301-SAM3-4S-4L-4E-4N-4CM-4C-G-Timer-Counter-TC-Driver_ApplicationNote_AT07898.pdf

- SAM3 Timers Application notes from Copper Hill Tech: https://copperhilltech.com/content/Application%20Note%20-%20Arduino%20Due%20Timer%20Control.pdf
