# MultiAxisSlider3

A 3 axis slider powered by stepper motors and the smart ramps 1.6v cnc shield with XYZ populated 
by A4988 (Soon to be replaced by TMC2130). 

The brais is a Arduino Due running a SAM3X8E @ 84Mhz. All steppers are controlled by interrupts with trapezoidal velocity 
profiles with coordination, a lot of it still needs polishing but it works. 












[References]

- Some of the code is from iforce2D's video about stepper motors - https://www.youtube.com/watch?v=fHAO7SW-SZI&t=3s
- AVR Guide to stepper motor controll with interrupts - http://ww1.microchip.com/downloads/en/AppNotes/doc8017.pdf
- SAM3X8E Datasheet - http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-11057-32-bit-Cortex-M3-Microcontroller-SAM3X-SAM3A_Datasheet.pdf

