/*
 Example sketch for the PS3 USB library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <PS3USB.h>



USBHost Usb;
/* You can create the instance of the class in two ways */
PS3USB PS3(&Usb); // This will just create the instance
//PS3USB PS3(&Usb,0x00,0x15,0x83,0x3D,0x0A,0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

bool printAngle;
uint8_t state = 0;

void setup()
{
  Serial.begin(115200);
  Serial.print(F("\r\nPS3 USB Library Started"));
  
}
void loop()
{
  Usb.Task();
  //PS3.printStatusString();
  if (PS3.PS3Connected || PS3.PS3NavigationConnected)
  {
    // uint8_t *buffer;
    // buffer = PS3.getBuffer();
    // for(int i=0;i<EP_MAXPKTSIZE;i++)
    // {
    //   Serial.print(buffer[i],HEX);
    // }
    // Serial.println();
    // PS3.setLedToggle(LED1);

    
    if (PS3.getAnalogHat(LeftHatX) > 137 || PS3.getAnalogHat(LeftHatX) < 117 || PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(LeftHatY) < 117 || PS3.getAnalogHat(RightHatX) > 137 || PS3.getAnalogHat(RightHatX) < 117 || PS3.getAnalogHat(RightHatY) > 137 || PS3.getAnalogHat(RightHatY) < 117)
    {
      Serial.print(F("\r\nLeftHatX: "));
      Serial.print(PS3.getAnalogHat(LeftHatX));
      Serial.print(F("\tLeftHatY: "));
      Serial.print(PS3.getAnalogHat(LeftHatY));
      if (PS3.PS3Connected)
      { // The Navigation controller only have one joystick
        Serial.print(F("\tRightHatX: "));
        Serial.print(PS3.getAnalogHat(RightHatX));
        Serial.print(F("\tRightHatY: "));
        Serial.print(PS3.getAnalogHat(RightHatY));
      }
    }
    // Analog button values can be read from almost all buttons
    if (PS3.getAnalogButton(L2) || PS3.getAnalogButton(R2))
    {
      Serial.print(F("\r\nL2: "));
      Serial.print(PS3.getAnalogButton(L2));
      if (!PS3.PS3NavigationConnected)
      {
        Serial.print(F("\tR2: "));
        Serial.print(PS3.getAnalogButton(R2));
      }
    }
    if (PS3.getButtonClick(PS))
      Serial.print(F("\r\nPS"));

    if (PS3.getButtonClick(TRIANGLE))
      Serial.print(F("\r\nTraingle"));
    if (PS3.getButtonClick(CIRCLE))
      Serial.print(F("\r\nCircle"));
    if (PS3.getButtonClick(CROSS))
      Serial.print(F("\r\nCross"));
    if (PS3.getButtonClick(SQUARE))
      Serial.print(F("\r\nSquare"));

    if (PS3.getButtonClick(UP))
    {
      Serial.print(F("\r\nUp"));
      PS3.setLedOff();
      PS3.setLedOn(LED4);
    }
    if (PS3.getButtonClick(RIGHT))
    {
      Serial.print(F("\r\nRight"));
      PS3.setLedOff();
      PS3.setLedOn(LED1);
    }
    if (PS3.getButtonClick(DOWN))
    {
      Serial.print(F("\r\nDown"));
      PS3.setLedOff();
      PS3.setLedOn(LED2);
    }
    if (PS3.getButtonClick(LEFT))
    {
      Serial.print(F("\r\nLeft"));
      PS3.setLedOff();
      PS3.setLedOn(LED3);
    }

    if (PS3.getButtonClick(L1))
      Serial.print(F("\r\nL1"));
    if (PS3.getButtonClick(L3))
      Serial.print(F("\r\nL3"));
    if (PS3.getButtonClick(R1))
      Serial.print(F("\r\nR1"));
    if (PS3.getButtonClick(R3))
      Serial.print(F("\r\nR3"));

    if (PS3.getButtonClick(SELECT))
    {
      Serial.print(F("\r\nSelect - "));
      PS3.printStatusString();
    }
    if (PS3.getButtonClick(START))
    {
      Serial.print(F("\r\nStart"));
      printAngle = !printAngle;
    }
    
  }
  
  delay(500);
}
