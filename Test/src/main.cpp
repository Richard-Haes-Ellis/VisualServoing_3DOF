#include <Arduino.h>

// #define debug

union payload {

  int32_t numbers[4];
  uint8_t array[12];

} m_payload;

union integer {

  int32_t number;
  uint8_t array[4];

} m_integer;

enum protocolState
{
  LISTENING, // blink disable
  READ_LOAD, // blink enable
  READ_END,  // we want the led to be on for interval
  END_CMD    // we want the led to be off for interval
};

void setup()
{
  Serial.begin(9600);
  SerialUSB.begin(9600);
  delay(3000);
  SerialUSB.print("\nReady");
}

void loop()
{

  if (Serial.available())
  {
#ifdef debug
    SerialUSB.println("\nIncomming data:");
#endif

    int16_t byten = 11;
    uint8_t state = LISTENING;
    uint8_t new_packet = 0;
    do
    {
      while (Serial.available() == 0)
        ;                              // Wait for incomming data // ADD TIMEOUT GOD DAMN IT
      uint8_t in_byte = Serial.read(); // READ BYTE
#ifdef debug
      SerialUSB.print("State: " + String(state) + " Byte: ");
      SerialUSB.print(in_byte, HEX);
      SerialUSB.println();
#endif
      switch (state)
      {
      case LISTENING:
        if (in_byte == 0x7E) // CHECK IF ITS START BYTE
        {
#ifdef debug
          SerialUSB.println("\n START command read");
#endif

          state = READ_LOAD;
        }
        else
        {
// Wrong start command
#ifdef debug
          SerialUSB.println("\nWrong START BYTE");
#endif

          state = END_CMD; // RESET IF IT ISN'T
        }
        break;

      case READ_LOAD:
        // Serial.readBytes(m_payload.array,12);
        m_payload.array[byten] = in_byte;
        byten--;
        if (byten < 0)
        {
#ifdef debug
          SerialUSB.println("\nRecived 12 bytes");
#endif

          // for(int i=0;i<12;i++)
          //   SerialUSB.print(m_payload.array[i],HEX);
          state = READ_END;
          byten = 3;
        }
        break;

      case READ_END:
        // Serial.readBytes(m_integer.array,4);
        m_integer.array[byten] = in_byte;
        byten--;
        if (byten < 0)
        {
#ifdef debug
          SerialUSB.println("\nRecived end command");
          for (int i = 0; i < 4; i++)
            SerialUSB.print(m_integer.array[i], HEX);
          SerialUSB.println();
          SerialUSB.println(m_integer.number, HEX);
#endif

          if (m_integer.number == (int32_t)0x7FFFFFFF) // END COMMAND
          {
            // Data is correct
            new_packet = 1;
            state = END_CMD;
#ifdef debug
            SerialUSB.println("\nEnd packet");
#endif
          }
          else
          {
#ifdef debug
            SerialUSB.println("\nWrong packet");
#endif

            state = END_CMD;
          }
        }
        break;
      }
    } while (state != END_CMD);

    if (new_packet)
    {
      new_packet = 0;
      SerialUSB.println();
      SerialUSB.print("Recieved M1 = ");
      SerialUSB.print(m_payload.numbers[2]);
      SerialUSB.print(" M2 = ");
      SerialUSB.print(m_payload.numbers[1]);
      SerialUSB.print(" M3 = ");
      SerialUSB.println(m_payload.numbers[0]);
    }
  }
}
