/*
Protean 8x2 Serial Protocol (V1.1)

(MIT License)
http://adamlhumphreys.com/ https://www.youtube.com/adamlhumphreys http://adamlhumphreys.deviantart.com https://www.facebook.com/adam.humphreys.3975
Copyright (c) 2016 Adam L. Humphreys

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*********************************

THE PROTOCOL:

How data bytes 2 and 3 correspond to pins: 
BITS:    15 14 13 12 11 10 09 08   07 06 05 04 03 02 01 00
PINS:    D7 D6 C5 C4 C3 C2 C1 C0   D5 D4 B5 B4 B3 B2 B1 B0
Arduino:  7  6 A5 A4 A3 A2 A1 A0    5  4 13 12 11 10  9  8


Output command: <[7:0][7:0][7:0]>
Byte 0: '<'
Byte 1: Address [7:0]
Byte 2: Outputs 15-8 [7:0]
Byte 3: Outputs 7-0 [7:0]
Byte 4: Command [7:6] | '>' [5:0]

Byte 4 Commands [7:6]:
00 - Set ('>')
01 - Pop ('~')
10 - Invert ('¾')
11 - Read ('þ')


Setup commands: ([7:0][7:0][7:0])
Byte 0: '('
Byte 1: Address [7:0]
Byte 2: Bits 15-8 [7:0]; 0 = input and 1 = output when Byte 4, bit 7 is 0; 0 = active high, 1 = pullup/active low when Byte 4, bit 7 is 1;
Byte 3: Bits 7-0 [7:0]
Byte 4: Command [7:6] | ')' [5:0]

Byte 4 Commands [7]:
0 - Configure I/Os
1 - Configure input pullups ('©')
Byte 4 Commands [6]:
0 - Only send input as a one time event/press
1 - Send input as a state change ('i')

Bit 7 and 6 set is 'é'


Input data when transmitted: {[7:0][7:0][7:0]}
Byte 0: '{'
Byte 1: Address [7:0]
Byte 2: Inputs 15-8 [7:0]
Byte 3: Inputs 7-0 [7:0]
Byte 4: '}'
//*/

#include <util/delay.h>

/*** Device Setup ***/

/*-- Auxiliary Functions --*/
#define MATRIX
//#define SELF_TEST

#ifdef MATRIX
  #include "fonts.h"
#endif

/*-- Address and Debounce --*/
uint8_t address = 'A'; // Device address
#define BYTES         5    // Bytes used per transmission
#define IO_BITS       16   // Bits useed for I/O
#define OUT_Countdown 4095 // Number of times an output must loop before being reset.
/*                   65535 ~= 1.83s
 *                   16383 ~= 0.46s
 *                    8191 ~= 0.23s
 *                    4095 ~= 0.115s
 */
#define DEBOUNCE      4000 // Number of times a state must loop before acknowledged as a valid input.
/*                     500 ~= 3.85ms
 *                    1000 ~= 7.7ms
 *                    2000 ~= 15.4ms
 *                    4000 ~= 30.8ms
 *                    6000 ~= 46.2ms
 *                    8000 ~= 61.6ms
 */

#ifdef MATRIX
  #define COLS_IN_CHAR  5 // Font dependent.
  #define ROWS_IN_CHAR (7 +2) // Font dependent. +2 for full column separation.
  //#define COL_TIME    800 // How many loops the coloumn will be displayed before moving to the next one (columnOfChar++) and shifting output to the right.
  #define ROW_TIME     60 // Number of times a dot/pixel must loop before moving onto the next one.
#endif


/*** Serial Setup ***/

#define F_CPU           16000000  // 16 MHz crystal
#define BAUD            2000000   // See Table 20-7 p.193
#if BAUD == 115200
  #define UBRR          (F_CPU / 16 / BAUD)
#elif BAUD <= 1000000
  #define UBRR          (F_CPU / 16 / BAUD) - 1 // For Asynchronous Normal mode (U2X0 = 0) // (F_CPU / (16 * BAUD)) - 1
#else
  #define UBRR          (F_CPU / 8 / BAUD) - 1 // For Asynchronous Double Speed mode (U2X0 = 1) // (F_CPU / (8 * BAUD)) - 1
#endif

//#define serialAvailable ( UCSR0A & _BV(RXC0) )
#define serialTxReady   ( UCSR0A & _BV(UDRE0) ) // See 20.6.3 p.180
volatile uint16_t bytesIn = 0;   // To falg that we have recieved a byte or bytes.
volatile uint16_t byteIndex = 0; // The byte we're receiving in the current transmission
uint8_t           serialData[BYTES]; //[64]; // A place to store the bytes
bool              gotData = false;

#if BAUD >= 14400
  #define BYTE_DELAY() _delay_us( 10 * 1 * 100 / (BAUD / 10000) ) // Allow for 1 byte's time.
#else
  #define BYTE_DELAY() _delay_ms( 10 * 1 * 100 / (BAUD / 10) ) // Allow for 1 byte's time.
#endif

//#define ECHO
#define NEWLINE

#ifdef SELF_TEST
void SendMessage(char []);
#endif


/*** Functions ***/

inline void serialWrite(uint8_t);
inline void finishTx(void);

void Port1(bool, uint8_t); // Whether we want to set input pullups (0), or outputs (1), and the data.
void Port2(bool, uint8_t);
uint8_t DDR1(bool = 0, uint8_t = 0); // Whether we are reading (0), or writing (1) data, and the data to write.
uint8_t DDR2(bool = 0, uint8_t = 0);
uint8_t Pins1(bool = 0); // Whether we are reading inputs (0), or outputs (1).
uint8_t Pins2(bool = 0);

inline void _delayUS(uint8_t);


//void setup()  /*** Setup ***/
int main()  /*** Setup ***/
{
  //init();
  /* Serial Initialization */
  UBRR0H = (unsigned char)(UBRR>>8);
  UBRR0L = (unsigned char)UBRR;
#if BAUD == 2000000
  UCSR0A |= _BV(U2X0); // For Asynchronous Double Speed mode (U2X0 = 1), 2000000 (2 Mbps) baud
#endif
  /* Enable receiver, transmitter, and interrupt */
  UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
  /* Set frame format: 8data, 1 stop bit */
  UCSR0C = (3<<UCSZ00); // | (1<<USBS0) for 2 stop bits // See 20.11.4 p.193

  /* Serial Pins Configuration */
  DDRD |= 1<<1 | 1<<2; // Tx(1) and Tx Enable(2) as Outputs
  PORTD &= ~(1<<2);    // Enable Rx (Disable Tx) [LOW]
  PORTD |= 1<<0;       // Pull Rx(0) Up  **VERY NECESSARY**


#ifdef SELF_TEST

  /* Pin Configuration */
  DDRB &= 0b11000000; //~(1<<0) & ~(1<<1) & ~(1<<2) & ~(1<<3) & ~(1<<4) & ~(1<<5); // Input
  DDRC &= 0b11000000; //~(1<<0) & ~(1<<1) & ~(1<<2) & ~(1<<3) & ~(1<<4) & ~(1<<5); // Input
  DDRD &= 0b00001111; //~(1<<4) & ~(1<<5) & ~(1<<6) & ~(1<<7); // Input

  DDRD |= 1<<3;
  PORTD |= 1<<3; // LED on means good!

  _delay_ms(500);
/*/
  SendMessage("Waiting for \"this1\"...\n");
  sei(); // Enable Interrupts
  while ( !(gotData && serialData[0] == 't' && serialData[1] == 'h' && serialData[2] == 'i' && serialData[3] == 's' && serialData[4] == '1') )
  {
    byteIndex = 0;
    gotData = false;
    _delay_ms(100);
  }
  cli(); // Disable Interrupts
//*/
  /*/if ((PINB & 0b00111111) || (PINC & 0b00111111) || (PIND & 0b11110000)) // Floating could be a problem.
  {
    //PORTD &= ~(1<<3); // LED off means bad!
    SendMessage("We've got power and it shouldn't be here.\n");
  }//*/

  // Pullup everything.
  PORTB |= 0b00111111;
  PORTC |= 0b00111111;
  PORTD |= 0b11110000;
  _delay_us(10);

  if ( ( (PINB & 0b00111111) != 0b00111111 ) || ( (PINC & 0b00111111) != 0b00111111 ) || ( (PIND & 0b11110000) != 0b11110000 ) )
  {
    PORTD &= ~(1<<3); // LED off means bad!
    SendMessage("Ground shouldn't be here.\n");
  }

  for (uint8_t i = 0; i < 6; i++)
  {
    PORTB &= ~(1<<i); // Bring low
    DDRB |= 1<<i;     // Output
    _delay_us(10);
    if ( (PINB & 0b00111111) != (0b00111111 & ~(1<<i) ) || ( (PINC & 0b00111111) != 0b00111111 ) || ( (PIND & 0b11110000) != 0b11110000 ) )
    {
      PORTD &= ~(1<<3); // LED off means bad!
      SendMessage("This low (PORTB cycle) shouldn't be here.\n");
    }
    DDRB &= ~(1<<i);  // Input
    PORTB |= 1<<i;    // Pullup

    PORTC &= ~(1<<i); // Bring low
    DDRC |= 1<<i;     // Output
    _delay_us(10);
    if ( (PINC & 0b00111111) != (0b00111111 & ~(1<<i) ) || ( (PINB & 0b00111111) != 0b00111111 ) || ( (PIND & 0b11110000) != 0b11110000 ) )
    {
      PORTD &= ~(1<<3); // LED off means bad!
      SendMessage("This low (PORTC cycle) shouldn't be here.\n");
    }
    DDRC &= ~(1<<i);  // Input
    PORTC |= 1<<i;    // Pullup
  }

  for (uint8_t i = 4; i < 8; i++)
  {
    PORTD &= ~(1<<i); // Bring low
    DDRD |= 1<<i;     // Output
    _delay_us(10);
    if ( (PIND & 0b11110000) != (0b11110000 & ~(1<<i) ) || ( (PINB & 0b00111111) != 0b00111111 ) || ( (PINC & 0b00111111) != 0b00111111 ) )
    {
      PORTD &= ~(1<<3); // LED off means bad!
      SendMessage("This low (PORTD cycle) shouldn't be here.\n");
    }
    DDRD &= ~(1<<i);  // Input
    PORTD |= 1<<i;    // Pullup
  }

  if (PIND & 1<<3) // If LED is stil on.
  {
    SendMessage("All seems well!\n");
    for (uint8_t i = 0; i < 3; i++)
    {
      PORTD &= ~(1<<3);
      _delay_ms(500);
      PORTD |= 1<<3;
      _delay_ms(500);
    }
  }
  else
    SendMessage("Oh well.\n");

#else // (no) SELF_TEST


  /* Pin Configuration */
  DDRB |= 1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5; // Output
  DDRC |= 1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5; // Output
  DDRD |= 1<<4 | 1<<5 | 1<<6 | 1<<7; // Output

  /* Pullups */
  PORTD |= 1<<3;
  //PORTC |= 1<<5;
  //PORTB |= 255;

  sei(); // Enable Interrupts
//}

  /*** Output "Pop" Management ***/
  uint8_t setOutBits[IO_BITS][2];
  uint16_t setOutBitsCountdown[IO_BITS];
  uint8_t setOutBitsNum = 0;
  bool haveSetOutBits = false;

#ifdef MATRIX
  /*** Matrix Management ***/
  bool     matrixMode   = false;
  uint8_t  currentChar  = 0;           // '0' - 48, this will be used in conjunction with the fonts array.
  uint8_t  currentRow   = 0;           // A row of currentChar in fonts array.
  uint8_t  rowOfChar    = 0;           // Current row position/index of currentChar's sub array.
  uint16_t rowCycleTime = ROW_TIME;    // How many loops the row will be displayed before moving to the next one (rowOfChar++). (See ROW_TIME near top.)
  uint8_t  columnOfChar = 0;           // Current column position/index of currentChar's array.
  //uint16_t columnCycleTime = COL_TIME; // How many loops the coloumn will be displayed before moving to the next one (columnOfChar++),
                                       // and shifting output to the right. (See COL_TIME near top.)
#endif

  /*** Input Management ***/
  bool sendOnStateChange = false;
  uint8_t     prevInBits[2] = {Pins1(), Pins2()};
  uint8_t lastSentInBits[2] = {0, 0};
  uint8_t  currentInBits[2] = {0, 0};
  uint8_t     sendInBits[3] = {0, 0, 0};
  //bool haveInput = false;
  uint16_t debounceTime = 0;
  uint16_t   debounceMS = 0;

//void loop()  /*** Loop ***/
while (1)  /*** Loop ***/
{
  if (bytesIn > 0) // We've received byte(s)! Now reset the bytesIn counter and the byteDelay counter,
  {
    bytesIn = 0;
    BYTE_DELAY();     // Rx interrupts may be occurring, so let’s wait.
    if (bytesIn == 0) // If we haven't received anymore bytes, let's reset the byteIndex. 
      byteIndex = 0;
  }

  /*** Data and Output Pin Handling ***/
  if (gotData && bytesIn == 0) // Shouldn't have to use second condition if only BYTES number of bytes are sent.
  {
    if (serialData[1] == address)
    {
      if (serialData[0] == '<' && (serialData[BYTES - 1] & '>') == '>') // Output Command
      {
        uint8_t command = serialData[BYTES - 1] >> 6;
        if (command == 0) // 00 Set
        {
          Port1(1, serialData[2]);
          Port2(1, serialData[3]);
        }
        else if (command == 1) // 01 Pop
        {
          setOutBits[setOutBitsNum][0] = serialData[2];
          setOutBits[setOutBitsNum][1] = serialData[3];
          setOutBitsCountdown[setOutBitsNum++] = OUT_Countdown;

          Port1(1, Pins1(1) | serialData[2]);
          Port2(1, Pins2(1) | serialData[3]);

          if (setOutBitsNum == IO_BITS)
            setOutBitsNum = 0;
          haveSetOutBits = true;
        }
        else if (command == 2) // 10 Invert
        {
          Port1(1, Pins1(1) ^ serialData[2]);
          Port2(1, Pins2(1) ^ serialData[3]);
        }
        else if (command == 3) // 11 Read
        {
          /** Begin Transmission **/
          PORTD |= 1<<2; // Enable Tx

          serialWrite('{');
          serialWrite(address);
          serialWrite(Pins1(1));
          serialWrite(Pins2(1));
          serialWrite('}');
#ifdef NEWLINE
          serialWrite(10);
#endif

          /** Finish Transmission **/
          finishTx();
        }
      }

#ifdef MATRIX
      else if (serialData[0] == '<' && (serialData[3] & '>') == '>') // Matrix Output Command
      {
        if (DDR1() == 255 && DDR2() == 255 && serialData[2] != 10) // If all outputs are enabled and serialData[2] isn't a newline.
        {
          matrixMode = true;
          currentChar = serialData[2] - 48; //'- 48' because we're reading ASCII, '0' = 48
        }
        else
        {
          matrixMode = false;
          //columnCycleTime = COL_TIME;
          PORTB &= 0b11000000;
          PORTC &= 0b11000000;
          PORTD &= 0b00001111;
        }
      }
#endif

      else if (serialData[0] == '(' && (serialData[BYTES - 1] & ')') == ')') // Setup Command
      {
        if (serialData[BYTES - 1] & 0b01000000) // 1 - Input handling command, bit 6
          sendOnStateChange = true;
        else
          sendOnStateChange = false;

        if (!(serialData[BYTES - 1] & 0b10000000)) // 0 - Set Command, bit 7
        {
          DDR1(1, serialData[2]);
          DDR2(1, serialData[3]);
        }
        else if (serialData[BYTES - 1] & 0b10000000) // 1 - Pullup Command, bit 7
        {
          Port1(0, serialData[2]);
          Port2(0, serialData[3]);
        }

        prevInBits[0] = Pins1();
        prevInBits[0] = Pins2();
      }
    }
#ifdef ECHO
    /** Begin Transmission **/
    PORTD |= 1<<2; // Enable Tx

    for (uint8_t i = 0; i < BYTES; i++)
      serialWrite(serialData[i]);
#ifdef NEWLINE
      serialWrite(10);
#endif

    /** Finish Transmission **/
    finishTx();
#endif

    gotData = false;
  }

#ifdef MATRIX
  /*** Matrix Handling ***/
  if (matrixMode)
  {
    if (rowCycleTime == ROW_TIME) // && rowOfChar <= ROWS_IN_CHAR)
    {
      PORTB &= 0b11000000;
      PORTC &= 0b11000000;
      PORTD &= 0b00001111;
      if (rowOfChar == ROWS_IN_CHAR)
      {
        rowOfChar = 0;
        columnOfChar++;

        if (columnOfChar == COLS_IN_CHAR) //Font dependent.
          columnOfChar = 0;

        currentRow = pgm_read_byte(&(fonts[0][currentChar][columnOfChar]) );
      }

      if (rowOfChar < ROWS_IN_CHAR - 2)
      {
        Port1(1, currentRow & (0b00000001 << rowOfChar) ); // Will be off/0 when rowOfChar is greater than ROWS_IN_CHAR.
        Port2(1, 0b10000000 >> columnOfChar);
      }

      rowOfChar++;
      rowCycleTime = 0;
    }

    rowCycleTime++;
  }
#endif

  /*** Input Pin Handling ***/
  if (sendInBits[3] == 0) // If we don't have pending input to be sent
  {
    currentInBits[0] = Pins1();
    currentInBits[1] = Pins2();
    if (prevInBits[0] != currentInBits[0] || prevInBits[1] != currentInBits[1]) // If input is different
    {
      debounceTime = 0;
      prevInBits[0] = currentInBits[0];
      prevInBits[1] = currentInBits[1];
    }
    else if (debounceTime < DEBOUNCE) // If we've not yet debounced
    {
      debounceTime++;
      if (debounceTime == DEBOUNCE) // If we've debounced
      {
        sendInBits[3] = 1; // Set true, we have pending input
        for (uint8_t i = 0; i < 2; i++)
        {
          if (sendOnStateChange)
            sendInBits[i] = currentInBits[i];
          else
            sendInBits[i] = ~(lastSentInBits[i]) & currentInBits[i];
    
          lastSentInBits[i] = currentInBits[i];
        }
        if (!sendOnStateChange && sendInBits[0] + sendInBits[1] == 0)
          sendInBits[3] = 0; // Set false, we don't have pending input
      }
    }
  }

  if (sendInBits[3] && bytesIn == 0) // If we have pending input and no incomming bytes
  {
    /** Begin Transmission **/
    PORTD |= 1<<2; // Enable Tx

    serialWrite('{');
    serialWrite(address);
    serialWrite(sendInBits[0]);
    serialWrite(sendInBits[1]);
    serialWrite('}');
#ifdef NEWLINE
    serialWrite(10);
#endif

    /** Finish Transmission **/
    finishTx();

    sendInBits[3] = 0; // Set false, we just sent the pending input
  }
  else if (bytesIn > 0)
    _delayUS(address); // Because we don't want two devices trying to send something at once.
    //delayMicroseconds(address);

  /*** Output Pin "Pop" Management ***/
  if (haveSetOutBits)
  {
    bool hadSetOutBits = false;

    for (uint8_t i = 0; i < IO_BITS; i++)
    {
      if (setOutBitsCountdown[i] > 0)
      {
        setOutBitsCountdown[i]--;
        if (setOutBitsCountdown[i] == 0)
        {
          Port1(1, Pins1(1) & (~setOutBits[i][0]));
          Port2(1, Pins2(1) & (~setOutBits[i][1]));
        }
        hadSetOutBits = true;
      }
    }

    haveSetOutBits = hadSetOutBits;
  }
}
#endif // SELF_TEST
return 0;
}


/*** Interrupts ***/

ISR(USART_RX_vect) // RX Complete Interrupt
{
  bytesIn++;
  if (!gotData && byteIndex < BYTES) // If we don't yet have the full BYTES bytes we need, keep reading.
  {
    serialData[byteIndex++] = UDR0;

    if (byteIndex == BYTES)
      gotData = true;
  }
  else
    uint8_t throwAway = UDR0;
}


/*** Functions ***/

inline void serialWrite(uint8_t outByte)
{
  while (serialTxReady == 0); // Wait until ready to write
  UDR0 = outByte;
}

inline void finishTx()
{
  while (serialTxReady == 0); // Wait until Tx is complete.
  //asm("nop"); // Use one or more only with very low error rate.
  BYTE_DELAY();
  BYTE_DELAY(); // Let Tx truly finish. Necessary for small error rates at 115.2k baud, etc.
  //delayMicroseconds(200); // Let Tx truly finish. Necessary for small error rates at 115.2k baud, etc.
  PORTD &= ~(1<<2); // Enable Rx
}

#ifdef SELF_TEST
void SendMessage(char str[])
{
  /** Begin Transmission **/
  PORTD |= 1<<2; // Enable Tx

  for (uint8_t i = 0; str[i] != '\n' && str[i] != 0; i++)
  {
    serialWrite(str[i]);
  }
#ifdef NEWLINE
  serialWrite(10);
#endif

  /** Finish Transmission **/
  finishTx();
}
#endif

void Port1(bool IO, uint8_t data)
{
  if (IO == 1) // Output
  {
    PORTC = (PORTC & (0b11000000 | ~DDRC)) | (data & 0b00111111);
    PORTD = (PORTD & (0b00111111 | ~DDRD)) | (data & 0b11000000);
  }
  else // Input Pullups
  {
    PORTC = (PORTC & (0b11000000 | DDRC)) | (data & 0b00111111);
    PORTD = (PORTD & (0b00111111 | DDRD)) | (data & 0b11000000);
  }
}
void Port2(bool IO, uint8_t data)
{
  if (IO == 1) // Output
  {
    PORTB = (PORTB & (0b11000000 | ~DDRB)) | (data & 0b00111111);
    PORTD = (PORTD & (0b11001111 | ~DDRD)) | ((data>>2) & 0b00110000);
  }
  else // Input Pullups
  {
    PORTB = (PORTB & (0b11000000 | DDRB)) | (data & 0b00111111);
    PORTD = (PORTD & (0b11001111 | DDRD)) | ((data>>2) & 0b00110000);
  }
}
uint8_t DDR1(bool write, uint8_t data)
{
  if (write)
  {
    DDRC = (DDRC & 0b11000000) | (data & 0b00111111);
    DDRD = (DDRD & 0b00111111) | (data & 0b11000000);
  }
  else
    data = (DDRC & 0b00111111) | (DDRD & 0b11000000);
  return data;
}
uint8_t DDR2(bool write, uint8_t data)
{
  if (write)
  {
    DDRB = (DDRB & 0b11000000) | (data & 0b00111111);
    DDRD = (DDRD & 0b11001111) | ((data>>2) & 0b00110000);
  }
  else
    data = (DDRB & 0b00111111) | (DDRD & 0b00110000)<<2;
  return data;
}
uint8_t Pins1(bool IO) // This is where we manage whether inputs are pulled up/active low or active high.
{
  uint8_t data;
  if (IO == 0) // Input states
    data = ( (PINC ^ PORTC) & 0b00111111 & ~DDRC) | ( (PIND ^ PORTD) & 0b11000000 & ~DDRD);
  else // Output states
    data = (PINC & 0b00111111 & DDRC) | (PIND & 0b11000000 & DDRD);
  return data;
}
uint8_t Pins2(bool IO) // This is where we manage whether inputs are pulled up/active low or active high.
{
  uint8_t data;
  if (IO == 0) // Input states
    data = ( (PINB ^ PORTB) & 0b00111111 & ~DDRB) | ( (PIND ^ PORTD) & 0b00110000 & ~DDRD)<<2;
  else // Output states
    data = (PINB & 0b00111111 & DDRB) | (PIND & 0b00110000 & DDRD)<<2;
  return data;
}

inline void _delayUS(uint8_t i)
{
  while (i > 0)
  {
    _delay_us(1);
    i--;
  }
}

