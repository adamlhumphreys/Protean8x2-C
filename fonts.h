/*
5 x 7 Matrix Font

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

*********************************/

#include <avr/pgmspace.h>

const uint8_t fonts[][17][5] PROGMEM = {
	{
		{0b00111110, //0
		 0b01000001,
		 0b01000001,
		 0b01000001,
		 0b00111110},
		{0b00000000, //1
		 0b01000010,
		 0b01111111,
		 0b01000000,
		 0b00000000},
		{0b01110010, //2
		 0b01001001,
		 0b01001001,
		 0b01001001,
		 0b01000110},
		{0b00100010, //3
		 0b01001001,
		 0b01001001,
		 0b01001001,
		 0b00110110},
		{0b00001111, //4
		 0b00001000,
		 0b00001000,
		 0b00001000,
		 0b01111111},
		{0b01001111, //5
		 0b01001001,
		 0b01001001,
		 0b01001001,
		 0b00110001},
		{0b00111110, //6
		 0b01001001,
		 0b01001001,
		 0b01001001,
		 0b00110000},
		{0b00000001, //7
		 0b00000001,
		 0b01110001,
		 0b00001001,
		 0b00000111},
		{0b00110110, //8
		 0b01001001,
		 0b01001001,
		 0b01001001,
		 0b00110110},
		{0b00000110, //9
		 0b01001001,
		 0b01001001,
		 0b01001001,
		 0b00111110},
    {0b01111111, //10
     0b01111111,
     0b01111111,
     0b01111111,
     0b01111111},
    {0b01010101, //11
     0b01010101,
     0b01010101,
     0b01010101,
     0b01010101},
    {0b00101010, //12
     0b00101010,
     0b00101010,
     0b00101010,
     0b00101010},
    {0b01010101, //13
     0b00101010,
     0b01010101,
     0b00101010,
     0b01010101},
    {0b01111111, //14
     0b00000000,
     0b01111111,
     0b00000000,
     0b01111111},
    {0b00000000, //15
     0b01111111,
     0b00000000,
     0b01111111,
     0b00000000},
    {0b00101010, //16
     0b01010101,
     0b00101010,
     0b01010101,
     0b00101010},
	}
};
