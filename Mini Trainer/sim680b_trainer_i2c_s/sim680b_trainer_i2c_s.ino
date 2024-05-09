//    Motorla M6800 Trainer Simulator for the Raspberry Pi Pico
//    Modified for 128x32 OLED display. Emulates 7-segment LED display
//    Copyright (C) 2022 and 2024  Simple-Circuit
//
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

// RAM memory 0x0000 to 0xDFFF
// ROM memory 0xE000 to 0xFFFF
//
// UART Status 0xF000 Read Only, Bit-1 Transmit Ready, Bit-0 Receive Ready
//      Use: LDAA $F000 or LDAB $F000
// UART Write Data 0xF001
//      Use: STAA $F001 or STAB $F001
// UART Read Data 0xF003
//      Use: LDAA $F003, LDAB $F003, or ANDB $F003
// I/O Data In at 0xF011 Digital pins D20 to D27
//     Use: Any load instruction, maps to memory
// I/O Data Out at 0xF010 Digital pins D14 = b6 and D15 = b8
//     Use: STAB $F010, STAA $F010, STAA 0,X where X=$F010
// LED On=0x01 Off=0x00 at memory location 0xC108
//     Write to memory with any instruction
//     1ms delay to respond
// ADC Start Read at 0xF020
//     Use: LDAB #ch then STAB $F020 where ch = 1 to 3
//     Use: LDAA #ch then STAA 0,X where X=$F020
// ADC High Byte at 0xF020, ADC Low Byte at 0xF021
//     Use: Any load instruction, maps to memory 
//
// Key Pad updated every 3ms:
//      0xC000 reads 0xFF for no key down else 0x00 to 0x0F or 0x20 or 0x10 for pressed key
//      0xC006 returns key column-1, low bits indicate  b5-0 b4-1 b3-4 b2-7 b1-A b0-D
//      0xC005 returns key column-2, low bits indicate  b5-grey b4-2 b3-5 b2-8 b1-B b0-E
//      0xC003 returns key column-3, low bits indicate  b5-red b4-3 b3-6 b2-9 b1-C b0-F
//
// Seven Segment Display updated every 1ms:
//
//      ASCII Data  0xC100-0xC107  Left to Right Digits. Any value other than 0x00 writes over segment memory
//                                 OR 0x80 with ASCII value to set decimal point on that digit
//
//      0xC109      0x80 -> 7-Seg Emulator Only, 0x01 -> ASCII Character Display, 0x00 -> Both
//      0xC10A      Clear 7-Seg Display Data 0xC110 to 0xC187
//
//      Right Digit 0xC110-0xC117  segments: g f e d c b a dp
//        .   Digit 0xC120-0xC127  segments: g f e d c b a dp
//        .   Digit 0xC130-0xC137  segments: g f e d c b a dp
//        .   Digit 0xC140-0xC147  segments: g f e d c b a dp
//        .   Digit 0xC150-0xC157  segments: g f e d c b a dp
//        .   Digit 0xC160-0xC167  segments: g f e d c b a dp
//        .   Digit 0xC170-0xC177  segments: g f e d c b a dp
//      Left  Digit 0xC180-0xC187  segments: g f e d c b a dp

//   ******NEW****** ADDED OP CODES for the 6800 simulator  
//      
// TRACON: Op code $12
//     Use: Turns on instruction trace (register dump)
// TRACOF: op code $13
//     Use: Turns off instruction trace
// SS2: op code $02
//     Use: call from system to run one opcode for user
//          pushes registers on SP
//          pulls registers from SP2
//          executes opcode
//          pushes registers on SP2
//          pulls registers from SP
// T2S: op code $03
//     Use: Transfer Index register to SP2
// T2S: op code $03
//     Use: Transfer Index register to SP2
// T2X: op code $04
//     Use: Transfer SP2 to Index register
// RS2: op code $05
//     Use: run from SP2
//          pushes registers on SP
//          pulls registers from SP2
// RS1: op code $15
//     Use: run from SP (use instead of RTI to return to system monitor)
//          pushes registers on SP2
//          pulls registers from SP1

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 32 
#define OLED_RESET     -1 
#define SCREEN_ADDRESS 0x3C 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NMI 18        //Falling Edge on D18 triggers non-maskable interrupt
#define IRQ 19        //Low on D19 triggers maskable interrupt 
#define RESET 16      //Low on D16 causes processor reset
#define TR_EN 17      //Low on D17 will print trace of instructions at 5Hz
#define LOOP_P 21     //Output on D21 shows length of instruction cycle
#define BOOTF 20      //Tie low for Forth, else Basic loads
#define ROM_START 0xE000 //End of RAM area

char text[32];
static const char *opcode[] = {
"-----", "NOP  ", "-----", "-----","-----", "-----", "TAP  ", "TPA  ",
"INX  ", "DEX  ", "CLV  ", "SEV  ","CLC  ", "SEC  ", "CLI  ", "SEI  ",
"SBA  ", "CBA  ", "TRCON", "TRCOF","-----", "-----", "TAB  ", "TBA  ",
"-----", "DAA  ", "-----", "ABA  ","-----", "-----", "-----", "-----",
"BRA  ", "-----", "BHI  ", "BLS  ","BCC  ", "BCS  ", "BNE  ", "BEQ  ",
"BVC  ", "BVS  ", "BPL  ", "BMI  ","BGE  ", "BLT  ", "BGT  ", "BLE  ",
"TSX  ", "INS  ", "PULA ", "PULB ","DES  ", "TXS  ", "PSHA ", "PSHB ",
"-----", "RTS  ", "-----", "RTI  ","-----", "-----", "WAI  ", "SWI  ",
"NEGA ", "-----", "-----", "COMA ","LSRA ", "-----", "RORA ", "ASRA ",
"ASLA ", "ROLA ", "DECA ", "-----","INCA ", "TSTA ", "-----", "CLRA ",
"NEGB ", "-----", "-----", "COMB ","LSRB ", "-----", "RORB ", "ASRB ",
"ASLB ", "ROLB ", "DECB ", "-----","INCB ", "TSTB ", "-----", "CLRB ",
"NEG X", "-----", "-----", "COM X","LSR X", "-----", "ROR X", "ASR X",
"ASL X", "ROL X", "DEC X", "-----","INC X", "TST X", "JMP X", "CLR X",
"NEG E", "-----", "-----", "COM E","LSR E", "-----", "ROR E", "ASR E",
"ASL E", "ROL E", "DEC E", "-----","INC E", "TST E", "JMP E", "CLR E",
"SUBAi", "CMPAi", "SBCAi", "-----","ANDAi", "BITAi", "LDAAi", "-----",
"EORAi", "ADCAi", "ORAAi", "ADDAi","CPX i", "BSR  ", "LDS i", "-----",
"SUBAd", "CMPAd", "SBCAd", "-----","ANDAd", "BITAd", "LDAAd", "STAAd",
"EORAd", "ADCAd", "ORAAd", "ADDAd","CPX d", "-----", "LDS d", "STS d",
"SUBAx", "CMPAx", "SBCAx", "-----","ANDAx", "BITAx", "LDAAx", "STAAx",
"EORAx", "ADCAx", "ORAAx", "ADDAx","CPX x", "JSR x", "LDS x", "STS x",
"SUBAe", "CMPAe", "SBCAe", "-----","ANDAe", "BITAe", "LDAAe", "STAAe",
"EORAe", "ADCAe", "ORAAe", "ADDAe","CPX e", "JSR e", "LDS e", "STS e",
"SUBBi", "CMPBi", "SBCBi", "RAC i","ANDBi", "BITBi", "LDABi", "-----",
"EORBi", "ADCBi", "ORABi", "ADDBi","-----", "-----", "LDX i", "-----",
"SUBBd", "CMPBd", "SBCBd", "-----","ANDBd", "BITBd", "LDABd", "STABd",
"EORBd", "ADCBd", "ORABd", "ADDBd","-----", "-----", "LDX d", "STX d",
"SUBBx", "CMPBx", "SBCBx", "-----","ANDBx", "BITBx", "LDABx", "STABx",
"EORBx", "ADCBx", "ORABx", "ADDBx","-----", "-----", "LDX x", "STX x",
"SUBBe", "CMPBe", "SBCBe", "-----","ANDBe", "BITBe", "LDABe", "STABe",
"EORBe", "ADCBe", "ORABe", "ADDBe","-----", "-----", "LDX e", "STX e",};


//Serial Monitor ROM code is loaded when input D20 is logic high
static uint16_t PICO_BUG[1133] = {0x8E,0x00,0xF1,0x9F,0xF6,0xCE,0x01,0x00,0xDF,0xFA,0x5F,0xD7,0xF3,0xD7,0xF2,0x7E,0xFD,0xF9,0x9F,0xF6,0x30,0xBD,0xFC,0x5F,0xBD,0xFC,0x5B,0xC6,0x70,0xBD,
0xFC,0x65,0xC6,0x63,0xBD,0xFC,0x65,0xA6,0x05,0xBD,0xFE,0x8E,0xA6,0x06,0xBD,0xFE,0x8E,0xBD,0xFC,0x63,0xC6,0x62,0xBD,0xFC,0x65,0xA6,0x01,0xBD,0xFE,0x8E,
0xBD,0xFC,0x63,0xC6,0x61,0xBD,0xFC,0x65,0xA6,0x02,0xBD,0xFE,0x8E,0xBD,0xFC,0x63,0xC6,0x78,0xBD,0xFC,0x65,0xA6,0x03,0xBD,0xFE,0x8E,0xA6,0x04,0xBD,0xFE,
0x8E,0xBD,0xFC,0x63,0xA6,0x00,0x85,0x20,0x27,0x04,0xC6,0x48,0x20,0x02,0xC6,0x2E,0xBD,0xFC,0x65,0x85,0x10,0x27,0x04,0xC6,0x49,0x20,0x02,0xC6,0x2E,0xBD,
0xFC,0x65,0x85,0x08,0x27,0x04,0xC6,0x4E,0x20,0x02,0xC6,0x2E,0xBD,0xFC,0x65,0x85,0x04,0x27,0x04,0xC6,0x5A,0x20,0x02,0xC6,0x2E,0xBD,0xFC,0x65,0x85,0x02,
0x27,0x04,0xC6,0x56,0x20,0xF5,0xC6,0x2E,0xBD,0xFC,0x65,0x85,0x01,0x27,0x04,0xC6,0x43,0x20,0x02,0xC6,0x2E,0xBD,0xFC,0x65,0xBD,0xFC,0x63,0xC6,0x73,0xBD,
0xFC,0x65,0xC6,0x70,0xBD,0xFC,0x65,0x96,0xF6,0xBD,0xFE,0x8E,0x96,0xF7,0xBD,0xFE,0x8E,0x7E,0xFD,0xF9,0xC6,0x0A,0x20,0x06,0xC6,0x0D,0x20,0x02,0xC6,0x20,
0x37,0xF6,0xF0,0x00,0x57,0x57,0x24,0xF9,0x33,0xF7,0xF0,0x01,0x39,0xF6,0xF0,0x00,0x56,0x24,0xFA,0xC6,0x7F,0xF4,0xF0,0x03,0x7D,0x00,0xF3,0x2A,0x01,0x39,
0x7E,0xFC,0x65,0x8D,0xEA,0xC1,0x30,0x2D,0x0C,0xC1,0x39,0x2F,0x0B,0xC1,0x41,0x2D,0x04,0xC1,0x46,0x2F,0x06,0x7E,0xFD,0xF9,0xC0,0x30,0x39,0xC0,0x37,0x39,
0xBD,0xFE,0x83,0xDF,0xFD,0xBD,0xFC,0x63,0xBD,0xFE,0x83,0xDF,0xFA,0xBD,0xFC,0x5F,0xBD,0xFC,0x5B,0xDE,0xFD,0xDF,0xFD,0xDE,0xFD,0x9C,0xFA,0x22,0x64,0x96,
0xFB,0x90,0xFE,0xD6,0xFA,0xD2,0xFD,0x26,0x04,0x81,0x0F,0x25,0x02,0x86,0x0F,0x97,0xFF,0xC6,0x53,0xBD,0xFC,0x65,0xC6,0x31,0xBD,0xFC,0x65,0x96,0xFF,0x8B,
0x04,0x97,0xF8,0xBD,0xFE,0x8E,0x96,0xFD,0xD6,0xF8,0xDB,0xFD,0xD7,0xF8,0xBD,0xFE,0x8E,0x96,0xFE,0xD6,0xF8,0xDB,0xFE,0xD7,0xF8,0xBD,0xFE,0x8E,0xDE,0xFD,
0xA6,0x00,0x16,0xDB,0xF8,0xD7,0xF8,0xBD,0xFE,0x8E,0x08,0xDF,0xFD,0x7A,0x00,0xFF,0x2A,0xEC,0x96,0xF8,0x43,0xBD,0xFE,0x8E,0xBD,0xFC,0x5F,0xBD,0xFC,0x5B,
0xCE,0x04,0xE2,0x09,0x26,0xFD,0x7E,0xFC,0xB8,0xC6,0x53,0xBD,0xFC,0x65,0xC6,0x39,0xBD,0xFC,0x65,0xBD,0xFC,0x5F,0xBD,0xFC,0x5B,0x7E,0xFD,0xF9,0x86,0xFF,
0x97,0xF8,0x20,0x03,0x7F,0x00,0xF8,0xBD,0xFE,0x83,0xDF,0xFD,0xBD,0xFC,0x63,0xBD,0xFE,0x83,0xDF,0xFA,0xCE,0x04,0xE2,0x09,0x26,0xFD,0xBD,0xFC,0x5F,0xBD,
0xFC,0x5B,0x7D,0x00,0xF8,0x2B,0x0D,0x96,0xFD,0xBD,0xFE,0x8E,0x96,0xFE,0xBD,0xFE,0x8E,0xBD,0xFC,0x63,0x86,0x10,0x97,0xFF,0xDE,0xFD,0x7D,0x00,0xF8,0x27,
0x0A,0xC6,0x30,0xBD,0xFC,0x65,0xC6,0x78,0xBD,0xFC,0x65,0xA6,0x00,0xBD,0xFE,0x8E,0x7D,0x00,0xF8,0x27,0x05,0xC6,0x2C,0xBD,0xFC,0x65,0x08,0xDF,0xFD,0x9C,
0xFA,0x22,0x07,0x7A,0x00,0xFF,0x26,0xD6,0x20,0xB0,0x09,0xDF,0xFA,0xBD,0xFC,0x5F,0xBD,0xFC,0x5B,0x7E,0xFD,0xF9,0xDE,0xFA,0x08,0xDF,0xFA,0x96,0xFA,0xBD,
0xFE,0x8E,0x96,0xFB,0xBD,0xFE,0x8E,0xBD,0xFC,0x63,0xA6,0x00,0xBD,0xFE,0x8E,0xBD,0xFC,0x63,0xBD,0xFE,0x74,0xE7,0x00,0x7E,0xFD,0xF9,0xBD,0xFE,0x83,0xDF,
0xFA,0xBD,0xFC,0x63,0xA6,0x00,0xBD,0xFE,0x8E,0xBD,0xFC,0x63,0xBD,0xFE,0x74,0xE7,0x00,0xBD,0xFC,0x5F,0xBD,0xFC,0x5B,0xDE,0xFA,0x08,0xDF,0xFA,0x96,0xFA,
0xBD,0xFE,0x8E,0x96,0xFB,0xBD,0xFE,0x8E,0xBD,0xFC,0x63,0x7E,0xFD,0xCE,0x9E,0xF6,0xBD,0xFC,0x5F,0xBD,0xFC,0x5B,0xC6,0x3E,0xBD,0xFC,0x65,0xBD,0xFC,0x72,
0x17,0xF6,0xF0,0x00,0x56,0x56,0x24,0xF9,0xC6,0x20,0xF7,0xF0,0x01,0x81,0x42,0x26,0x03,0x7E,0xFE,0x5D,0x81,0x44,0x26,0x03,0x7E,0xFD,0x3B,0x81,0x4C,0x26,
0x03,0x7E,0xFE,0xBB,0x81,0x4A,0x26,0x03,0x7E,0xFE,0x58,0x81,0x4D,0x26,0x03,0x7E,0xFD,0xC9,0x81,0x4E,0x26,0x03,0x7E,0xFD,0xA7,0x81,0x50,0x26,0x03,0x7E,
0xFC,0xA1,0x81,0x58,0x26,0x03,0x7E,0xFD,0x35,0x81,0x52,0x26,0x03,0x7E,0xFE,0x6D,0x7E,0xFF,0xAB,0xBD,0xFE,0x83,0x6E,0x00,0xBD,0xFE,0x83,0xDF,0xF4,0xA6,
0x00,0x97,0xF9,0x86,0x3F,0xA7,0x00,0x7E,0xFD,0xF9,0xDE,0xF4,0x96,0xF9,0xA7,0x00,0x3B,0xBD,0xFC,0x86,0x58,0x58,0x58,0x58,0xD7,0xFC,0xBD,0xFC,0x86,0xDA,
0xFC,0x39,0x8D,0xEF,0xD7,0xFA,0x8D,0xEB,0xD7,0xFB,0xDE,0xFA,0x39,0xF6,0xF0,0x00,0x56,0x56,0x24,0xF9,0x16,0x54,0x54,0x54,0x54,0xCA,0x30,0xC1,0x3A,0x2D,
0x02,0xCB,0x07,0xF7,0xF0,0x01,0xF6,0xF0,0x00,0x56,0x56,0x24,0xF9,0x16,0xC4,0x0F,0xCA,0x30,0xC1,0x3A,0x2D,0x02,0xCB,0x07,0xF7,0xF0,0x01,0x39,0xBD,0xFC,
0x72,0xC1,0x53,0x26,0xF9,0xBD,0xFC,0x72,0xC1,0x39,0x26,0x03,0x7E,0xFD,0xF9,0xC1,0x31,0x26,0xEB,0x4F,0xBD,0xFE,0x74,0x1B,0xC0,0x02,0xD7,0xFF,0xBD,0xFE,
0x83,0x9B,0xFA,0x9B,0xFB,0xDE,0xFA,0xBD,0xFE,0x74,0x1B,0x7A,0x00,0xFF,0x27,0x0F,0x8C,0x00,0xEE,0x25,0x05,0x8C,0x00,0xFF,0x23,0x02,0xE7,0x00,0x08,0x20,
0xE8,0x4C,0x27,0xBE,0x7E,0xFD,0xF9,
0x7E,0xFC,0x72,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x7E,0xFC,0x86,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0xF6,0xF0,0x00,0x56,0x39,
0x7E,0xFE,0xBB,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x7E,0xFE,0x74,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x7E,0xFE,0x83,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x7E,0xFE,0x8E,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x8C,0xC6,0x20,0x37,0xF6,0xF0,0x00,0x56,0x56,0x24,0xF9,0x33,0xC4,0x7F,0xF7,0xF0,0x01,0x39,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x7E,0xFD,0xF9,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x7E,0xFB,0x93,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x7E,0xFB,0xA5,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x01,0x00,0xFB,0xA5,0x01,0x04,0xFB,0x93};

//Memory used for decimal adjust lookup
static uint16_t DAA[1024];

//trainer monitor (push buttons and seven segment I/O) ROM code loaded when D20 = logic low
static uint16_t TRAINER[0x0490] = {0x81,0x0D,0x26,0x04,0x7F,0x00,0xED,0x39,0x81,0x0A,0x26,0x1B,0x86,0x20,0xB7,0xC1,0x00,0xB7,0xC1,0x01,0xB7,0xC1,0x02,0xB7,0xC1,0x03,0xB7,0xC1,0x04,0xB7,
0xC1,0x05,0xB7,0xC1,0x06,0xB7,0xC1,0x07,0x39,0x36,0x96,0xED,0x2A,0x02,0x32,0x39,0x4A,0x2B,0x17,0x4A,0x2B,0x1A,0x4A,0x2B,0x1D,0x4A,0x2B,0x20,0x4A,0x2B,
0x23,0x4A,0x2B,0x26,0x4A,0x2B,0x29,0x4A,0x2B,0x2C,0x32,0x39,0x32,0xB7,0xC1,0x00,0x20,0x28,0x32,0xB7,0xC1,0x01,0x20,0x22,0x32,0xB7,0xC1,0x02,0x20,0x1C,
0x32,0xB7,0xC1,0x03,0x20,0x16,0x32,0xB7,0xC1,0x04,0x20,0x10,0x32,0xB7,0xC1,0x05,0x20,0x0A,0x32,0xB7,0xC1,0x06,0x20,0x04,0x32,0xB7,0xC1,0x07,0x7C,0x00,
0xED,0x39,0x36,0x44,0x44,0x44,0x44,0xBD,0xFC,0xDA,0xBD,0xFB,0x70,0x32,0x84,0x0F,0xBD,0xFC,0xDA,0xBD,0xFB,0x70,0x39,0x01,
0x0F,0x8E,0x00,0xE0,0xCE,0x00,0xC0,0x04,0x86,0x10,0xA7,0x01,0x4F,0xA7,0x02,0xA7,0x03,0xA7,0x04,0xA7,0x05,0xA7,0x06,0xA7,0x07,0x86,0xFF,0x97,0xEA,0x97,
0xEB,0xCE,0xFC,0x82,0xBD,0xFC,0x8A,0xBD,0xFC,0xC3,0xCE,0xFD,0x62,0x81,0x01,0x27,0x4F,0xCE,0xFD,0x82,0x81,0x02,0x27,0x48,0xCE,0xFD,0xA2,0x81,0x05,0x27,
0x41,0xCE,0xFE,0x54,0x81,0x04,0x27,0x3A,0xCE,0xFE,0x7C,0x81,0x03,0x27,0x33,0xCE,0xFE,0xE6,0x81,0x0E,0x27,0x2C,0xCE,0xFF,0x2B,0x81,0x0A,0x27,0x25,0xCE,
0xFF,0x59,0x81,0x0D,0x27,0x1E,0xCE,0xFF,0x75,0x81,0x06,0x27,0x17,0xCE,0xFF,0x8D,0x81,0x08,0x27,0x10,0xCE,0xFF,0xA3,0x81,0x09,0x27,0x09,0xCE,0xFF,0xC3,
0x81,0x07,0x27,0x02,0x20,0xA1,0xAD,0x00,0x20,0x9D,0x52,0x45,0x41,0x44,0x59,0x20,0x20,0x20,0xA6,0x00,0x27,0x34,0xB7,0xC1,0x00,0xA6,0x01,0x27,0x2D,0xB7,
0xC1,0x01,0xA6,0x02,0x27,0x26,0xB7,0xC1,0x02,0xA6,0x03,0x27,0x1F,0xB7,0xC1,0x03,0xA6,0x04,0x27,0x18,0xB7,0xC1,0x04,0xA6,0x05,0x27,0x11,0xB7,0xC1,0x05,
0xA6,0x06,0x27,0x0A,0xB7,0xC1,0x06,0xA6,0x07,0x27,0x03,0xB7,0xC1,0x07,0x39,0xB6,0xC0,0x00,0x2B,0xFB,0xF6,0xC0,0x00,0xC1,0x10,0x26,0x01,0x17,0x5D,0x2A,
0xF5,0x81,0x10,0x26,0x02,0x97,0xE9,0x39,0x81,0x09,0x22,0x03,0x8B,0x30,0x39,0x8B,0x37,0x39,0x8D,0xDD,0x97,0xE5,0x48,0x48,0x48,0x48,0x97,0xE6,0x96,0xE5,
0x8D,0xE8,0xB7,0xC1,0x06,0x8D,0xCC,0x97,0xE5,0x8D,0xDF,0xB7,0xC1,0x07,0x96,0xE5,0x9B,0xE6,0x97,0xE5,0x39,0x8D,0xBC,0x97,0xE4,0x48,0x48,0x48,0x48,0x97,
0xE6,0x96,0xE4,0x8D,0xC7,0xB7,0xC1,0x04,0x8D,0xAB,0x97,0xE4,0x8D,0xBE,0xB7,0xC1,0x05,0x96,0xE4,0x9B,0xE6,0x97,0xE4,0x8D,0xBD,0x39,0xBD,0xFC,0x8A,0x96,
0xE5,0x44,0x44,0x44,0x44,0xBD,0xFC,0xDA,0xB7,0xC1,0x06,0x96,0xE5,0x84,0x0F,0xBD,0xFC,0xDA,0xB7,0xC1,0x07,0xBD,0xFC,0xC3,0x81,0x0C,0x26,0x0D,0x86,0x01,
0xB7,0xC1,0x27,0xB7,0xC1,0x17,0xBD,0xFC,0xE4,0x86,0x80,0x39,0x96,0xE5,0x04,0xA7,0x03,0x39,0x04,0xA6,0x03,0x97,0xE5,0x39,0x8D,0xF8,0xCE,0xFD,0x6E,0xBD,
0xFD,0x28,0xBD,0xFD,0x56,0x39,0x41,0x43,0x43,0x41,0x3D,0x3D,0x20,0x20,0x96,0xE5,0x04,0xA7,0x02,0x39,0x04,0xA6,0x02,0x97,0xE5,0x39,0x8D,0xF8,0xCE,0xFD,
0x8E,0xBD,0xFD,0x28,0xBD,0xFD,0x76,0x39,0x41,0x43,0x43,0x42,0x3D,0x3D,0x20,0x20,0x96,0xE5,0x04,0xA7,0x01,0x39,0x04,0xA6,0x01,0x97,0xE5,0x39,0x8D,0xF8,
0x8D,0x07,0xBD,0xFD,0x2B,0xBD,0xFD,0x96,0x39,0x96,0xE5,0x44,0x24,0x04,0xC6,0x63,0x20,0x02,0xC6,0x5F,0xF7,0xC1,0x05,0x44,0x24,0x04,0xC6,0x56,0x20,0x02,
0xC6,0x5F,0xF7,0xC1,0x04,0x44,0x24,0x04,0xC6,0x6F,0x20,0x02,0xC6,0x5F,0xF7,0xC1,0x03,0x44,0x24,0x04,0xC6,0x4E,0x20,0x02,0xC6,0x5F,0xF7,0xC1,0x02,0x44,
0x24,0x04,0xC6,0x69,0x20,0x02,0xC6,0x5F,0xF7,0xC1,0x01,0x44,0x24,0x04,0xC6,0x68,0x20,0x02,0xC6,0x5F,0xF7,0xC1,0x00,0x39,0xBD,0xFC,0x8A,0x96,0xE4,0x44,
0x44,0x44,0x44,0xBD,0xFC,0xDA,0xB7,0xC1,0x04,0x96,0xE4,0x84,0x0F,0xBD,0xFC,0xDA,0xB7,0xC1,0x05,0x96,0xE5,0x44,0x44,0x44,0x44,0xBD,0xFC,0xDA,0xB7,0xC1,
0x06,0x96,0xE5,0x84,0x0F,0xBD,0xFC,0xDA,0xB7,0xC1,0x07,0xBD,0xFC,0xC3,0x81,0x0C,0x26,0x11,0x86,0x01,0xB7,0xC1,0x47,0xB7,0xC1,0x37,0xB7,0xC1,0x27,0xB7,
0xC1,0x17,0xBD,0xFD,0x05,0x39,0x04,0x96,0xE4,0xA7,0x04,0x96,0xE5,0xA7,0x05,0x39,0x04,0xA6,0x04,0x97,0xE4,0xA6,0x05,0x97,0xE5,0x39,0x8D,0xF4,0xCE,0xFE,
0x60,0xBD,0xFD,0xF8,0xBD,0xFE,0x40,0x39,0x69,0x72,0x3D,0x3D,0x20,0x20,0x20,0x20,0x04,0x96,0xE4,0xA7,0x06,0x96,0xE5,0xA7,0x07,0x39,0x04,0xA6,0x06,0x97,
0xE4,0xA6,0x07,0x97,0xE5,0x39,0x8D,0xF4,0xCE,0xFE,0x88,0xBD,0xFD,0xF8,0xBD,0xFE,0x68,0x39,0x50,0x43,0x3D,0x3D,0x20,0x20,0x20,0x20,0x96,0xE5,0x44,0x44,
0x44,0x44,0xBD,0xFC,0xDA,0x8A,0x80,0xB7,0xC1,0x06,0x96,0xE5,0x84,0x0F,0xBD,0xFC,0xDA,0x8A,0x80,0xB7,0xC1,0x07,0x39,0x96,0xE7,0x44,0x44,0x44,0x44,0xBD,
0xFC,0xDA,0xB7,0xC1,0x00,0x96,0xE7,0x84,0x0F,0xBD,0xFC,0xDA,0xB7,0xC1,0x01,0x96,0xE8,0x44,0x44,0x44,0x44,0xBD,0xFC,0xDA,0xB7,0xC1,0x02,0x96,0xE8,0x84,
0x0F,0xBD,0xFC,0xDA,0xB7,0xC1,0x03,0x86,0x3D,0xB7,0xC1,0x04,0xB7,0xC1,0x05,0xDE,0xE7,0xA6,0x00,0x97,0xE5,0x39,0xCE,0xFF,0x23,0xBD,0xFC,0x8A,0xBD,0xFD,
0x05,0xDE,0xE4,0xDF,0xE7,0x8D,0xB6,0xBD,0xFD,0x2B,0x81,0x0F,0x27,0x1C,0x81,0x0B,0x27,0x0D,0x81,0x80,0x26,0x08,0xDE,0xE7,0x96,0xE5,0xA7,0x00,0x20,0xE7,
0x39,0xDE,0xE7,0x09,0xDF,0xE7,0xA6,0x00,0x97,0xE5,0x20,0xDB,0xDE,0xE7,0x08,0xDF,0xE7,0xA6,0x00,0x97,0xE5,0x20,0xD0,0x41,0x44,0x44,0x52,0x5F,0x5F,0x5F,
0x5F,0x7F,0x00,0xE9,0xCE,0xFF,0x23,0xBD,0xFC,0x8A,0xBD,0xFD,0x05,0xDE,0xE4,0xDF,0xE7,0xA6,0x00,0x97,0xE5,0xBD,0xFE,0xAB,0xBD,0xFE,0x90,0xBD,0xFC,0xE4,
0xDE,0xE7,0x96,0xE5,0xA7,0x00,0x08,0xDF,0xE7,0x7D,0x00,0xE9,0x26,0x02,0x20,0xE3,0x39,0xCE,0xFF,0x6D,0xBD,0xFC,0x8A,0xBD,0xFD,0x05,0xBD,0xFE,0x68,0xCE,
0xFF,0xF0,0xBD,0xFC,0x8A,0x05,0x39,0x20,0x44,0x4F,0x20,0x5F,0x5F,0x5F,0x5F,0xCE,0xFF,0x85,0xBD,0xFC,0x8A,0x04,0xDF,0xE7,0xBD,0xFE,0xAB,0xBD,0xFC,0xC3,
0x39,0x20,0x20,0x20,0x20,0x3D,0x3D,0x53,0x50,0x02,0xBD,0xFE,0x72,0xDE,0xE4,0xDF,0xE7,0xBD,0xFE,0xAB,0xBD,0xFE,0x90,0xBD,0xFC,0xC3,0x81,0x08,0x27,0xEB,
0x39,0xCE,0xFF,0xE8,0xBD,0xFC,0x8A,0xDE,0xEA,0x08,0x27,0x05,0x09,0x96,0xEC,0xA7,0x00,0xBD,0xFD,0x05,0xDE,0xE4,0xDF,0xEA,0xA6,0x00,0x97,0xEC,0x86,0x15,
0xA7,0x00,0x39,0xCE,0xFF,0xF0,0xBD,0xFC,0x8A,0x04,0xEE,0x06,0x09,0x9C,0xEA,0x27,0x02,0x05,0x39,0x96,0xEC,0xA7,0x00,0x04,0x96,0xEA,0xA7,0x06,0x96,0xEB,
0xA7,0x07,0x86,0xFF,0x97,0xEA,0x97,0xEB,0x05,0x39,0x42,0x52,0x2E,0x20,0x5F,0x5F,0x5F,0x5F,0x55,0x53,0x45,0x52,0x20,0x20,0x20,0x20,0x01,0x03,0x01,0x00,
0x01,0x06,0xFC,0x00};

static uint16_t APPS[0x0294] = {0x86,0x0A,0xBD,0xFB,0x70,0x86,0x0D,0xCE,0xF8,0x74,0xBD,0xFC,0x8A,0xBD,0xFC,0xC3,0x81,0x01,0x27,0x09,0x81,0x02,0x27,0x22,0x81,0x03,0x27,0x3B,0x15,0xCE,
0x00,0x00,0xDF,0xE4,0xCE,0xF8,0x7A,0x8C,0xF8,0xBC,0x27,0xF2,0xA6,0x00,0x08,0xDF,0xE7,0xDE,0xE4,0xA7,0x00,0x08,0xDF,0xE4,0xDE,0xE7,0x20,0xEB,0xCE,0x02,
0x00,0xDF,0xE4,0xCE,0xF8,0xBC,0x8C,0xF9,0x6D,0x27,0xD5,0xA6,0x00,0x08,0xDF,0xE7,0xDE,0xE4,0xA7,0x00,0x08,0xDF,0xE4,0xDE,0xE7,0x20,0xEB,0xCE,0x01,0x06,
0xDF,0xE4,0xCE,0xF9,0x6D,0x8C,0xFA,0x93,0x27,0xB8,0xA6,0x00,0x08,0xDF,0xE7,0xDE,0xE4,0xA7,0x00,0x08,0xDF,0xE4,0xDE,0xE7,0x20,0xEB,0x41,0x50,0x50,0x20,
0x3F,0x00,0x86,0x0A,0xBD,0xFB,0x70,0x86,0x0D,0xCE,0xF8,0xA1,0xBD,0xFC,0x8A,0xDF,0xE7,0xCE,0x80,0x00,0x09,0x01,0x01,0x01,0x01,0x26,0xF9,0xDE,0xE7,0x08,
0xDF,0xE7,0x8C,0xF8,0xB4,0x26,0xE7,0xBD,0xFC,0xC3,0x15,0x4D,0x45,0x53,0x53,0x41,0x47,0x45,0x20,0x49,0x4E,0x20,0x41,0x20,0x42,0x4F,0x54,0x54,0x4C,0x45,
0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x86,0x0A,0xBD,0xFB,0x70,0xC6,0x01,0xF7,0xF0,0x20,0xB6,0xF0,0x20,0xF6,0xF0,0x21,0xD7,0x04,0x97,0x03,0x58,0x49,
0x58,0x49,0x97,0x07,0xD7,0x08,0xDB,0x04,0xD7,0x04,0x99,0x03,0x97,0x03,0x96,0x07,0xD6,0x08,0x58,0x49,0x58,0x49,0xDB,0x04,0xD7,0x04,0x99,0x03,0x97,0x03,
0xBD,0xF9,0x01,0x86,0x02,0xCE,0xFF,0xFF,0x09,0x26,0xFD,0x4A,0x26,0xF7,0x7E,0xF8,0xC1,0xCE,0x00,0x00,0xDF,0x05,0x4F,0x8D,0x0A,0x8D,0x08,0x8D,0x06,0x8D,
0x04,0xBD,0xF9,0x49,0x39,0x4F,0x78,0x00,0x04,0x79,0x00,0x03,0x49,0x97,0x02,0x97,0x01,0xD6,0x04,0x96,0x03,0x58,0x49,0x79,0x00,0x01,0x58,0x49,0x79,0x00,
0x01,0xDB,0x04,0xD7,0x04,0x99,0x03,0x97,0x03,0x96,0x01,0x99,0x02,0xC6,0x04,0x78,0x00,0x06,0x79,0x00,0x05,0x5A,0x26,0xF7,0x9A,0x06,0x97,0x06,0x39,0x86,
0x0D,0xBD,0xFB,0x70,0x86,0x20,0xBD,0xFB,0x70,0x96,0x05,0x44,0x44,0x44,0x44,0x8A,0xB0,0xBD,0xFB,0x70,0x96,0x05,0x84,0x0F,0x8A,0x30,0xBD,0xFB,0x70,0x96,
0x06,0xBD,0xFB,0xEA,0x39,0x7E,0xF9,0x77,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7D,0x00,0x06,0x27,0x0C,0x2B,0x05,0xCE,0x00,0x00,0x20,0x06,0xCE,0x00,0x03,
0x20,0x01,0x3B,0x6A,0x02,0x26,0xFB,0x86,0x3C,0xA7,0x02,0x86,0x99,0xAB,0x01,0x19,0x81,0x99,0x26,0x04,0x86,0x59,0x6A,0x00,0xA7,0x01,0xA6,0x00,0x2A,0x05,
0x4F,0xA7,0x00,0xA7,0x01,0x86,0x0D,0xBD,0xFB,0x70,0x96,0x00,0xBD,0xFC,0xDA,0xBD,0xFB,0x70,0x96,0x01,0xBD,0xFB,0xEA,0x86,0x20,0xBD,0xFB,0x70,0xBD,0xFB,
0x70,0x96,0x03,0xBD,0xFC,0xDA,0xBD,0xFB,0x70,0x96,0x04,0xBD,0xFB,0xEA,0x86,0x5F,0x7D,0x00,0x06,0x2B,0x04,0xB7,0xC1,0x03,0x3B,0xB7,0xC1,0x04,0x3B,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCE,0x00,0x00,0x6F,0x01,0x6F,0x04,0x6F,0x06,0x86,0x3C,0xA7,0x02,0xA7,0x05,
0x86,0x05,0xA7,0x00,0xA7,0x03,0x86,0x0A,0xBD,0xFB,0x70,0xBD,0xFC,0xC3,0x4D,0x27,0x06,0x86,0x01,0x97,0x06,0x20,0xF4,0x86,0x82,0x97,0x06,0x20,0xEE,0x00};

static volatile uint16_t xtra;
static volatile uint16_t i;
static volatile unsigned int j;
static volatile uint16_t a = 0;          //accumulator A
static volatile uint16_t b = 0;          //accumulator B
static volatile uint16_t zero = 0;
static volatile uint16_t acc = 0;        //Intermediate Instruction result
static volatile uint16_t x = 0;          //Index register X
static volatile uint16_t pc = 0xffd8;    //Program counter
static volatile uint16_t sp = 0x00f3;    //Stack pointer
static volatile uint16_t sp1 = 0x00f3;    //Stack pointer
static volatile uint16_t sp2 = 0x00cb;    //Stack pointer
static volatile uint16_t ss_flag = 0;         //Single step flag
static volatile uint16_t ir = 0;         //Instruction register
static volatile uint16_t ccr = 0xd0;     //Condition code register
static volatile uint16_t mp = 0;         //Memory pointer address
static volatile uint16_t mem = 0;        //Memory data
static volatile uint16_t uart_status = 0; //bit-1 write ready, bit=0 read ready
static volatile uint16_t uart_write = 0;  //data write
static volatile uint16_t uart_read = 0;   //data read
static volatile unsigned long xbyte = 0;  //holds xor value for I/O write
static volatile unsigned long inpins = 0; //holds I/O read value
static volatile unsigned char trace = 0;  //op code trace flag
unsigned int mod1 = 0;
unsigned int mod_uart = 0;                //modulus count for uart status check 
volatile unsigned char adc_flag = 0;      //flag for adc read 
volatile uint16_t MITS_RAM[65536];        //64K of RAM  
volatile unsigned int adc_val;            //adc read temp value
volatile uint16_t nmi_flag = 0;           //flag for NMI falling edge

volatile unsigned int ascii[128]={
  0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000, //null soh stx etx eot enq ack bel
  0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000, //bbs ht lf vt ff cr so si
  0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000, //dle dc1 dc2 dc3 dc4 nak syn etb
  0b00000001,0b00000010,0b00000100,0b00001000,0b00010000,0b00100000,0b01000000,0b10000000, //can em sub esc fs gs rs us
  0b00000000,0b10110000,0b01000010,0b00000000,0b00000000,0b00000000,0b00000000,0b00100000, //SP ! " # $ % & '
  0b01001110,0b01111000,0b00000000,0b00000111,0b00010000,0b00000001,0b10000000,0b00100101, //( ) * + , - . /
  0b01111110,0b00110000,0b01101101,0b01111001,0b00110011,0b01011011,0b01011111,0b01110000, //0 1 2 3 4 5 6 7
  0b01111111,0b01110011,0b00000000,0b00000000,0b01000011,0b00001001,0b01100001,0b01100101, //8 9 : ; < = > ?
  0b00000000,0b01110111,0b00011111,0b01001110,0b00111101,0b01001111,0b01000111,0b01011110, //@ A B C D E F G
  0b00110111,0b00000110,0b00111100,0b00000111,0b00001110,0b01010101,0b00010101,0b00011101, //H I J K L M N O
  0b01100111,0b01110011,0b01100110,0b01011011,0b01110000,0b00011100,0b00111110,0b01011100, //P Q R S T U V W
  0b00010011,0b00111011,0b01101100,0b01001110,0b00010011,0b01111000,0b00000000,0b00001000, //X Y Z [ \ ] ^ _
  0b00000000,0b00011101,0b00011111,0b00001101,0b00111101,0b01001111,0b01000111,0b01011110, // slash a b c d e f g
  0b00010111,0b00000100,0b00011000,0b00000011,0b00001100,0b01010101,0b00010101,0b00011101, //h i j k l m n o
  0b01100111,0b01110011,0b00000101,0b01011011,0b00010001,0b00011100,0b00111110,0b01011100, //p q r s t u v w
  0b00010011,0b00111011,0b01101100,0b01001110,0b00110000,0b01111000,0b00000001,0b01001001 //x y z { | } ~ =
 };  


void load_boot_rom(void){
 if (gpio_get(20)==1) for ( i = 0; i < 1133; i++) MITS_RAM[0xFB93 + i] = PICO_BUG[i];
 else {
  for ( i = 0; i < 0x490; i++) MITS_RAM[0xFB70 + i] = TRAINER[i];
  MITS_RAM[0xC109] = 0x80; //OLED display use 7-seg sim only
  for (i = 0; i < 0x294; i++) MITS_RAM[0xF800 + i] = APPS[i];
 }
}

void setup() {
  Serial.begin(115200);
//  pinMode(0, INPUT_PULLUP);     //UART1 data out
//  pinMode(1, INPUT_PULLUP);     //UART1 data in
  pinMode(2, INPUT_PULLUP);  //key bit 0
  pinMode(3, INPUT_PULLUP);  //key bit 1
  pinMode(4, INPUT_PULLUP);  //key bit 2
  pinMode(5, INPUT_PULLUP);  //key bit 3
  pinMode(6, INPUT_PULLUP);  //key bit 4
  pinMode(7, INPUT_PULLUP);  //key bit 5
  pinMode(8, INPUT_PULLUP);  //key address 0
  pinMode(9,INPUT_PULLUP);   //key address 1
  pinMode(10, INPUT_PULLUP); //key address 2
  pinMode(11, OUTPUT); //*CS
//  pinMode(12, OUTPUT); //clock
  pinMode(14, OUTPUT);          //GP OUT
  pinMode(15, OUTPUT);          //GP OUT
  pinMode(20,INPUT_PULLUP);     //0 to boot trainer, 1 to boot serial monitor
  pinMode(22,INPUT_PULLUP);     //cassette read=1 write=0 
  pinMode(LOOP_P, OUTPUT);     //Instruction Loop Pulse Out
  pinMode(NMI, INPUT_PULLUP);     //*NMI in
  pinMode(IRQ, INPUT_PULLUP);     //*IRQ in
  pinMode(RESET, INPUT_PULLUP);   //*Reset in
  pinMode(TR_EN, INPUT_PULLUP);   //*Trace Enable in
  pinMode(26, INPUT);   //A0 analog in
  pinMode(27, INPUT);   //A1 analog in
  pinMode(28, INPUT);   //A2 analog in

 Serial1.begin(9600,SERIAL_8N1);  

 for (i = 0; i < 0xff00; i++) MITS_RAM[i] = 0;

 load_boot_rom();
 
 decimal_table();

 MITS_RAM[ROM_START]=0xFF;   //Mark end of RAM with some 0xFF bytes
 MITS_RAM[ROM_START+1]=0xFF;
 MITS_RAM[ROM_START+2]=0xFF;
 MITS_RAM[ROM_START+3]=0xFF;
 MITS_RAM[ROM_START+4]=0xFF; 

 delay(2500);
 tone(LOOP_P,60);
 attachInterrupt(digitalPinToInterrupt(NMI),service_nmi,FALLING);
 
}


void service_nmi(void){ //NMI falling edge detected
  nmi_flag = 1;
}

void loop() {
 i = 0;
 nmi_flag = 0;

while (true){ 
 if ((0x010000 & inpins) == 0){  //RESET LOW
  Serial.println("reset");
  a = 0;
  b = 0;
  acc = 0;
  x = 0;
  sp = 0x00f3;
  ir = 0;
  ccr = 0xd0;
  mp = 0;
  xbyte = 0;
  MITS_RAM[ROM_START]=0xFF;
  MITS_RAM[ROM_START+1]=0xFF;
  MITS_RAM[ROM_START+2]=0xFF;
  MITS_RAM[ROM_START+3]=0xFF;
  MITS_RAM[ROM_START+4]=0xFF; 
  uart_status = 0;
  uart_write = 0;
  uart_read = 0;
  load_boot_rom();
  pc = (MITS_RAM[0xFFFE] << 8) + (MITS_RAM[0xFFFF] & 0xFF);
  Serial.println(pc,HEX);
  Serial.println();
  trace = 0;
  delay(2000);
  nmi_flag = 0;
  ss_flag = 0;
 }
 
// gpio_set_mask(1ul << LOOP_P); //timing loop pulse

 if (((0x080000 & inpins) == 0) && ((ccr & 0b00010000)==0) && (ss_flag==0)){ //IRQ LOW
      MITS_RAM[sp--] = pc & 0xFF;
      MITS_RAM[sp--] = (pc >> 8) & 0xff;
      MITS_RAM[sp--] = x & 0xFF;
      MITS_RAM[sp--] = (x >> 8) & 0xff;
      MITS_RAM[sp--] = a & 0xff;
      MITS_RAM[sp--] = b & 0xff;
      MITS_RAM[sp--] = ccr & 0xff;
      pc = (MITS_RAM[0xFFF8] << 8) + (MITS_RAM[0xFFF9] & 0xFF); 
      ccr = ccr & 0b00010000;  
 }

 if ((nmi_flag) && (ss_flag==0)) { //NMI flagged
      MITS_RAM[sp--] = pc & 0xFF;
      MITS_RAM[sp--] = (pc >> 8) & 0xff;
      MITS_RAM[sp--] = x & 0xFF;
      MITS_RAM[sp--] = (x >> 8) & 0xff;
      MITS_RAM[sp--] = a & 0xff;
      MITS_RAM[sp--] = b & 0xff;
      MITS_RAM[sp--] = ccr & 0xff;
      pc = (MITS_RAM[0xFFFC] << 8) + (MITS_RAM[0xFFFD] & 0xFF); 
      ccr = ccr & 0b00010000;
      nmi_flag = 0;  
 }

 if (ss_flag!=0) ss_flag--;
 ir = MITS_RAM[pc & 0xffff] & 0xff;
 if (ss_flag==3) ir = 0x05;
 if (ss_flag==1) ir = 0x15;

 
if (trace) {
 Serial.print("pc");
 Serial1.print("pc");
 print4hex(pc);
 Serial.print(" ir");
 Serial1.print(" ir");
 print2hex(ir);
 sprintf(text," %s",opcode[ir]);
 Serial.print(text);
 Serial1.print(text);
} 
 
  switch (ir) {
     case 0x00: //no instr
      pc++;
      break;
     case 0x01: //nop
      pc++;
      break;
     case 0x02: //single step
      ss_flag = 4;
      pc++;
      break;
     case 0x03: //x to sp2
      sp2 = x;
      pc++;
      break;
     case 0x04: //sp2 to x
      x = sp2;
      pc++;
      break;
     case 0x05: //run stack 2
      if (ss_flag==0) pc++;
      MITS_RAM[sp--] = pc & 0xFF;       //push registers on stack 1
      MITS_RAM[sp--] = (pc >> 8) & 0xff;
      MITS_RAM[sp--] = x & 0xFF;
      MITS_RAM[sp--] = (x >> 8) & 0xff;
      MITS_RAM[sp--] = a & 0xff;
      MITS_RAM[sp--] = b & 0xff;
      MITS_RAM[sp--] = ccr & 0xff;
      sp1 = sp;
      //ccr = ccr & 0b00010000;    
      sp = sp2;   
      ccr = MITS_RAM[++sp]  & 0xff;   //pull registers stack 2
      b = MITS_RAM[++sp] & 0xff;
      a = MITS_RAM[++sp] & 0xff;
      x = (MITS_RAM[++sp] << 8);
      x = (MITS_RAM[++sp] & 0xff) + x;
      pc = (MITS_RAM[++sp] << 8);
      pc = (MITS_RAM[++sp] & 0xff) + pc;      
      break;
     case 0x06: //tap
      ccr = a;
      pc++;
      break;
     case 0x07: //tpa
      a = ccr;
      pc++;
      break;
     case 0x08: //inx
      x = x + 1;
      if (x==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x09: //dex
      x = x - 1;
      if (x==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x0A: //clv
      ccr = ccr & 0b11111101; 
      pc++;
      break;
     case 0x0B: //sev
      ccr = ccr | 0b00000010; 
      pc++;
      break;
     case 0x0C: //clc
      ccr = ccr & 0b11111110; 
      pc++;
      break;
     case 0x0D: //sec 
      ccr = ccr | 0b00000001; 
      pc++;
      break;
     case 0x0E: //cli
      ccr = ccr & 0b11101111; 
      pc++;
      break;
     case 0x0F: //sei
      ccr = ccr | 0b00010000; 
      pc++;
      break;
     case 0x10: //sba
      acc = (a - b) & 0xff;
      if (((a ^ b) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~a) & b) | (acc & (~a)) | (b & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      a = acc;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x11: //cba 
      acc = (a - b) & 0xff;
      if (((a ^ b) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~a) & b) | (acc & (~a)) | (b & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x12: //trace on new instr
      pc++;
      trace = trace | 1;
      break;
     case 0x13: //trace off new instr
      pc++;
      trace = trace & 0b11111110;
      break;
     case 0x14: //nba and b to a
      a = a & b;
      ccr = ccr & 0b11111101; //clear v  
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((a & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg            
      pc++;
      break;
     case 0x15: //run stack 1
      if (ss_flag==0) pc++;
      MITS_RAM[sp--] = pc & 0xFF;       //push registers on stack 2
      MITS_RAM[sp--] = (pc >> 8) & 0xff;
      MITS_RAM[sp--] = x & 0xFF;
      MITS_RAM[sp--] = (x >> 8) & 0xff;
      MITS_RAM[sp--] = a & 0xff;
      MITS_RAM[sp--] = b & 0xff;
      MITS_RAM[sp--] = ccr & 0xff;
      sp2 = sp;
      //ccr = ccr & 0b00010000;    
      sp = sp1;   
      ccr = MITS_RAM[++sp]  & 0xff;   //pull registers stack 1
      b = MITS_RAM[++sp] & 0xff;
      a = MITS_RAM[++sp] & 0xff;
      x = (MITS_RAM[++sp] << 8);
      x = (MITS_RAM[++sp] & 0xff) + x;
      pc = (MITS_RAM[++sp] << 8);
      pc = (MITS_RAM[++sp] & 0xff) + pc;  
      break;
     case 0x16: //tab 
      b =  a;
      ccr = ccr & 0b11111101; //clear v  
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((a & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg            
      pc++;
      break;      
     case 0x17: //tba 
      a =  b;
      ccr = ccr & 0b11111101; //clear v  
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((b & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg            
      pc++;
      break;      
     case 0x18: //no instr
      pc++;
      break;
     case 0x19: //daa
      mem = a;
      if (ccr & 1) mem = mem | 0x200;
      if (ccr & 32) mem = mem | 0x100;
      acc = DAA[mem];
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg 
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry 
      a = acc & 0xff;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero            
      pc++;
      break;
     case 0x1a: //no instr
      pc++;
      break;
     case 0x1b: //aba
      acc = a + b;
      if (((~(a ^ b)) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry 
      if (((a & b) | (b & (~acc)) | ((~acc) & a)) & 0x08) ccr = ccr | 0b00100000; else ccr = ccr & 0b11011111; //test h          
      a = acc & 0xff;
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x1c: //no instr
      pc++;
      break;
     case 0x1d: //no instr
      pc++;
      break;
     case 0x1e: //no instr
      pc++;
      break;
     case 0x1f: //no instr
      pc++;
      break;
     case 0x20: //bra 
      pc++;
      acc = MITS_RAM[pc];
      if (acc > 127) acc = acc | 0xff00;
      pc = pc + acc + 1;
      break;
     case 0x21: //no instr
      pc++;
      break;
     case 0x22: //bhi 
      pc++;
      acc = MITS_RAM[pc];
      if (acc > 127) acc = acc | 0xff00;
      if ( (((ccr>>2) | ccr) & 1) == 0)  pc = pc + acc + 1; else pc++;
      break;
     case 0x23: //bls 
      pc++;
      acc = MITS_RAM[pc];
      if (acc > 127) acc = acc | 0xff00;
      if (((ccr>>2) | ccr) & 1) pc = pc + acc + 1; else pc++;
      break;
     case 0x24: //bcc 
      pc++;
      acc = MITS_RAM[pc];
      if (acc > 127) acc = acc | 0xff00;
      if (ccr & 0b00000001) pc++; else pc = pc + acc + 1;
      break;
     case 0x25: //bcs 
      pc++;
      acc = MITS_RAM[pc];
      if (acc > 127) acc = acc | 0xff00;
      if (ccr & 0b00000001) pc = pc + acc + 1; else pc++;
      break;
     case 0x26: //bne 
      pc++;
      acc = MITS_RAM[pc];
      if (acc > 127) acc = acc | 0xff00;
      if (ccr & 0b00000100) pc++; else pc = pc + acc + 1;
      break;
     case 0x27: //beq 
      pc++;
      acc = MITS_RAM[pc];
      if (acc > 127) acc = acc | 0xff00;
      if (ccr & 0b00000100) pc = pc + acc + 1; else pc++;
      break;
     case 0x28: //bvc 
      pc++;
      acc = MITS_RAM[pc];
      if (acc > 127) acc = acc | 0xff00;
      if (ccr & 0b00000010) pc++; else pc = pc + acc + 1;
      break;
     case 0x29: //bvs 
      pc++;
      acc = MITS_RAM[pc];
      if (acc > 127) acc = acc | 0xff00;
      if (ccr & 0b00000010) pc = pc + acc + 1; else pc++;
      break;
     case 0x2a: //bpl 
      pc++;
      acc = MITS_RAM[pc];
      if (acc > 127) acc = acc | 0xff00;
      if (ccr & 0b00001000) pc++; else pc = pc + acc + 1;
      break;
     case 0x2b: //bmi 
      pc++;
      acc = MITS_RAM[pc];
      if (acc > 127) acc = acc | 0xff00;
      if (ccr & 0b00001000) pc = pc + acc + 1; else pc++;
      break;
     case 0x2c: //bge 
      pc++;
      acc = MITS_RAM[pc];
      if (acc > 127) acc = acc | 0xff00;
      if (((ccr >> 2) ^ ccr) & 0b00000010) pc++; else pc = pc + acc + 1;
      break;
     case 0x2d: //blt 
      pc++;
      acc = MITS_RAM[pc];
      if (acc > 127) acc = acc | 0xff00;
      if (((ccr >> 2) ^ ccr) & 0b00000010) pc = pc + acc + 1; else pc++;
      break;
     case 0x2e: //bgt 
      pc++;
      acc = MITS_RAM[pc];
      if (acc > 127) acc = acc | 0xff00;
      if ( (( ( (ccr >> 2) ^ ccr) & 0b00000010) | (ccr >> 1)) & 0b00000010) pc++; else pc = pc + acc + 1;
      break;
     case 0x2f: //ble 
      pc++;
      acc = MITS_RAM[pc];
      if (acc > 127) acc = acc | 0xff00;
      if ( (( ( (ccr >> 2) ^ ccr) & 0b00000010) | (ccr >> 1)) & 0b00000010) pc = pc + acc + 1; else pc++;
      break;
     case 0x30: //tsx
      pc++;
      x = sp+1;
      break;
     case 0x31: //ins
      pc++;
      sp++;
      break;
     case 0x32: //pula 
      pc++;
      sp++;
      a = MITS_RAM[sp] & 0xff;
      break;
     case 0x33: //pulb 
      pc++;
      sp++;
      b = MITS_RAM[sp] & 0xff;
      break;
     case 0x34: //des
      pc++;
      sp--;
      break;
     case 0x35: //txs
      pc++;
      sp = x-1;
      break;
     case 0x36: //psha 
      pc++;
      if (sp < ROM_START) MITS_RAM[sp--] = a  & 0xff;
      break;
     case 0x37: //pshb 
      pc++;
      if (sp < ROM_START) MITS_RAM[sp--] = b  & 0xff;
      break;
     case 0x38: //no instr
      pc++;
      break;     
     case 0x39: //rts
      pc = (MITS_RAM[++sp] << 8);
      pc = pc + (MITS_RAM[++sp] & 0xff);
      break;     
     case 0x3A: //no instr
      pc++;
      break;     
     case 0x3B: //rti
      ccr = MITS_RAM[++sp]  & 0xff;
      b = MITS_RAM[++sp] & 0xff;
      a = MITS_RAM[++sp] & 0xff;
      x = (MITS_RAM[++sp] << 8);
      x = (MITS_RAM[++sp] & 0xff) + x;
      pc = (MITS_RAM[++sp] << 8);
      pc = (MITS_RAM[++sp] & 0xff) + pc;
      break;     
     case 0x3C: //no instr
      pc++;
      break;     
     case 0x3D: //no instr
      pc++;
      break;     
     case 0x3E: //wai
      //stay on same instruction until interrupt occurs
      break;     
     case 0x3F: //swi
      ss_flag = 0;
      pc++;
      MITS_RAM[sp--] = pc & 0xFF;
      MITS_RAM[sp--] = (pc >> 8) & 0xff;
      MITS_RAM[sp--] = x & 0xFF;
      MITS_RAM[sp--] = (x >> 8) & 0xff;
      MITS_RAM[sp--] = a & 0xff;
      MITS_RAM[sp--] = b & 0xff;
      MITS_RAM[sp--] = ccr & 0xff;
      pc = (MITS_RAM[0xFFFA] << 8) + (MITS_RAM[0xFFFB] & 0xFF); 
      ccr = ccr & 0b00010000;
      break;     
     case 0x40: //nega
      acc = ((~a) + 1) & 0xff;
      if (a == 0x80)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (a == 0) ccr = ccr & 0b11111110; else ccr = ccr | 0b00000001; //test carry            
      a = acc;
      pc++;
      break;
     case 0x41: //no instr
      pc++;
      break;     
     case 0x42: //no instr
      pc++;
      break;     
     case 0x43: //coma
      acc = (~a) & 0xff;
      ccr = ccr & 0b11111101; //reset v    
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((acc & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg            
      ccr = ccr | 0b00000001; //set carry            
      a = acc;
      pc++;
      break;
     case 0x44: //lsra
      acc = a;
      if (acc & 1)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      ccr = ccr & 0b11110111;  //reset neg            
      if (acc & 1) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      a = (acc >> 1)  & 0xff;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x45: //no instr
      pc++;
      break;     
     case 0x46: //rora 
      if (ccr & 1) acc = a | 0x100; else acc = a;
      if (((acc & 1) ^ ((acc & 0x100) >> 8)) & 1)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc & 0x100) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc & 1) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      a = (acc>>1) & 0xff;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x47: //asra 
      if (a & 0x80) acc = a | 0xff00; else acc = a;
      if (((acc & 1 ) ^ ((acc & 0x100) >> 8)) & 1)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc & 0x100) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc & 1) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      a = (acc>>1) & 0xff;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x48: //asla 
      acc = a << 1;
      if (((acc & 0x80 ) ^ ((acc & 0x100) >> 1)) & 0x80)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      a = acc & 0xff;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x49: //rola 
      acc = (a << 1) | (ccr & 1);
      if (((acc & 0x80 ) ^ ((acc & 0x100) >> 1)) & 0x80)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      a = acc & 0xff;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x4A: //deca
      if (a == 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      acc = (a - 1) & 0xff;
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      a = acc;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x4B: //no instr
      pc++;
      break;     
     case 0x4C: //inca
      if (a == 0x7F) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      acc = (a + 1) & 0xff;
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      a = acc;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x4D: //tsta
      ccr = ccr & 0b11111100; //clear v and c         
      if (a & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x4E: //no instr
      pc++;
      break;     
     case 0x4F: //clra
      a = 0;
      ccr = (ccr & 0b00110000) | 0b00000100;
      pc++;
      break;
     case 0x50: //negb
      acc = ((~b) + 1) & 0xff;
      if (b == 0x80)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (b == 0) ccr = ccr & 0b11111110; else ccr = ccr | 0b00000001; //test carry            
      b = acc;
      pc++;
      break;
     case 0x51: //no instr
      pc++;
      break;     
     case 0x52: //no instr
      pc++;
      break;     
     case 0x53: //comb
      acc = (~b) & 0xff;
      ccr = ccr & 0b11111101; //reset v    
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((acc & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg            
      ccr = ccr | 0b00000001; //set carry            
      b = acc;
      pc++;
      break;
     case 0x54: //lsrb
      acc = b;
      if (acc & 1)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      ccr = ccr & 0b11110111;  //reset neg            
      if (acc & 1) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      b = (acc >> 1)  & 0xff;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x55: //no instr
      pc++;
      break;     
     case 0x56: //rorb 
      if (ccr & 1) acc = b | 0x100; else acc = b;
      if ((acc ^ ((acc & 0x100) >> 8)) & 1)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc & 0x100) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc & 1) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      b = (acc>>1) & 0xff;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x57: //asrb 
      if (b & 0x80) acc = b | 0xff00; else acc = b;
      if ((acc ^ ((acc & 0x100) >> 8)) & 1)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc & 0x100) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc & 1) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      b = (acc>>1) & 0xff;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x58: //aslb 
      acc = b << 1;
      if (((acc & 0x80 ) ^ ((acc & 0x100) >> 1)) & 0x80)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      b = acc & 0xff;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x59: //rolb 
      acc = (b << 1) | (ccr & 1);
      if (((acc & 0x80 ) ^ ((acc & 0x100) >> 1)) & 0x80)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      b = acc & 0xff;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x5A: //decb
      if (b == 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      acc = (b - 1) & 0xff;
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      b = acc;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x5B: //no instr
      pc++;
      break;     
     case 0x5C: //incb *
      if (b == 0x7F) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      acc = (b + 1) & 0xff;
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      b = acc;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x5D: //tstb
      ccr = ccr & 0b11111100; //clear v and c         
      if (b & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x5E: //no instr
      pc++;
      break;     
     case 0x5F: //clrb
      b = 0;
      ccr = (ccr & 0b00110000) | 0b00000100;
      pc++;
      break;
     case 0x60: //neg indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      acc = ((~mem) + 1) & 0xff;
      if (mem == 0x80)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (mem == 0) ccr = ccr & 0b11111110; else ccr = ccr | 0b00000001; //test carry            
      if (mp < ROM_START) MITS_RAM[mp] = acc;
      pc++;
      break;
     case 0x61: //no instr
      pc++;
      break;     
     case 0x62: //no instr
      pc++;
      break;     
     case 0x63: //com indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      acc = (~MITS_RAM[mp]) & 0xff;
      ccr = ccr & 0b11111101; //reset v    
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((acc & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg            
      ccr = ccr | 0b00000001; //set carry            
      if (mp < ROM_START) MITS_RAM[mp] = acc;
      pc++;
      break;
     case 0x64: //lsr indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      acc = MITS_RAM[mp];
      if (acc & 1)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      ccr = ccr & 0b11110111;  //reset neg            
      if (acc & 1) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      acc = (acc >> 1)  & 0xff;
      if (mp < ROM_START) MITS_RAM[mp] = acc;
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x65: //no instr
      pc++;
      break;     
     case 0x66: //ror indexed 
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      if (ccr & 1) acc = MITS_RAM[mp] | 0x100; else acc = MITS_RAM[mp];
      if (((acc & 1) ^ ((acc & 0x100) >> 8)) & 1)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc & 0x100) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc & 1) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      acc = (acc>>1) & 0xff;
      if (mp < ROM_START) MITS_RAM[mp] = acc;
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x67: //asr indexed 
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff; 
      acc = MITS_RAM[mp];     
      if (acc & 0x80) acc = acc | 0xff00;
      if (((acc & 1 ) ^ ((acc & 0x100) >> 8)) & 1)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc & 0x100) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc & 1) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      acc = (acc>>1) & 0xff;
      if (mp < ROM_START) MITS_RAM[mp] = acc;
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x68: //asl indexed 
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff; 
      acc = MITS_RAM[mp] << 1;
      if (((acc & 0x80 ) ^ ((acc & 0x100) >> 1)) & 0x80)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      acc = acc & 0xff;
      if (mp < ROM_START) MITS_RAM[mp] = acc;      
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x69: //rol indexed 
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff; 
      acc = (MITS_RAM[mp] << 1) | (ccr & 1);
      if (((acc & 0x80 ) ^ ((acc & 0x100) >> 1)) & 0x80)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      acc = acc & 0xff;
      if (mp < ROM_START) MITS_RAM[mp] = acc;
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x6A: //dec indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff; 
      acc = MITS_RAM[mp] & 0xff;
      if (acc == 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      acc = (acc - 1) & 0xff;
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (mp < ROM_START) MITS_RAM[mp] = acc;
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x6B: //no instr
      pc++;
      break;     
     case 0x6C: //inc indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff; 
      acc = MITS_RAM[mp];      
      if (acc == 0x7F) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      acc = (acc + 1) & 0xff;
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (mp < ROM_START) MITS_RAM[mp] = acc;
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x6D: //tst indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff; 
      acc = MITS_RAM[mp];     
      ccr = ccr & 0b11111100; //clear v and c         
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x6E: //jmp indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      pc = mp;
      break;
     case 0x6F: //clr indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff; 
      if (mp < ROM_START) MITS_RAM[mp] = 0;     
      ccr = (ccr & 0b00110000) | 0b00000100;
      pc++;
      break;
     case 0x70: //neg extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      acc = ((~mem) + 1) & 0xff;
      if (mem == 0x80)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (mem == 0) ccr = ccr & 0b11111110; else ccr = ccr | 0b00000001; //test carry            
      if (mp < ROM_START) MITS_RAM[mp] = acc;
      pc++;
      break;
     case 0x71: //no instr
      pc++;
      break;     
     case 0x72: //no instr
      pc++;
      break;     
     case 0x73: //com extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc])  & 0xffff;
      acc = (~MITS_RAM[mp]) & 0xff;
      ccr = ccr & 0b11111101; //reset v    
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((acc & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg            
      ccr = ccr | 0b00000001; //set carry            
      if (mp < ROM_START) MITS_RAM[mp] = acc;
      pc++;
      break;
     case 0x74: //lsr extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      acc = MITS_RAM[mp];
      if (acc & 1)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      ccr = ccr & 0b11110111;  //reset neg            
      if (acc & 1) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      acc = (acc >> 1)  & 0xff;
      if (mp < ROM_START) MITS_RAM[mp] = acc;
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x75: //no instr
      pc++;
      break;     
     case 0x76: //ror extended 
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc])  & 0xffff;
      acc = MITS_RAM[mp];
      if (ccr & 1) acc = MITS_RAM[mp] | 0x100;
      if (((acc & 1) ^ ((acc & 0x100) >> 8)) & 1)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc & 0x100) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc & 1) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      acc = (acc>>1) & 0xff;
      if (mp < ROM_START) MITS_RAM[mp] = acc;
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x77: //asr extended 
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      acc = MITS_RAM[mp]; 
      if (acc & 0x80) acc = acc | 0xff00;
      if (((acc & 1 ) ^ ((acc & 0x100) >> 8)) & 1)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc & 0x100) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc & 1) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      acc = (acc>>1) & 0xff;
      if (mp < ROM_START) MITS_RAM[mp] = acc;
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x78: //asl extended 
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      acc = MITS_RAM[mp] << 1;
      if (((acc & 0x80 ) ^ ((acc & 0x100) >> 1)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      acc = acc & 0xff;
      if (mp < ROM_START) MITS_RAM[mp] = acc;      
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x79: //rol extended 
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      acc = (MITS_RAM[mp] << 1) | (ccr & 1);
      if (((acc & 0x80 ) ^ ((acc & 0x100) >> 1)) & 0x80)  ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      acc = acc & 0xff;
      if (mp < ROM_START) MITS_RAM[mp] = acc;
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x7A: //dec extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      acc = MITS_RAM[mp] & 0xff;
      if (acc == 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      acc = (acc - 1) & 0xff;
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (mp < ROM_START) MITS_RAM[mp] = acc;
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x7B: //no instr
      pc++;
      break;     
     case 0x7C: //inc extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      acc = MITS_RAM[mp];      
      if (acc == 0x7F) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v    
      acc = (acc + 1) & 0xff;
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (mp < ROM_START) MITS_RAM[mp] = acc;
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x7D: //tst extended 
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      acc = MITS_RAM[mp];     
      ccr = ccr & 0b11111100; //clear v and c         
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111;  //test neg            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x7E: //jmp extended 
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      pc = mp;
      break;
     case 0x7F: //clr extended 
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      if (mp < ROM_START) MITS_RAM[mp] = 0;     
      ccr = (ccr & 0b00110000) | 0b00000100;
      pc++;
      break;
     case 0x80: //suba immediate
      pc++;
      mem = MITS_RAM[pc];
      acc = (a - mem) & 0xff;
      if (((a ^ mem) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~a) & mem) | ((acc) & (~a)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      a = acc;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x81: //cmpa immediate 
      pc++;
      mem = MITS_RAM[pc];
      acc = (a - mem) & 0xff;
      if (((a ^ mem) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~a) & mem) | ((acc) & (~a)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x82: //sbca immediate
      pc++;
      mem = MITS_RAM[pc];
      if (ccr & 1) acc = (a - mem - 1) & 0xff; else acc = (a - mem) & 0xff;
      if (((a ^ mem) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~a) & mem) | ((acc) & (~a)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      a = acc;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x83: //no instr
      pc++;
      break;     
     case 0x84: //anda immediate
      pc++;
      mem = MITS_RAM[pc];
      a = (a & mem)  & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (a & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x85: //BIta immediate
      pc++;
      mem = MITS_RAM[pc];
      acc = (a & mem)  & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x86: //lDAA imed 
      pc++;
      a = MITS_RAM[pc] & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((a & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg
      pc++;
      break;
     case 0x87: //staa imed (not defined)
      pc++;
      break;
     case 0x88: //eora immediate
      pc++;
      mem = MITS_RAM[pc];
      a = (a ^ mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (a & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x89: //adca imed
      pc++;
      mem = MITS_RAM[pc];
      if (ccr & 1) acc = a + mem + 1; else acc = a + mem;
      if (((~(a ^ mem)) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (((a & mem) | (mem & (~acc)) | ((~acc) & a)) & 0x08) ccr = ccr | 0b00100000; else ccr = ccr & 0b11011111; //test h          
      a = acc & 0xff;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x8A: //oraa immediate
      pc++;
      mem = MITS_RAM[pc];
      a = (a | mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (a & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x8B: //adda imed
      pc++;
      mem = MITS_RAM[pc];
      acc = a + mem;
      if (((~(a ^ mem)) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (((a & mem) | (mem & (~acc)) | ((~acc) & a)) & 0x08) ccr = ccr | 0b00100000; else ccr = ccr & 0b11011111; //test h          
      a = acc & 0xff;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x8C: //cpx immediate 
      pc++;
      mem = ((MITS_RAM[pc] << 8) + (MITS_RAM[pc+1] & 0xff))  & 0xffff;
      pc++;
      acc = (x - mem)  & 0xffff;
      if (((x & (~mem) & (~acc)) |((~x) & mem & acc)) & 0x8000) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~x) & mem) | (mem & acc) | (acc & (~x))) & 0x8000) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test c            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x8D: //bsr 
      pc++;
      acc = MITS_RAM[pc];
      if (acc > 127) acc = acc | 0xff00;
      pc++;     
      MITS_RAM[sp--] = pc & 0xff;
      MITS_RAM[sp--] = (pc >> 8) & 0xff;
      pc = pc + acc;
      break;     
     case 0x8E: //lds immediate 
      pc++;
      sp = ((MITS_RAM[pc] << 8) + (MITS_RAM[pc+1] & 0xff))  & 0xffff;
      pc++;
      ccr = ccr & 0b11111101; //clear v
      if (sp & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (sp==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x8F: //no instr
      pc++;
      break;     
     case 0x90: //suba direct
      pc++;
      mp = MITS_RAM[pc]  & 0xff;
      mem = MITS_RAM[mp];
      acc = (a - mem) & 0xff;
      if (((a ^ mem) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~a) & mem) | ((acc) & (~a)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      a = acc;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x91: //cmpa direct 
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = MITS_RAM[mp];
      acc = (a - mem) & 0xff;
      if (((a ^ mem) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~a) & mem) | ((acc) & (~a)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x92: //sbca direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = MITS_RAM[mp];
      if (ccr & 1) (acc = a - mem - 1) & 0xff; else acc = (a - mem) & 0xff;
      if (((a ^ mem) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~a) & mem) | ((acc) & (~a)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      a = acc;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x93: //no instr
      pc++;
      break;     
     case 0x94: //anda direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = MITS_RAM[mp];
      a = a & mem;
      ccr = ccr & 0b11111101; //clear v
      if (a & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x95: //bita direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = MITS_RAM[mp];
      acc = (a & mem)  & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x96: //ldaa direct 
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      a = MITS_RAM[mp] & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((a & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg
      pc++;
      break;
     case 0x97: //staa direct 
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((a & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg
      if (mp < ROM_START) MITS_RAM[mp] = a & 0xff;
      pc++;
      break;
     case 0x98: //eora direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = MITS_RAM[mp];
      a = (a ^ mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (a & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x99: //adca direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = MITS_RAM[mp];
      if (ccr & 1) acc = a + mem + 1; else acc = a + mem;
      if (((~(a ^ mem)) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (((a & mem) | (mem & (~acc)) | ((~acc) & a)) & 0x08) ccr = ccr | 0b00100000; else ccr = ccr & 0b11011111; //test h          
      a = acc & 0xff;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x9A: //oraa direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = MITS_RAM[mp];
      a = (a | mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (a & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x9B: //adda direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = MITS_RAM[mp];
      acc = a + mem;
      if (((~(a ^ mem)) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (((a & mem) | (mem & (~acc)) | ((~acc) & a)) & 0x08) ccr = ccr | 0b00100000; else ccr = ccr & 0b11011111; //test h          
      a = acc & 0xff;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x9C: //cpx direct 
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = ((MITS_RAM[mp] << 8) + (MITS_RAM[mp+1] & 0xff)) & 0xffff;
      acc = (x - mem)  & 0xffff;
      if (((x & (~mem) & (~acc)) |((~x) & mem & acc)) & 0x8000) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~x) & mem) | (mem & acc) | (acc & (~x))) & 0x8000) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test c            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x9D: //bsr direct not implemented
      pc++;
      break;     
     case 0x9E: //lds direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      sp = ((MITS_RAM[mp] << 8) + (MITS_RAM[mp+1] & 0xff)) & 0xffff;
      ccr = ccr & 0b11111101; //clear v
      if (sp & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (sp==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0x9F: //sts direct 
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      if (mp < ROM_START) MITS_RAM[mp++] = (sp >> 8) & 0xff;
      if (mp < ROM_START) MITS_RAM[mp] = sp & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (sp & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (sp==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xA0: //suba indexed
      pc++;
      mp = (MITS_RAM[pc] + x)  & 0xffff;
      mem = MITS_RAM[mp];
      acc = (a - mem) & 0xff;
      if (((a ^ mem) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~a) & mem) | ((acc) & (~a)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      a = acc;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xA1: //cmpa indexed 
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      acc = (a - mem) & 0xff;
      if (((a ^ mem) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~a) & mem) | ((acc) & (~a)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (acc == 0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xA2: //sbca indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      if (ccr & 1) acc = (a - mem - 1) & 0xff; else acc = (a - mem) & 0xff;
      if (((a ^ mem) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~a) & mem) | ((acc) & (~a)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      a = acc;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xA3: //no instr
      pc++;
      break;     
     case 0xA4: //anda indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      a = (a & mem)  & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (a & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xA5: //bita indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      acc = (a & mem)  & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xA6: //ldaa indexed
      pc++;
      mp = (MITS_RAM[pc] + x)  & 0xffff;
      a = MITS_RAM[mp] & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((a & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg
      pc++;
      break;
     case 0xA7: //staa indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      ccr = ccr & 0b11111101; //clear v
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((a & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg
      if (mp < ROM_START) MITS_RAM[mp] = a;
      if (mp == 0xf010) MITS_RAM[0xf010] = a;
      if (mp == 0xf020) adc_flag = a & 3;
      pc++;
      break;
     case 0xA8: //eora indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      a = (a ^ mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (a & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xA9: //adca indexed 
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      if (ccr & 1) acc = a + mem + 1; else acc = a + mem;
      if (((~(a ^ mem)) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (((a & mem) | (mem & (~acc)) | ((~acc) & a)) & 0x08) ccr = ccr | 0b00100000; else ccr = ccr & 0b11011111; //test h          
      a = acc & 0xff;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xAA: //oraa indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      a = (a | mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (a & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xAB: //adda indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      acc = a + mem;
      if (((~(a ^ mem)) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (((a & mem) | (mem & (~acc)) | ((~acc) & a)) & 0x08) ccr = ccr | 0b00100000; else ccr = ccr & 0b11011111; //test h          
      a = acc & 0xff;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xAC: //cpx indexed 
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = (MITS_RAM[mp] << 8) + (MITS_RAM[mp+1] & 0xff);
      acc = (x - mem) & 0xffff;
      if (((x & (~mem) & (~acc)) |((~x) & mem & acc)) & 0x8000) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~x) & mem) | (mem & acc) | (acc & (~x))) & 0x8000) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test c            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xAD: //jsr indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;   
      pc++;
      MITS_RAM[sp--] = pc & 0xff;
      MITS_RAM[sp--] = (pc >> 8) & 0xff;
      pc = mp;
      break;      
     case 0xAE: //lds indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      sp = ((MITS_RAM[mp] << 8) + (MITS_RAM[mp+1] & 0xff)) & 0xffff;
      ccr = ccr & 0b11111101; //clear v
      if (sp & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (sp==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xAF: //sts indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      MITS_RAM[mp++] = (sp >> 8) & 0xff;
      MITS_RAM[mp] = sp & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (sp & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (sp==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xB0: //suba extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      acc = (a - mem) & 0xff;
      if (((a ^ mem) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~a) & mem) | ((acc) & (~a)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      a = acc;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xB1: //cmpa extended 
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      acc = (a - mem) & 0xff;
      if (((a ^ mem) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~a) & mem) | ((acc) & (~a)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (acc & 0x01) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (acc == 0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xB2: //sbca extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      if (ccr & 1) acc = (a - mem - 1) & 0xff; else acc = (a - mem) & 0xff;
      if (((a ^ mem) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~a) & mem) | ((acc) & (~a)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      a = acc;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xB3: //no instr
      pc++;
      break;     
     case 0xB4: //anda extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      a = a & mem;
      ccr = ccr & 0b11111101; //clear v
      if (a & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xB5: //bitta extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      acc = (a & mem)  & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xB6: //lDAA extended **** uart
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      if (mp == 0xf003){
        uart_status = uart_status & 0b11111110;
        mem = uart_read;
      } else
      if (mp == 0xf000){
        mem = uart_status;
      } else mem = MITS_RAM[mp];
      a = mem & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((a & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg
      pc++;
      break;
     case 0xB7: //staa extended **** uart
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      ccr = ccr & 0b11111101; //clear v
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((a & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg
      if (mp < ROM_START) MITS_RAM[mp] = a;
      if (mp == 0xf010) MITS_RAM[0xf010] = a;
      if (mp == 0xf001) {
        uart_status = uart_status & 0b01111101; //was 0b11111101
        uart_write = a;
      };
      pc++;
      break;
     case 0xB8: //eora extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      a = (a ^ mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (a & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xB9: //adca extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      if (ccr & 1) acc = a + mem + 1; else acc = a + mem;
      if (((~(a ^ mem)) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (((a & mem) | (mem & (~acc)) | ((~acc) & a)) & 0x08) ccr = ccr | 0b00100000; else ccr = ccr & 0b11011111; //test h          
      a = acc & 0xff;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xBA: //oraa extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      a = (a | mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (a & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xBB: //adda extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      acc = a + mem;
      if (((~(a ^ mem)) & (a ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (((a & mem) | (mem & (~acc)) | ((~acc) & a)) & 0x08) ccr = ccr | 0b00100000; else ccr = ccr & 0b11011111; //test h          
      a = acc & 0xff;
      if (a==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xBC: //cpx extended 
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = ((MITS_RAM[mp] << 8) + (MITS_RAM[mp+1] & 0xff)) & 0xffff;
      acc = (x - mem)  & 0xffff;
      if (((x & (~mem) & (~acc)) |((~x) & mem & acc)) & 0x8000) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~x) & mem) | (mem & acc) | (acc & (~x))) & 0x8000) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test c            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xBD: //jsr extended 
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + (MITS_RAM[pc] & 0xff)) & 0xffff;
      pc++;
      MITS_RAM[sp--] = pc & 0xff;
      MITS_RAM[sp--] = (pc >> 8) & 0xff;
      pc = mp;
      break;      
     case 0xBE: //lds extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      sp = ((MITS_RAM[mp] << 8) + (MITS_RAM[mp+1] & 0xff)) & 0xffff;
      ccr = ccr & 0b11111101; //clear v
      if (sp & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (sp==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xBF: //sts extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      MITS_RAM[mp++] = (sp >> 8) & 0xff;
      MITS_RAM[mp] = sp & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (sp & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (sp==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xC0: //subb immediate
      pc++;
      mem = MITS_RAM[pc];
      acc = (b - mem) & 0xff;
      if (((b ^ mem) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~b) & mem) | ((acc) & (~b)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      b = acc;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xC1: //cmpb immediate 
      pc++;
      mem = MITS_RAM[pc];
      acc = (b - mem) & 0xff;
      if (((b ^ mem) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~b) & mem) | ((acc) & (~b)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xC2: //sbcb immediate
      pc++;
      mem = MITS_RAM[pc];
      if (ccr & 1) acc = (b - mem - 1) & 0xff; else acc = (b - mem) & 0xff;
      if (((b ^ mem) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~b) & mem) | ((acc) & (~b)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      b = acc;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xC3: //no instr
      pc++;
      break;     
     case 0xC4: //andb immediate
      pc++;
      mem = MITS_RAM[pc];
      b = (b & mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (b & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xC5: //tstb immediate
      pc++;
      mem = MITS_RAM[pc];
      acc = (b & mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xC6: //ldab imed
      pc++;
      b = MITS_RAM[pc];
      ccr = ccr & 0b11111101; //clear v
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((b & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg
      pc++;
      break;
     case 0xC7: //stab imed (not defined)
      pc++;
      break;
     case 0xC8: //eorb immediate
      pc++;
      mem = MITS_RAM[pc];
      b = (b ^ mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (b & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xC9: //adcb imed
      pc++;
      mem = MITS_RAM[pc];
      if (ccr & 1) acc = b + mem + 1; else acc = b + mem;
      if (((~(b ^ mem)) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (((b & mem) | (mem & (~acc)) | ((~acc) & b)) & 0x08) ccr = ccr | 0b00100000; else ccr = ccr & 0b11011111; //test h          
      b = acc & 0xff;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xCA: //orab immediate
      pc++;
      mem = MITS_RAM[pc];
      b = (b | mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (b & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xCB: //adda imed
      pc++;
      mem = MITS_RAM[pc];
      acc = b + mem;
      if (((~(b ^ mem)) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (((b & mem) | (mem & (~acc)) | ((~acc) & b)) & 0x08) ccr = ccr | 0b00100000; else ccr = ccr & 0b11011111; //test h          
      b = acc & 0xff;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xCC: //nop
      pc++;
      break;
     case 0xCD: //nop
      pc++;
      break;     
     case 0xCE: //ldx immediate 
      pc++;
      x = ((MITS_RAM[pc] << 8) + (MITS_RAM[pc+1] & 0xff)) & 0xffff;
      pc++;
      ccr = ccr & 0b11111101; //clear v
      if (x & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (x==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xCF: //no instr
      pc++;
      break;     
     case 0xD0: //subb direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = MITS_RAM[mp];
      acc = (b - mem) & 0xff;
      if (((b ^ mem) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~b) & mem) | ((acc) & (~b)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      b = acc;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xD1: //cmpb direct 
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = MITS_RAM[mp];
      acc = (b - mem) & 0xff;
      if (((b ^ mem) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~b) & mem) | ((acc) & (~b)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xD2: //sbcb direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = MITS_RAM[mp];
      if (ccr & 1) acc = (b - mem - 1) & 0xff; else acc = (b - mem) & 0xff;
      if (((b ^ mem) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~b) & mem) | ((acc) & (~b)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      b = acc;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xD3: //no instr
      pc++;
      break;     
     case 0xD4: //andb direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = MITS_RAM[mp];
      b = (b & mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (b & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xD5: //bitb direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = MITS_RAM[mp];
      acc = (b & mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xD6: //ldab direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      b = MITS_RAM[mp] & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((b & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg
      pc++;
      break;
     case 0xD7: //stab direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((b & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg
      if (mp < ROM_START) MITS_RAM[mp] = b & 0xff;
      pc++;
      break;
     case 0xD8: //eorb direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = MITS_RAM[mp];
      b = (b ^ mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (b & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xD9: //adcb direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = MITS_RAM[mp];
      if (ccr & 1) acc = b + mem + 1; else acc = b + mem;
      if (((~(b ^ mem)) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (((b & mem) | (mem & (~acc)) | ((~acc) & b)) & 0x08) ccr = ccr | 0b00100000; else ccr = ccr & 0b11011111; //test h          
      b = acc & 0xff;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xDA: //orab direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = MITS_RAM[mp];
      b = (b | mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (b & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xDB: //addb direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      mem = MITS_RAM[mp];
      acc = b + mem;
      if (((~(b ^ mem)) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (((b & mem) | (mem & (~acc)) | ((~acc) & b)) & 0x08) ccr = ccr | 0b00100000; else ccr = ccr & 0b11011111; //test h          
      b = acc & 0xff;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xDC: //nop
      pc++;
      break;
     case 0xDD: //nop
      pc++;
      break;     
     case 0xDE: //ldx direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      x = ((MITS_RAM[mp] << 8) + (MITS_RAM[mp+1] & 0xff)) & 0xffff;
      ccr = ccr & 0b11111101; //clear v
      if (x & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (x==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xDF: //stx direct
      pc++;
      mp = MITS_RAM[pc] & 0xff;
      MITS_RAM[mp++] = (x >> 8) & 0xff;
      MITS_RAM[mp] = x & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (x & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (x==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xE0: //subb indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      acc = (b - mem) & 0xff;
      if (((b ^ mem) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~b) & mem) | ((acc) & (~b)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      b = acc;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xE1: //cmpb indexed 
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      acc = (b - mem) & 0xff;
      if (((b ^ mem) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~b) & mem) | ((acc) & (~b)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xE2: //sbcb indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      if (ccr & 1) acc = (b - mem - 1) & 0xff; else acc = (b - mem) & 0xff;
      if (((b ^ mem) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~b) & mem) | ((acc) & (~b)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      b = acc;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xE3: //no instr
      pc++;
      break;     
     case 0xE4: //andb indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      b = (b & mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (b & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xE5: //bitb indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      acc = (b & mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xE6: //ldab indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      b = MITS_RAM[mp];
      ccr = ccr & 0b11111101; //clear v
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((b & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg
      pc++;
      break;
     case 0xE7: //stab indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      ccr = ccr & 0b11111101; //clear v
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((b & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg
      if (mp < ROM_START) MITS_RAM[mp] = b;
      if (mp == 0xf010) MITS_RAM[0xf010] = b;
      if (mp == 0xf020) adc_flag = b & 3;
      pc++;
      break;
     case 0xE8: //eorb indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      b = (b ^ mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (b & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xE9: //adcb indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      if (ccr & 1) acc = b + mem + 1; else acc = b + mem;
      if (((~(b ^ mem)) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (((b & mem) | (mem & (~acc)) | ((~acc) & b)) & 0x08) ccr = ccr | 0b00100000; else ccr = ccr & 0b11011111; //test h          
      b = acc & 0xff;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xEA: //orab indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      b = (b | mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (b & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xEB: //addb indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      mem = MITS_RAM[mp];
      acc = b + mem;
      if (((~(b ^ mem)) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (((b & mem) | (mem & (~acc)) | ((~acc) & b)) & 0x08) ccr = ccr | 0b00100000; else ccr = ccr & 0b11011111; //test h          
      b = acc & 0xff;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xEC: //nop
      pc++;
      break;
     case 0xED: //nop
      pc++;
      break;      
     case 0xEE: //ldx indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      x = ((MITS_RAM[mp] << 8) + (MITS_RAM[mp+1] & 0xff)) & 0xffff;
      ccr = ccr & 0b11111101; //clear v
      if (x & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (x==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xEF: //stx indexed
      pc++;
      mp = (MITS_RAM[pc] + x) & 0xffff;
      MITS_RAM[mp++] = (x >> 8) & 0xff;
      MITS_RAM[mp] = x & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (x & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (x==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xF0: //subb extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      acc = (b - mem) & 0xff;
      if (((b ^ mem) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~b) & mem) | ((acc) & (~b)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      b = acc;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xF1: //cmpb extended 
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      acc = (b - mem) & 0xff;
      if (((b ^ mem) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ( (((~b) & mem) | ((acc) & (~b)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xF2: //sbcb extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      if (ccr & 1) acc = (b - mem - 1) & 0xff; else acc = (b - mem) & 0xff;
      if (((b ^ mem) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if ((((~b) & mem) | ((acc) & (~b)) | (mem & acc)) & 0x80) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      b = acc;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xF3: //no instr
      pc++;
      break;     
     case 0xF4: //andb extended **** uart
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      if (mp == 0xf003){
        uart_status = uart_status & 0b11111110;
        mem = uart_read;
      } else mem = MITS_RAM[mp];
      b = (b & mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (b & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xF5: //bitb extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      acc = (b & mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xF6: //ldab extended **** uart
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      if (mp == 0xf003){
        uart_status = uart_status & 0b11111110;
        mem = uart_read;
      } else
      if (mp == 0xf000){
        mem = uart_status;
      } else mem = MITS_RAM[mp];
      b = mem & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((b & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg
      pc++;
      break;
     case 0xF7: //stab extended **** uart, I/O, adc
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      ccr = ccr & 0b11111101; //clear v
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      if ((b & 0x80) == 0) ccr = ccr & 0b11110111; else ccr = ccr | 0b00001000; //test neg
      if (mp < ROM_START) MITS_RAM[mp] = b;
      if (mp == 0xf001) {
        uart_status = uart_status & 0b01111101; //was 0b11111101
        uart_write = b;
      };
      if (mp == 0xf010) MITS_RAM[0xf010] = b;
      if (mp == 0xf020) adc_flag = b & 3;
      pc++;
      break;
     case 0xF8: //eorb extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      b = (b ^ mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (b & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xF9: //adcb extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      if (ccr & 1) acc = b + mem + 1; else acc = b + mem;
      if (((~(b ^ mem)) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (((b & mem) | (mem & (~acc)) | ((~acc) & b)) & 0x08) ccr = ccr | 0b00100000; else ccr = ccr & 0b11011111; //test h          
      b = acc & 0xff;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xFA: //orab extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      b = (b | mem) & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (b & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xFB: //addb extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      mem = MITS_RAM[mp];
      acc = b + mem;
      if (((~(b ^ mem)) & (b ^ acc)) & 0x80) ccr = ccr | 0b00000010; else ccr = ccr & 0b11111101; //test v
      if (acc & 0x80) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (acc & 0x100) ccr = ccr | 0b00000001; else ccr = ccr & 0b11111110; //test carry            
      if (((b & mem) | (mem & (~acc)) | ((~acc) & b)) & 0x08) ccr = ccr | 0b00100000; else ccr = ccr & 0b11011111; //test h          
      b = acc & 0xff;
      if (b==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xFC: //nop
      pc++;
      break;
     case 0xFD: //nop
      pc++;
      break;      
     case 0xFE: //ldx extended
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      x = ((MITS_RAM[mp] << 8) + (MITS_RAM[mp+1] & 0xff)) & 0xffff;
      ccr = ccr & 0b11111101; //clear v
      if (x & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (x==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
     case 0xFF: //stx extended 
      pc++;
      mp = MITS_RAM[pc] << 8;
      pc++;
      mp = (mp + MITS_RAM[pc]) & 0xffff;
      MITS_RAM[mp++] = (x >> 8) & 0xff;
      MITS_RAM[mp] = x & 0xff;
      ccr = ccr & 0b11111101; //clear v
      if (x & 0x8000) ccr = ccr | 0b00001000; else ccr = ccr & 0b11110111; //test neg            
      if (x==0) ccr = ccr | 0b00000100; else ccr = ccr & 0b11111011; //test zero
      pc++;
      break;
    default:
      break;  
  } 

 inpins =  sio_hw->gpio_in;
 MITS_RAM[0xF011] = 0xff & (inpins>>20); // D20 to D27
 xbyte = ((MITS_RAM[0xF010] << 8) ^ inpins) & 0x0000C000; //was ff00

 mod_uart++;
 if ((mod_uart % 200) == 0) { 
   if ((uart_status & 1) == 0){ 
     if (Serial.available() != 0){ 
       uart_status = uart_status | 0x01; // set received data bit
       uart_read = Serial.read() & 0x00ff; 
     }
     if (Serial1.available() != 0){ 
       uart_status = uart_status | 0x01; // set received data bit
       uart_read = Serial1.read() & 0x00ff; 
     }
   } 
 } 
 if ((uart_status & 0x02)==0){  
    Serial.write(uart_write & 0x00ff);
    Serial1.write(uart_write & 0x00ff);
    uart_status = uart_status | 0x82;  // set Transmit Data Register Empty //was 0x02
 }  


 
 gpio_xor_mask(xbyte);   

 if (adc_flag != 0){
  switch (adc_flag) {
   case 1: adc_val = analogRead(A0);
   break;
   case 2: adc_val = analogRead(A1);
   break;
   case 3: adc_val = analogRead(A2);
   break;   
  }
  MITS_RAM[0xF020] = (adc_val >> 8) & 0x0f;
  MITS_RAM[0xF021] = adc_val & 0xff;
  adc_flag = 0;
 }
 
// gpio_clr_mask(1ul << LOOP_P); 
// gpio_xor_mask(1ul << LOOP_P);

if ((inpins & 0x20000) == 0)  trace = trace | 2; else trace = trace & 0b11111101; ////special test
if (trace) {

 Serial.print(" a");
 Serial1.print(" a");
 print2hex(a);
 Serial.print(" b");
 Serial1.print(" b");
 print2hex(b);
 Serial.print(" x");
 Serial1.print(" x");
 print4hex(x);
 Serial.print(" sp");
 Serial1.print(" sp");
 print4hex(sp);
 Serial.print(" ");
 if (ccr & 0b00100000) Serial.print('H'); else Serial.print('-');
 if (ccr & 0b00010000) Serial.print('I'); else Serial.print('-');
 if (ccr & 0b00001000) Serial.print('N'); else Serial.print('-');
 if (ccr & 0b00000100) Serial.print('Z'); else Serial.print('-');
 if (ccr & 0b00000010) Serial.print('V'); else Serial.print('-');
 if (ccr & 0b00000001) Serial.print('C'); else Serial.print('-');
 Serial1.print(" ");
 if (ccr & 0b00100000) Serial1.print('H'); else Serial1.print('-');
 if (ccr & 0b00010000) Serial1.print('I'); else Serial1.print('-');
 if (ccr & 0b00001000) Serial1.print('N'); else Serial1.print('-');
 if (ccr & 0b00000100) Serial1.print('Z'); else Serial1.print('-');
 if (ccr & 0b00000010) Serial1.print('V'); else Serial1.print('-');
 if (ccr & 0b00000001) Serial1.print('C'); else Serial1.print('-');
 Serial.print(" acc");
 Serial1.print(" acc");
 print4hex(acc);
 Serial.print(" mp");
 Serial1.print(" mp");
 print4hex(mp);
 Serial.print(" mem");
 Serial1.print(" mem");
 print4hex(mem);
 Serial.println();
 Serial1.println();
 delay(100);
 }

}  

}

void print4hex(uint16_t hx4){
  print2hex((hx4 >> 8) & 0xff);
  print2hex(hx4 & 0xff);  
}
void print2hex(uint16_t hx2){
  unsigned char ch, hc;
  hc = (hx2 >> 4) & 0x0f;
  if (hc < 0x0a) hc = hc + '0'; else hc = hc + '7';
  Serial.write(hc);  
  Serial1.write(hc);  
  ch = hx2 & 0x0f;
  if (ch < 0x0a) ch = ch + '0'; else ch = ch + '7';
  Serial.write(ch);  
  Serial1.write(ch);  
}

void decimal_table(void){
  unsigned int j;
  for (j = 0; j < 1024; j++) {
    if (j < 512){   //carry bit-9, half carry bit-8
      if (((j & 0xf0) <= 0x90) && ((j & 0x0f) <= 0x09) && ((j & 0x100)==0)) DAA[j] = j & 0xff;
      if (((j & 0xf0) <= 0x80) && ((j & 0x0f) >= 0x0A) && ((j & 0x100)==0)) DAA[j] = (j + 0x06) & 0xff;
      if (((j & 0xf0) <= 0x90) && ((j & 0x0f) <= 0x03) && ((j & 0x100))) DAA[j] = (j + 0x06) & 0xff;

      if (((j & 0xf0) >= 0xA0) && ((j & 0x0f) <= 0x09) && ((j & 0x100)==0)) DAA[j] = ((j + 0x60) & 0xff) | 0x100;
      if (((j & 0xf0) >= 0x90)  && ((j & 0x0f) >= 0x0A) && ((j & 0x100)==0)) DAA[j] = ((j + 0x66) & 0xff) | 0x100;
      if (((j & 0xf0) >= 0xA0)  && ((j & 0x0f) <= 0x03) && ((j & 0x100))) DAA[j] = ((j + 0x66) & 0xff) | 0x100;

    }
    else {
      if (((j & 0xf0) <= 0x20) && ((j & 0x0f) <= 0x09) && ((j & 0x100)==0)) DAA[j] = ((j + 0x60) & 0xff) | 0x100;
      if (((j & 0xf0) <= 0x20)  && ((j & 0x0f) >= 0x0A) && ((j & 0x100)==0)) DAA[j] = ((j + 0x66) & 0xff) | 0x100;
      if (((j & 0xf0) <= 0x30)  && ((j & 0x0f) <= 0x03) && ((j & 0x100))) DAA[j] = ((j + 0x66) & 0xff) | 0x100;
      
    }
  }
}


void setup1() { 
   //D0 is UART TX out
   //D1 is UART RX in
   pinMode(2, INPUT_PULLUP);  //key bit 0
   pinMode(3, INPUT_PULLUP);  //key bit 1
   pinMode(4, INPUT_PULLUP);  //key bit 2
   pinMode(5, INPUT_PULLUP);  //key bit 3
   pinMode(6, INPUT_PULLUP);  //key bit 4
   pinMode(7, INPUT_PULLUP);  //key bit 5
   pinMode(8, INPUT_PULLUP);  //key address 0
   pinMode(9,INPUT_PULLUP);   //key address 1
   pinMode(10, INPUT_PULLUP); //key address 2
   pinMode(11, OUTPUT); //clock    old-*CS
   pinMode(12, INPUT_PULLUP); //*CS      old-clock
   pinMode(13, INPUT_PULLUP); //data
   pinMode(25, OUTPUT); //LED
   delay(2000);

   Wire.setSDA(12);
   Wire.setSCL(13);
   Wire.begin();
   display.begin(SSD1306_EXTERNALVCC, SCREEN_ADDRESS);
   delay(500);
   display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
   display.display();
   delay(4000); // Pause for 4 seconds

   // Clear the buffer
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font   
}

void loop1() {
 volatile int i,j,k,m;
 volatile unsigned int k1,k2,k3;
 volatile unsigned int key = 0xFF;
 volatile unsigned int keyold = 0xFF;
 volatile unsigned int key2 = 0;

 MITS_RAM[0xc000]=0xFF; //FF flags no key pressed
 while (true){ 

 update_display();
 pinMode(10, OUTPUT); //0 1 4 7 A D
 gpio_put(10,0); 
 delayMicroseconds(500);
 k1 =  ((sio_hw->gpio_in) >> 2) & 0b111111;
 MITS_RAM[0xc006] = k1;
 pinMode(10, INPUT_PULLUP);

 update_display();
 pinMode(9, OUTPUT); //20 2 5 8 B E
 gpio_put(9,0); 
 delayMicroseconds(500);
 k2 =  ((sio_hw->gpio_in) >> 2) & 0b111111;
 MITS_RAM[0xc005] = k2;
 pinMode(9, INPUT_PULLUP);

 update_display();
 pinMode(8, OUTPUT); //10 3 6 9 C F
 gpio_put(8,0); 
 delayMicroseconds(500);
 k3 =  ((sio_hw->gpio_in) >> 2) & 0b111111;
 MITS_RAM[0xc003] = k3;
 pinMode(8, INPUT_PULLUP);

 switch (k1) {
  case 0b111110:
    key = 0x0D;
    break;
  case 0b111101:
    key = 0x0A;
    break;
  case 0b111011:
    key = 0x07;
    break;
  case 0b110111:
    key = 0x04;
    break;
  case 0b001111:
      key = 0x10;
    break;    
  case 0b101111:
    if (key != 0x10) key = 0x01;
    break;
  case 0b011111:
    if (key != 0x10) key = 0x00;
    break;
  default:
    key = 0xFF;
    break;
 }    

 if (key==0xFF) {
  switch (k2) {
  case 0b111110:
    key = 0x0E;
    break;
  case 0b111101:
    key = 0x0B;
    break;
  case 0b111011:
    key = 0x08;
    break;
  case 0b110111:
    key = 0x05;
    break;
  case 0b101111:
    key = 0x02;
    break;
  case 0b011111:
    key = 0x20;
    break;    
  default:
    key = 0xFF;
    break;
  }    
 }

 if (key==0xFF) {
   switch (k3) {
  case 0b111110:
    key = 0x0F;
    break;
  case 0b111101:
    key = 0x0C;
    break;
  case 0b111011:
    key = 0x09;
    break;
  case 0b110111:
    key = 0x06;
    break;
  case 0b101111:
    key = 0x03;
    break;
  case 0b011111:
    key = 0x10;
    break;    
  default:
    key = 0xFF;
    break;
  }
 }

  if (keyold != key){
   MITS_RAM[0xc000] = key;
   keyold = key;
  }
 }
}

void update_display(void){
 volatile int i,j,k,m,f;

 f = MITS_RAM[0xc109];
 
 for (i=0; i<=7; i++){
  k = MITS_RAM[i+0xc100];
  m = 0xC100 + ((8-i)<<4);
  if (k !=0){
    display.fillRect(16*i, 0, 16, 16, SSD1306_BLACK); //clear top line
    display.setCursor(16*i, 0);
    if ((f & 0x01) == 0){  //LSB disables 7-segment ascii write
     for (j=0; j<=7;j++){
       MITS_RAM[m+j] = (ascii[k & 0x7f] >> j) & 1;  //USE IF 7-SEG DISPLAY ASCII ON
     }
     if ((k & 0x80) != 0) MITS_RAM[m+7] = 1; else MITS_RAM[m+7] = 0;
    }
    if ((f & 0x80) == 0){     //MSB set disables line-1 text out
     display.write(k & 0x7f);
     if ((k & 0x80) != 0) display.drawPixel(16*i+12, 15, SSD1306_WHITE);
    } 
    MITS_RAM[i+0xc100] = 0;
  }
    //process decimal point from 7-segment write
    if ((MITS_RAM[m+7] != 0)&&((f & 0x80) == 0)) display.drawPixel(16*i+12, 15, SSD1306_WHITE);
 }
 display.fillRect(0, 18, 128, 14, SSD1306_BLACK); //clear 7-seg line
 for (j=1;j<=8;j++){
   k = 8-j;
   f = k*16;
   for (i=0; i<=7; i++){
        if ((MITS_RAM[i+0xc100+(j<<4)] & 1)!=0) {
        switch(i){
          case 0: display.drawFastHLine(f+2, 24, 7, SSD1306_WHITE);
          break;
          case 1: display.drawFastVLine(f, 18, 5, SSD1306_WHITE);
          break;
          case 2: display.drawFastVLine(f, 24, 6,  SSD1306_WHITE);
          break;
          case 3: display.drawFastHLine(f+2, 30, 7, SSD1306_WHITE);
          break;
          case 4: display.drawFastVLine(f+10, 24, 6, SSD1306_WHITE);
          break;
          case 5: display.drawFastVLine(f+10, 18,  5, SSD1306_WHITE);
          break;
          case 6: display.drawFastHLine(f+2, 18, 7, SSD1306_WHITE);
          break;
          case 7: display.drawPixel(f+12, 31, SSD1306_WHITE);
          break;
          default:
          break;
        } 
      }
   }  
 }
 display.display();
 if (MITS_RAM[0xC10A] != 0){
  MITS_RAM[0xC10A] = 0;
  for (i=0xC110; i<= 0xC187; i++) MITS_RAM[i] = 0;
 }
 if (MITS_RAM[0xC108] != 0) gpio_put(25,1); else gpio_put(25,0); //process LED out
}
