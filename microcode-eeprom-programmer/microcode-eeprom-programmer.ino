/**
 * This sketch programs the microcode EEPROMs for the 8-bit breadboard computer
 * See this video for more: https://youtu.be/JUVt_KYAp-I
 */
#define SHIFT_DATA 2
#define SHIFT_CLK 3
#define SHIFT_LATCH 4
#define EEPROM_D0 5
#define EEPROM_D7 12
#define WRITE_EN 13
// WORD       [unique][ in   ][  out ][jumps ]
// WORD       01234567012345670123456701234567

// Main
#define HLT 0b10000000000000000000000000000000  // Halt clock

// Instructions
#define PCO 0b00000000000000000000000000000000  // Program counter out 			(active low)
#define PCE 0b00000000000000000000000000000000  // Program counter enable
#define MI  0b00000000000000000000000000000000  // Memory address register in 	(active low)
#define LI  0b00000000000000000000000000000000  // Load instruction 			(active low)
#define II  0b00000000000000000000000000000000  // Instruction register in 		(active low)
#define IO  0b00000000000000000000000000000000  // Instruction register out 	(active low)
#define SI  0b00000000000000000000000000000000  // Stack pointer in 			(active low)
#define SO  0b00000000000000000000000000000000  // Stack pointer out 			(active low)

// Interrupts
#define STI 0b00000000000000000000000000000000  // Set interrupt flag
#define CLI 0b00000000000000000000000000000000  // Clear interrupt flag

// ALU
#define EO  0b00000000000000000000000000000000  // ALU out 						(active low)
#define SU  0b00000000000000000000000000000000  // ALU subtract
#define AND 0b00000000000000000000000000000000  // AND result out 				(active low)
#define OR  0b00000000000000000000000000000000  // OR result out 				(active low)
#define XOR 0b00000000000000000000000000000000  // XOR result out 				(active low)
#define FI  0b00000000000000000000000000000000  // Flags in 					(active low)

// Registers
#define AI  0b00000000000000000000000000000000  // A register in 				(active low)
#define BI  0b00000000000000000000000000000000  // B register in				(active low)
#define CI  0b00000000000000000000000000000000  // C register in 				(active low)
#define AO  0b00000000000000000000000000000000  // A register out 				(active low)
#define BO  0b00000000000000000000000000000000  // B register out 				(active low)
#define CO  0b00000000000000000000000000000000  // C register out 				(active low)
#define RND 0b00000000000000000000000000000000  // Random register out 			(active low)

// RAM
#define RI  0b00000000000000000000000000000000  // RAM data in
#define RO  0b00000000000000000000000000000000  // RAM data out 				(active low)

// Output
#define OI  0b00000000000000000000000000000000  // Output register in 			(active low)
#define DI  0b00000000000000000000000000000000  // LCD display in
#define DSI 0b00000000000000000000000000000000  // LCD display set instruction

// Jumps
#define J   0b00000000000000000000000000000000  // Jump (program counter in)
#define JC  0b00000000000000000000000000000000  // Jump Carry
#define JZ  0b00000000000000000000000000000000  // Jump Zero
#define JO  0b00000000000000000000000000000000  // Jump Overflow
#define JS  0b00000000000000000000000000000000  // Jump Sign
#define JG  0b00000000000000000000000000000000  // Jump Greater
#define JL  0b00000000000000000000000000000000  // Jump Less
#define JE  0b00000000000000000000000000000000  // Jump Equal

uint16_t data[] = {
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0, 0,   // 0000 - NOP
  MI|CO,  RO|II|CE,  IO|MI,  RO|AI,  0,         0, 0, 0,   // 0001 - LDA
  MI|CO,  RO|II|CE,  IO|MI,  RO|BI,  EO|AI,     0, 0, 0,   // 0010 - ADD
  MI|CO,  RO|II|CE,  IO|MI,  RO|BI,  EO|AI|SU,  0, 0, 0,   // 0011 - SUB
  MI|CO,  RO|II|CE,  IO|MI,  AO|RI,  0,         0, 0, 0,   // 0100 - STA
  MI|CO,  RO|II|CE,  IO|AI,  0,      0,         0, 0, 0,   // 0101 - LDI
  MI|CO,  RO|II|CE,  IO|J,   0,      0,         0, 0, 0,   // 0110 - JMP
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0, 0,   // 0111
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0, 0,   // 1000
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0, 0,   // 1001
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0, 0,   // 1010
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0, 0,   // 1011
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0, 0,   // 1100
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0, 0,   // 1101
  MI|CO,  RO|II|CE,  AO|OI,  0,      0,         0, 0, 0,   // 1110 - OUT
  MI|CO,  RO|II|CE,  HLT,    0,      0,         0, 0, 0,   // 1111 - HLT
};


/*
 * Output the address bits and outputEnable signal using shift registers.
 */
void setAddress(int address, bool outputEnable) {
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, (address >> 8) | (outputEnable ? 0x00 : 0x80));
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, address);

  digitalWrite(SHIFT_LATCH, LOW);
  digitalWrite(SHIFT_LATCH, HIGH);
  digitalWrite(SHIFT_LATCH, LOW);
}


/*
 * Read a byte from the EEPROM at the specified address.
 */
byte readEEPROM(int address) {
  for (int pin = EEPROM_D0; pin <= EEPROM_D7; pin += 1) {
    pinMode(pin, INPUT);
  }
  setAddress(address, /*outputEnable*/ true);

  byte data = 0;
  for (int pin = EEPROM_D7; pin >= EEPROM_D0; pin -= 1) {
    data = (data << 1) + digitalRead(pin);
  }
  return data;
}


/*
 * Write a byte to the EEPROM at the specified address.
 */
void writeEEPROM(int address, byte data) {
  setAddress(address, /*outputEnable*/ false);
  for (int pin = EEPROM_D0; pin <= EEPROM_D7; pin += 1) {
    pinMode(pin, OUTPUT);
  }

  for (int pin = EEPROM_D0; pin <= EEPROM_D7; pin += 1) {
    digitalWrite(pin, data & 1);
    data = data >> 1;
  }
  digitalWrite(WRITE_EN, LOW);
  delayMicroseconds(1);
  digitalWrite(WRITE_EN, HIGH);
  delay(10);
}


/*
 * Read the contents of the EEPROM and print them to the serial monitor.
 */
void printContents() {
  for (int base = 0; base <= 255; base += 16) {
    byte data[16];
    for (int offset = 0; offset <= 15; offset += 1) {
      data[offset] = readEEPROM(base + offset);
    }

    char buf[80];
    sprintf(buf, "%03x:  %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x %02x %02x %02x %02x %02x %02x",
            base, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
            data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);

    Serial.println(buf);
  }
}


void setup() {
  // put your setup code here, to run once:
  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_CLK, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);
  digitalWrite(WRITE_EN, HIGH);
  pinMode(WRITE_EN, OUTPUT);
  Serial.begin(57600);

  // Program data bytes
  Serial.print("Programming EEPROM");

  // Program the 8 high-order bits of microcode into the first 128 bytes of EEPROM
  for (int address = 0; address < sizeof(data)/sizeof(data[0]); address += 1) {
    writeEEPROM(address, data[address] >> 8);

    if (address % 64 == 0) {
      Serial.print(".");
    }
  }

  // Program the 8 low-order bits of microcode into the second 128 bytes of EEPROM
  for (int address = 0; address < sizeof(data)/sizeof(data[0]); address += 1) {
    writeEEPROM(address + 128, data[address]);

    if (address % 64 == 0) {
      Serial.print(".");
    }
  }

  Serial.println(" done");


  // Read and print out the contents of the EERPROM
  Serial.println("Reading EEPROM");
  printContents();
}


void loop() {
  // put your main code here, to run repeatedly:

}

