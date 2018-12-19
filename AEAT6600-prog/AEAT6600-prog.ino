// read/write programming data from an AEAT6600-T16  16 bit magnetic rotation sensor
// dan@marginallyclever.com 2018-12-07
// Arduino Mega

// sensor bits, flags, and masks
#define SENSOR_TOTAL_BITS    (16)  // 18 bits of data
#define SENSOR_ANGLE_BITS    (16)
#define SENSOR_ANGLE_PER_BIT (360.0/(float)((long)1<<SENSOR_ANGLE_BITS))  // 0.00549316406

// pins for sensor
#define PIN_SENSOR_CSEL   24
#define PIN_SENSOR_CLK    26
#define PIN_SENSOR_D0     28
#define PIN_SENSOR_MAGL   43
#define PIN_SENSOR_MAGH   45
#define PIN_SENSOR_PROG   34

#define PIN_OTP_ERR       PIN_SENSOR_MAGH
#define PIN_OTP_PROG_STAT PIN_SENSOR_MAGL


void setup() {
  Serial.begin(57600);
  // put your setup code here, to run once:
  pinMode(PIN_SENSOR_CLK, OUTPUT);
  pinMode(PIN_SENSOR_CSEL, OUTPUT);

  pinMode(PIN_OTP_ERR, INPUT);
  pinMode(PIN_OTP_PROG_STAT, INPUT);
  pinMode(PIN_SENSOR_PROG, OUTPUT);

  OTP_write();
  OTP_read();
}


void loop() {}


void OTP_write() {
  // set 16 bit mode in absolute resolution, bits 9:8
  digitalWrite(PIN_SENSOR_PROG, HIGH);
  digitalWrite(PIN_SENSOR_CSEL, LOW);
  digitalWrite(PIN_SENSOR_CLK, LOW);
  pinMode(PIN_SENSOR_D0, OUTPUT);
  long data = 0;
  data |= 1 << 9;
  data |= 1 << 8;

  int parity = 0;
  Serial.println();
  for (int i = 0; i < 32; ++i) {
    int oneBit = (data >> 31) & 0x1;
    digitalWrite(PIN_SENSOR_CLK, HIGH);
    digitalWrite(PIN_SENSOR_D0, oneBit);
    digitalWrite(PIN_SENSOR_CLK, LOW);
    Serial.print(oneBit == 1 ? '1' : '0');
    data <<= 1;
    parity ^= oneBit;
  }
  Serial.print(' ');
  Serial.print(parity == 1 ? '1' : '0');

  int err = digitalRead(PIN_OTP_ERR);
  int stat = digitalRead(PIN_OTP_PROG_STAT);
  Serial.print(' ');
  Serial.print(err == 1 ? '1' : '0');
  Serial.print(stat == 1 ? '1' : '0');

  Serial.println();
}

void OTP_read() {
  digitalWrite(PIN_SENSOR_PROG, HIGH);
  digitalWrite(PIN_SENSOR_CSEL, LOW);
  digitalWrite(PIN_SENSOR_CLK, LOW);
  pinMode(PIN_SENSOR_D0, INPUT);

  byte zero_hi = OTP_read_byte();
  byte zero_lo = OTP_read_byte();
  byte mode_hi = OTP_read_byte();
  byte mode_lo = OTP_read_byte();

  Serial.println();

  printBit(mode_lo, 7);
  printBit(mode_lo, 6);
  printBit(mode_lo, 5);
  printBit(mode_lo, 4);
  printBit(mode_lo, 3);
  printBit(mode_lo, 2);
  printBit(mode_lo, 1);
  printBit(mode_lo, 0);  Serial.println(" mode_lo");
  printBit(mode_hi, 7);
  printBit(mode_hi, 6);
  printBit(mode_hi, 5);
  printBit(mode_hi, 4);
  printBit(mode_hi, 3);
  printBit(mode_hi, 2);
  printBit(mode_hi, 1);
  printBit(mode_hi, 0);  Serial.println(" mode_hi");

  printBit(zero_lo, 7);
  printBit(zero_lo, 6);
  printBit(zero_lo, 5);
  printBit(zero_lo, 4);
  printBit(zero_lo, 3);
  printBit(zero_lo, 2);
  printBit(zero_lo, 1);
  printBit(zero_lo, 0);  Serial.println(" zero_lo");
  printBit(zero_hi, 7);
  printBit(zero_hi, 6);
  printBit(zero_hi, 5);
  printBit(zero_hi, 4);
  printBit(zero_hi, 3);
  printBit(zero_hi, 2);
  printBit(zero_hi, 1);
  printBit(zero_hi, 0);  Serial.println(" zero_hi");


}

byte OTP_read_byte() {
  byte data = 0;
  for (int i = 0; i < 8; ++i) {
    digitalWrite(PIN_SENSOR_CLK, HIGH);
    digitalWrite(PIN_SENSOR_CLK, LOW);
    byte input = digitalRead(PIN_SENSOR_D0);
    data <<= 1;
    data |= input;
  }
  return data;
}

void printBit(long data, int bitNumber) {
  Serial.print((data & (1 << bitNumber)) == 1 ? '1' : '0');
}
