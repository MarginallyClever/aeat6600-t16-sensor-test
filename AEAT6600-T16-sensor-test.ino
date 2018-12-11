// visualizing data from an AEAT6600-T16  16 bit magnetic rotation sensor
// dan@marginallyclever.com 2018-12-07
// Arduino Mega

// sensor bits, flags, and masks
#define SENSOR_TOTAL_BITS    (16)  // 18 bits of data
#define SENSOR_ANGLE_BITS    (16)
#define SENSOR_ANGLE_PER_BIT (360.0/(float)((long)1<<SENSOR_ANGLE_BITS))

// pins for sensor
#define PIN_SENSOR_CSEL   24
#define PIN_SENSOR_CLK    26
#define PIN_SENSOR_SDOUT  28
#define PIN_MAG_LOW       43
#define PIN_MAG_HIGH      45

float previousAngle;

void setup() {
  Serial.begin(57600);
  // put your setup code here, to run once:
  pinMode(PIN_SENSOR_CLK, OUTPUT);
  pinMode(PIN_SENSOR_CSEL, OUTPUT);
  pinMode(PIN_SENSOR_SDOUT, INPUT);
  pinMode(PIN_MAG_LOW, INPUT);
  pinMode(PIN_MAG_HIGH, INPUT);

  
  digitalWrite(PIN_SENSOR_CLK, HIGH);
  digitalWrite(PIN_SENSOR_CSEL, LOW);
  
  previousAngle = -1;
}


uint32_t previousD = 0;

void loop() {
  uint32_t d = sensor_update();
  //if(previousD!=d)
  {
    previousD = d;
    float currentAngle = sensor_angle(d);
    //if(previousAngle != currentAngle)
    {
      previousAngle = currentAngle;
      printBinary(16,d);
      //Serial.print(d,BIN);
      Serial.print('\t');
      Serial.print(digitalRead(PIN_MAG_LOW)==HIGH?'1':'0');
      Serial.print(digitalRead(PIN_MAG_HIGH)==HIGH?'1':'0');
      Serial.print('\t');
      Serial.print(currentAngle);
    }
    Serial.print('\n');
  }
  delay(50);
}


void printBinary(int maskSize,long data) {
  long mask = (long)1<<(maskSize-1);
  while(mask>0) {
    Serial.print( (mask&data) ? '1':'0' );
    mask>>=1;
  }
}


// from http://www.madscientisthut.com/forum_php/viewtopic.php?f=11&t=7
uint32_t sensor_update() {
  uint32_t data = 0, inputStream;
  int x;

  digitalWrite(PIN_SENSOR_CSEL, LOW);
  digitalWrite(PIN_SENSOR_CLK, HIGH);
  //while(digitalRead(PIN_SENSOR_SDOUT)==LOW);
  
  // tWait is supposed to be max 10us
  digitalWrite(PIN_SENSOR_CSEL, HIGH);
  //while(digitalRead(PIN_SENSOR_SDOUT)==HIGH);
  // dCsn is supposed to be typical 500ns 
  //delayNanoseconds(500);
  digitalWrite(PIN_SENSOR_CSEL, LOW);

  for (x = 0; x < SENSOR_TOTAL_BITS; x++) {
    // Set the clock low.  On the next high sensor will start to deliver data.
    digitalWrite(PIN_SENSOR_CLK, LOW);
    digitalWrite(PIN_SENSOR_CLK, HIGH);
    inputStream = digitalRead(PIN_SENSOR_SDOUT);
    //delayMicroseconds(1);
    // one bit of data is now waiting on sensor pin
    data = ((data << 1) + inputStream); // left-shift summing variable, add pin value
  }
  
  return data;
}


/**
   @input data the raw sensor reading
   @return the angle in degrees
*/
float sensor_angle(uint32_t data) {
  // it might be better to say (360*angle)/(2^16) instead of angle*(360.0/2^16) because of rounding errors
  return (data * SENSOR_ANGLE_PER_BIT);
}
