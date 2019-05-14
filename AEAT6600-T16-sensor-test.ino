// visualizing data from an AEAT6600-T16  16 bit magnetic rotation sensor
// dan@marginallyclever.com 2018-12-07
// Arduino Mega

// uncomment to see binary (raw) sensor data
//#define BE_VERBOSE
//#define INCLUDE_MAGNETS

// uncomment to graph angles in arduino serial plot
#define ARDUINO_PLOT

#ifdef INCLUDE_MAGNETS
// uncomment to plot mag hi & mag lo
//#define PLOT_MAG
#endif


// sensor bits, flags, and masks
#define SENSOR_TOTAL_BITS    (16)  // 18 bits of data
#define SENSOR_ANGLE_BITS    (16)
#define SENSOR_ANGLE_PER_BIT (360.0/(float)((long)1<<SENSOR_ANGLE_BITS))  // 0.00549316406

// pins for sensor
#define PIN_SENSOR_CSEL   8//4
#define PIN_SENSOR_CLK    9//5
#define PIN_SENSOR_D0     10//6

#ifdef INCLUDE_MAGNETS
#define PIN_SENSOR_MAGL   43
#define PIN_SENSOR_MAGH   45
#endif

#define SAMPLES  10

float previousAngle[SAMPLES];

char ledValue=0;

void setup() {
  Serial.begin(57600);
  // put your setup code here, to run once:
  pinMode(PIN_SENSOR_CLK, OUTPUT);
  pinMode(PIN_SENSOR_CSEL, OUTPUT);
  pinMode(PIN_SENSOR_D0, INPUT);
#ifdef INCLUDE_MAGNETS
  pinMode(PIN_SENSOR_MAGL, INPUT);
  pinMode(PIN_SENSOR_MAGH, INPUT);
#endif
  
  pinMode(13, OUTPUT);
  
  digitalWrite(PIN_SENSOR_CSEL, HIGH);
  digitalWrite(PIN_SENSOR_CLK, HIGH);
  
  for(int i=SAMPLES-1;i>0;--i) {
    previousAngle[i] = 0;
  }
}


void loop() {
  uint32_t d = readSensor();
  float currentAngle = angleFromRawData(d);
  //if(previousAngle[0] != currentAngle)
  {
    float average=0;
    for(int i=SAMPLES-1;i>0;--i) {
      previousAngle[i]=previousAngle[i-1];
      average+=previousAngle[i-1];
    }
    previousAngle[0] = currentAngle;
    average += currentAngle;
    average /= (float)SAMPLES;
    
#ifdef BE_VERBOSE
    printBinary(16,d);
    Serial.print('\t');
#endif
#ifdef ARDUINO_PLOT
    Serial.print(currentAngle,3);
    Serial.print(' ');
    Serial.print(average,3);
#ifdef INCLUDE_MAGNETS
#ifdef PLOT_MAG
    //char mh = digitalRead(PIN_SENSOR_MAGH)==HIGH?'+':' ';
    //char ml = digitalRead(PIN_SENSOR_MAGL)==HIGH?'-':' ';
    Serial.print(' ');
    Serial.print(mh);
    Serial.print(' ');
    Serial.print(ml);
#endif
#endif
    Serial.println();
#endif
  }
  
  digitalWrite(13,ledValue);
  ledValue^=1;
  delay(20);
}


void printBinary(int maskSize,long data) {
  long mask = (long)1<<(maskSize-1);
  while(mask>0) {
    Serial.print( (mask&data) ? '1':'0' );
    mask>>=1;
  }
}


// from http://hades.mech.northwestern.edu/images/a/a9/AEAT-6600-T16.pdf
uint32_t readSensor() {
  uint32_t data = 0, inputStream;
  int x;

  // 10ns since last read
  digitalWrite(PIN_SENSOR_CSEL, HIGH);
  // 500ns wait
  digitalWrite(PIN_SENSOR_CSEL, LOW);

  for (x = 0; x < SENSOR_TOTAL_BITS; x++) {
    // Set the clock low.  On the next high sensor will start to deliver data.
    digitalWrite(PIN_SENSOR_CLK, LOW);
    // 50ns typical wait
    digitalWrite(PIN_SENSOR_CLK, HIGH);
    inputStream = digitalRead(PIN_SENSOR_D0);
    //delayMicroseconds(1);
    // one bit of data is now waiting on sensor pin
    data = ((data << 1) + inputStream); // left-shift summing variable, add pin value
  }
  //digitalWrite(PIN_SENSOR_CSEL, HIGH);
  digitalWrite(PIN_SENSOR_CLK, HIGH);
  
  return data;
}


// from http://hades.mech.northwestern.edu/images/a/a9/AEAT-6600-T16.pdf
uint32_t readSensorUno() {
  uint32_t data = 0;
  uint8_t inputStream;
  int x;

  digitalWrite(PIN_SENSOR_CSEL, LOW);
  // 500ns wait
  delayMicroseconds(1);
  // Set the clock low.  On the next high sensor will start to deliver data.
  digitalWrite(PIN_SENSOR_CLK, LOW);
  delayMicroseconds(1);

  //long t0=micros();
  for (x = 0; x < SENSOR_TOTAL_BITS; x++) {
    // 50ns typical wait
    digitalWrite(PIN_SENSOR_CLK, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_SENSOR_CLK, LOW);
    delayMicroseconds(1);
    inputStream = digitalRead(PIN_SENSOR_D0);
    // one bit of data is now waiting on sensor pin
    data = (data << 1) | inputStream;  // left-shift summing variable,add pin value
  }
  //long t1=micros();
  //Serial.print(t1-t0);
  //Serial.print("\t");
  digitalWrite(PIN_SENSOR_CSEL, HIGH);
  digitalWrite(PIN_SENSOR_CLK, HIGH);
  
  return data;
}


/**
 * @input data the raw sensor reading
 * @return the angle in degrees
 */
float angleFromRawData(uint32_t data) {
  data >>= SENSOR_TOTAL_BITS-SENSOR_ANGLE_BITS;
  // it might be better to say (360*angle)/(2^16) instead of angle*(360.0/2^16) because of rounding errors
  return (data * SENSOR_ANGLE_PER_BIT);
}
