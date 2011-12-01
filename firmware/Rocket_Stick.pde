/*
 Rocket Stick
 SparkFun Electronics 2010
 
 This code is public domain - well, as much of it that is ours.
 
 This is a very bad/rough example of how to hook the 9DOF Stick to an OpenLog.
 
 I use the onboard I2C of the ATmega328 to connect to the 9DOF and then record
 the magneto, accel, and gyro in a total of nine axis about 55 times a second.
 
 This uses Bill Greiman's sdfatlib (very good!) as well as many other people's
 code (their names should be on the headers). 
*/

#include <SdFat.h>
#include <SdFatUtil.h>

#include <Wire.h>
#include <HMC.h>

Sd2Card card;
SdVolume volume;
SdFile root;
SdFile file;

// ADXL345 Constants
#define X0 0x32
#define X1 0x33
#define Y0 0x34
#define Y1 0x35
#define Z0 0x36
#define Z1 0x37

// ITG3200 Constants
#define GYRO_ADDR 0x68 // gyro address, binary = 11101001
#define SMPLRT_DIV 0x02
#define DLPF_FS 0x16
#define INT_CFG 0x17
#define PWR_MGM 0x3E

int mx, my, mz;
int ax, ay, az;
int gx, gy, gz;

int statLED = 5;

// store error strings in flash to save RAM
#define error(s) error_P(PSTR(s))

void error_P(const char* str) {
  PgmPrint("error: ");
  SerialPrintln_P(str);
  if (card.errorCode()) {
    PgmPrint("SD error: ");
    Serial.print(card.errorCode(), HEX);
    Serial.print(',');
    Serial.println(card.errorData(), HEX);
  }
  while(1);
}
/*
 * Write CR LF to a file
 */
void writeCRLF(SdFile& f) {
  f.write((uint8_t*)"\r\n", 2);
}
/*
 * Write an unsigned number to a file
 */
void writeNumber(SdFile& f, uint32_t n) {
  uint8_t buf[10];
  uint8_t i = 0;
  do {
    i++;
    buf[sizeof(buf) - i] = n%10 + '0';
    n /= 10;
  } 
  while (n);
  f.write(&buf[sizeof(buf) - i], i);
}
/*
 * Write a string to a file
 */
void writeString(SdFile& f, char *str) {
  uint8_t n;
  for (n = 0; str[n]; n++);
  f.write((uint8_t *)str, n);
}

void setup(void) {
  Serial.begin(9600);
  Serial.println();

  Wire.begin(); // join i2c bus

  initITG3200();
  HMC.init();
  initADXL345();
}

void loop(void) {

  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  if (!card.init(SPI_FULL_SPEED)) error("card.init failed");

  // initialize a FAT volume
  if (!volume.init(&card)) error("volume.init failed");

  // open the root directory
  if (!root.openRoot(&volume)) error("openRoot failed");

  // create a new file
  char name[] = "WRITE00.TXT";
  for (uint8_t i = 0; i < 100; i++) {
    name[5] = i/10 + '0';
    name[6] = i%10 + '0';
    if (file.open(&root, name, O_CREAT | O_EXCL | O_WRITE)) break;
  }
  if (!file.isOpen()) error ("file.create");
  Serial.print("Writing to: ");
  Serial.println(name);

  char buffer[50];

  writeString(file, "#,mx,my,mz,ax,ay,az,gx,gy,gz,ms,$");
  writeCRLF(file);

  while(1) //Begin recording data to file... forever
  {
    if( ((millis() / 1000) % 2) == 0) //Toggle the status LED every second
      digitalWrite(statLED, LOW);
    else
      digitalWrite(statLED, HIGH);

    //Print HMC5843 and accel values
    HMC.getValues(&mx, &my, &mz);
    getITG3200();
    ax = getDir('x');
    ay = getDir('y');
    az = getDir('z');

    sprintf(buffer, "#,%04d,%04d,%04d,%04d,%04d,%04d,%05d,%05d,%05d,", mx, my, mz, ax, ay, az, gx, gy, gz);
    writeString(file, buffer);
    writeNumber(file, millis());
    writeString(file, ",$");
    writeCRLF(file);
    
    //Serial.println(buffer);

    file.sync(); //Record all current data to SD card
  }

  //We should never get this far!
  file.close();
  Serial.println("Done");
  while(1);
}

void initADXL345() {
  Wire.beginTransmission(0x53); // transmit to ADXL345
  Wire.send(0x2D);// POWER_CTL
  Wire.send(0x09);// measurement mode, 4hz wakeup
  Wire.send(0x31);// DATA_FORMAT
  Wire.send(0x03);// Go into 16g mode
  Wire.endTransmission(); // stop transmitting
  Serial.println("Starting");
}

byte requestByte(char dir){
  Wire.beginTransmission(0x53); // transmit to ADXL345
  Wire.send(dir);
  Wire.endTransmission(); // stop transmitting
  Wire.requestFrom(0x53, 1); // request 1 byte from ADXL345
  while(Wire.available())
  {
    return(Wire.receive()); 
  }
}

int getDir(char dir){
  int var;

  if(dir=='x'){
    var=requestByte(X0);
    var=var+(requestByte(X1)<<8);
  }
  else if(dir=='y'){
    var=requestByte(Y0);
    var=var+(requestByte(Y1)<<8);
  }
  else if(dir=='z'){
    var=requestByte(Z0);
    var=var+(requestByte(Z1)<<8);
  }
  return(var);
}

//initializes the gyroscope
void initITG3200()
{
  /*****************************************
   * 	*	ITG 3200
   * 	*	power management set to:
   * 	*	clock select = internal oscillator
   * 	* 		no reset, no sleep mode
   * 	*		no standby mode
   * 	*	sample rate to = 500Hz
   * 	*	parameter to +/- 2000 degrees/sec
   * 	*	low pass filter = 5Hz
   * 	*	no interrupt
   	******************************************/
  Wire.beginTransmission(GYRO_ADDR);
  Wire.send(PWR_MGM);
  Wire.send(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(GYRO_ADDR);
  Wire.send(SMPLRT_DIV);
  Wire.send(0xFF); // EB, 50, 80, 7F, DE, 23, 20, FF
  Wire.endTransmission();

  Wire.beginTransmission(GYRO_ADDR);
  Wire.send(DLPF_FS);
  Wire.send(0x1E); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
  Wire.endTransmission();

  Wire.beginTransmission(GYRO_ADDR);
  Wire.send(INT_CFG);
  Wire.send(0x00);
  Wire.endTransmission();
}

void getITG3200()
{
  char buffer[50];

  /**************************************
   * 		Gyro ITG-3200 I2C
   * 		registers:
   * 		x axis MSB = 1D, x axis LSB = 1E
   * 		y axis MSB = 1F, y axis LSB = 20
   * 		z axis MSB = 21, z axis LSB = 22
   	**************************************/
  // Arduino Wire library (I2C)
  Wire.beginTransmission(GYRO_ADDR);
  Wire.send(0x1D); // MSB x axis
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDR, 1); // one byte
  byte msb = 0;
  byte lsb = 0;
  while(!Wire.available());
  msb = Wire.receive();

  Wire.beginTransmission(GYRO_ADDR);
  Wire.send(0x1E); // LSB x axis
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDR, 1); // one byte

  while(!Wire.available());
  lsb = Wire.receive();

  // calculate total x axis
  gx = (( msb << 8) | lsb);

  // clear variables
  msb = 0;
  lsb = 0;

  Wire.beginTransmission(GYRO_ADDR);
  Wire.send(0x1F); // MSB y axis
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDR, 1); // one byte

  while(!Wire.available());
  msb = Wire.receive();

  Wire.beginTransmission(GYRO_ADDR);
  Wire.send(0x20); // LSB y axis
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDR, 1); // one byte

  while(!Wire.available());
  lsb = Wire.receive();

  // calculate total y axis
  gy = (( msb << 8) | lsb);

  // clear variables
  msb = 0;
  lsb = 0;

  Wire.beginTransmission(GYRO_ADDR);
  Wire.send(0x21); // MSB z axis
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDR, 1); // one byte

  while(!Wire.available());
  msb = Wire.receive();

  Wire.beginTransmission(GYRO_ADDR);
  Wire.send(0x22); // LSB z axis
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDR, 1); // one byte

  while(!Wire.available());
  lsb = Wire.receive();

  // calculate z axis
  gz = (( msb << 8) | lsb);
}
