/*
 Rocket Stick
 SparkFun Electronics 2010
 
 This code is public domain - well, as much of it that is ours.
 
 This is a very bad/rough example of how to hook the 9DOF Stick to an OpenLog.
 
 I use the onboard I2C of the ATmega328 to connect to the 9DOF and then record
 the magneto, accel, and gyro in a total of nine axis about 55 times a second.
 
 This uses Bill Greiman's sdfatlib (very good!) as well as many other people's
 code (their names should be on the headers). 
 
 ----
 Modified for CU Spaceflight RaccoonKit by Jon Sowman 2011
 http://www.cusf.co.uk
 
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

int ax, ay, az;

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

  writeString(file, "#,ax,ay,az,$");
  writeCRLF(file);

  while(1) //Begin recording data to file... forever
  {
    if( ((millis() / 1000) % 2) == 0) //Toggle the status LED every second
      digitalWrite(statLED, LOW);
    else
      digitalWrite(statLED, HIGH);

    //Get accel values
    ax = getDir('x');
    ay = getDir('y');
    az = getDir('z');

    sprintf(buffer, "#,%04d,%04d,%04d,", ax, ay, az);
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
