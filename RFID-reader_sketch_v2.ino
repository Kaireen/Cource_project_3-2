#include <MFRC522.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <GyverButton.h>

// EEPROM_1 device constants
#define EEPROM_1_ADDR_Ax 0b000 //A2, A1, A0
#define EEPROM_1_ADDR (0b1010 << 3) + EEPROM_1_ADDR_Ax  // I2C device address
#define EEPROM_1_SIZE 256       // Mem size in bytes
#define EEPROM_1_SECTOR_SIZE 8  // Length of UID's thats will be saved in EEPROM_1 in bytes
#define EEPROM_1_SECTORS_TOTAL EEPROM_1_SIZE/EEPROM_1_SECTOR_SIZE

// PIN BINDING
#define SS_PIN 10
#define RST_PIN 9
#define PIEZO_PIN 8
#define S1_BTN 2
#define S2_BTN 3

GButton s1_btn(2);  // Create instance for button 1
GButton s2_btn(3);  // Create instance for button 2
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance
LiquidCrystal_PCF8574 lcd(0x27);    // Create lcd instance

// Set device mode:
// 0 - read only
// 1 - read and write to EEPROM
// 2 - view uid records in EEPROM
byte device_mode = 0;
char device_mode_title = 'R';

char lcd_default_banner[16] = "// RFID          ";
byte sectors_used_EEPROM_1 = 0;

////////////////////////////////////////////////////////////////////////////
void setup() {
  tone(PIEZO_PIN, 500, 100);
  delay(150);
  tone(PIEZO_PIN, 1000, 100);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  s1_btn.setDebounce(50);        // Debounce configuring (80 ms by default)
  s1_btn.setTimeout(300);        // Holding timeout configuring (500 ms by default)
  s1_btn.setClickTimeout(600);   // Between clicks timeout configuring (300 ms by default)
  s2_btn.setDebounce(50);
  s2_btn.setTimeout(300);
  s2_btn.setClickTimeout(600);
  
  Serial.begin(9600);
  SPI.begin();          // SPI library initialization
  Wire.begin();         // Wire library initialization
  lcd.begin(16, 2);     // Display initialization 
  lcd.setBacklight(1);  // Turn on LCD's backlight

  mfrc522.PCD_Init();                 // MFRC522 initialization
  mfrc522.PCD_DumpVersionToSerial();  // Print firmware version of the RFID-reader

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(lcd_default_banner);
  lcd.setCursor(0,1);
  lcd.print("Mem:");
  lcd.setCursor(0,6);
  lcd.setCursor(9,1);
  lcd.print("|mode:");

  mem_check();
  lcd_set_mem_using(sectors_used_EEPROM_1, EEPROM_1_SECTORS_TOTAL);
  
//  writeI2CByte(0x0, 0xAF);
//  delay(3);
//  Serial.println(readI2CByte(0x0));
}

void loop() {
  lcd_set_device_mode(device_mode_title);
  
  s1_btn.tick();
  s2_btn.tick();
  
  rfid_read_write();
  
  if (s1_btn.isSingle()) {
    Serial.println("s1_btn Single R-mode");
    tone(PIEZO_PIN, 1000, 50);
    device_mode = 0;
    device_mode_title = 'R';
  }
  if (s1_btn.isDouble()) {
    Serial.println("s1_btn Double W-mode");
    tone(PIEZO_PIN, 1000, 50);
    device_mode = 1;
    device_mode_title = 'W';
  }
  if (s1_btn.isTriple()) {
    Serial.println("s1_btn Triple");
    tone(PIEZO_PIN, 1000, 50);
    enterIn_viewMode();
  }
  if (s1_btn.isHolded()) {
//    Serial.println("s1_btn Holded");
//    tone(PIEZO_PIN, 1000, 25);
//    delay(100);
//    tone(PIEZO_PIN, 1000, 25);
  }

  if (s2_btn.isSingle()) {
//    Serial.println("s2_btn Single");
//    tone(PIEZO_PIN, 1000, 50);
  }
  if (s2_btn.isDouble()) {
    Serial.println("s2_btn Double");
    tone(PIEZO_PIN, 1000, 50);
    serial_print_data(EEPROM_1_ADDR);
  }
  if (s2_btn.isTriple()) {
    Serial.println("s2_btn Triple");
    tone(PIEZO_PIN, 1000, 50);
    wipe_data();
  }
  if (s2_btn.isHolded()) {
//    tone(PIEZO_PIN, 500, 25);
//    delay(100);
//    tone(PIEZO_PIN, 500, 25);
//    Serial.println("s2_btn Holded");
  }
}
////////////////////////////////////////////////////////////////////////////
// DEVICE MODES
//
void enterIn_viewMode(){
  device_mode = 2;
  device_mode_title = 'V';
  lcd_set_device_mode(device_mode_title);

//  rfid_reader_restart(RST_PIN);
  
  lcd.setCursor(0,1);
  lcd.print("Rec:");
  
  unsigned int current_sector_fbyte = 0;
  while(1){
    s1_btn.tick();
    s2_btn.tick();
    check_for_uid();

    if (s1_btn.isSingle()) {
      tone(PIEZO_PIN, 1000, 25);

      if(current_sector_fbyte == 0){
        if (sectors_used_EEPROM_1 != 0){
          bool sector_is_free = true;
          for (unsigned int sector_fbyte = EEPROM_1_SIZE - EEPROM_1_SECTOR_SIZE; sector_is_free == true; sector_fbyte-=EEPROM_1_SECTOR_SIZE){ // Go over sectors
            byte current_uid[EEPROM_1_SECTOR_SIZE];
            read_data(sector_fbyte, sector_fbyte+EEPROM_1_SECTOR_SIZE-1, current_uid);
            for (byte bt = 0; bt < EEPROM_1_SECTOR_SIZE; bt++){ // Check every sector for non-empty record
              if (current_uid[bt] != 0x00){ // If sector is not empty - stop on it
                sector_is_free = false;
                current_sector_fbyte = sector_fbyte;
              }
              if (bt == EEPROM_1_SECTOR_SIZE-1){ // If it is the last byte in sector - go to the next sector
                break;
              }
            }
          }
        }
        else {
          current_sector_fbyte = EEPROM_1_SIZE - EEPROM_1_SECTOR_SIZE;
        }
      }
      
      else {
        current_sector_fbyte -= EEPROM_1_SECTOR_SIZE;
      }
//      DEBUG
//      Serial.print("current_sector: ");
//      Serial.println(current_sector);
//      Serial.print("current_record: ");
//      Serial.println((current_sector)/EEPROM_1_SECTOR_SIZE);
    }
    if (s1_btn.isHolded()) {
      tone(PIEZO_PIN, 1000, 100);
      device_mode = 0;
      device_mode_title = 'R';
      lcd.setCursor(0,0);
      lcd.print(lcd_default_banner);
      lcd.setCursor(0,1);
      lcd.print("Mem:");
      mem_check();
      lcd_set_mem_using(sectors_used_EEPROM_1, EEPROM_1_SECTORS_TOTAL);
      break;
    }
    if (s2_btn.isSingle()) {
      tone(PIEZO_PIN, 1000, 25);

      if(current_sector_fbyte == EEPROM_1_SIZE - EEPROM_1_SECTOR_SIZE){
        current_sector_fbyte = 0;
      }
      else{
        current_sector_fbyte += EEPROM_1_SECTOR_SIZE;
      }
      lcd_set_uid(get_uid_str(current_sector_fbyte, current_sector_fbyte+EEPROM_1_SECTOR_SIZE-1));
      lcd_set_mem_using((current_sector_fbyte)/EEPROM_1_SECTOR_SIZE + 1, EEPROM_1_SECTORS_TOTAL);
    }
    if (s2_btn.isHolded()) {
      tone(PIEZO_PIN, 1000, 25);
      delete_record(current_sector_fbyte, EEPROM_1_SECTOR_SIZE);
    }
    
    lcd_set_uid(get_uid_str(current_sector_fbyte, current_sector_fbyte+EEPROM_1_SECTOR_SIZE-1));
    lcd_set_mem_using((current_sector_fbyte)/EEPROM_1_SECTOR_SIZE + 1, EEPROM_1_SECTORS_TOTAL);
  }
}

void check_for_uid(){
  // Waiting
  if (!mfrc522.PICC_IsNewCardPresent())
    return;
  // Reading
  if (!mfrc522.PICC_ReadCardSerial())
    return;

  byte rfid_uid_byte[EEPROM_1_SECTOR_SIZE];
  for (unsigned int bt=0; bt<EEPROM_1_SECTOR_SIZE; bt++){
    rfid_uid_byte[bt] = 0x00;
  }
  for (unsigned int bt=0; bt<mfrc522.uid.size; bt++){
    rfid_uid_byte[bt] = mfrc522.uid.uidByte[bt];
  }
  
 
  for (unsigned int sector_fbyte = 0; sector_fbyte < EEPROM_1_SIZE; sector_fbyte+=EEPROM_1_SECTOR_SIZE){
    bool uid_is_exist = false;
    byte mem_uid_byte[EEPROM_1_SECTOR_SIZE];
    read_data(sector_fbyte, sector_fbyte+EEPROM_1_SECTOR_SIZE-1, mem_uid_byte);
//  // DEBUG
//    Serial.print("mem_UID = ");
//    view_data(mem_uid_byte, sizeof(mem_uid_byte));
//    Serial.println("");
  
    for (unsigned int bt = 0; bt < EEPROM_1_SECTOR_SIZE; bt++){
//        // DEBUG
//        Serial.print(mem_uid_byte[bt], HEX);
//        Serial.print("   ");
//        Serial.println(rfid_uid_byte[bt], HEX);
      if (mem_uid_byte[bt] != rfid_uid_byte[bt]){
        break;
      }
      if (bt == EEPROM_1_SECTOR_SIZE-1){
        uid_is_exist = true;
      }
    }
    if (uid_is_exist){
      tone(PIEZO_PIN, 800, 25);
      delay(100);
      tone(PIEZO_PIN, 1000, 100);
      delay(1000);
      return;
    }
  }
  tone(PIEZO_PIN, 200, 25);
    delay(100);
    tone(PIEZO_PIN, 200, 100);
    delay(1000);

//  DEBUG
//  Serial.print("UID = ");
//  view_data(mfrc522.uid.uidByte, mfrc522.uid.size);
}

void view_data (byte *buf, byte size) {
  for (byte j = 0; j < size; j++) {
//  Serial.print(buf [j]);
  Serial.print(buf [j], HEX);
  }
}

void rfid_reader_restart(unsigned int rst_pin){
  pinMode(rst_pin, OUTPUT);
  digitalWrite(rst_pin, LOW);        // hard power down of RFID board 
  delay(10);
  digitalWrite(rst_pin, HIGH);                        
  mfrc522.PCD_Init();
}

/////////////////////////////////////////////////////////
// LCD
//
void lcd_set_default(){
  lcd.setCursor(0,0);
  lcd.print(lcd_default_banner);
}

void lcd_set_message(String str, unsigned int waitTime){
  lcd.setCursor(0,0);
  lcd.print(str);
  delay(waitTime);
}

void lcd_set_uid(String uid){
  lcd.setCursor(0,0);
  lcd.print("0x");
  lcd.setCursor(0,3);
  lcd.print(uid + "        ");
}

void lcd_set_device_mode(char mode){
  lcd.setCursor(15,1);
  lcd.print(mode);
}

void lcd_set_mem_using(byte used, byte total){
  byte used_tens = used / 10;
  byte used_ones = used % (used_tens*10);

  byte total_tens = total / 10;
  byte total_ones = total % (total_tens*10);

  String to_print = String(used_tens)+String(used_ones)+'/'+String(total_tens)+String(total_ones);
  
  lcd.setCursor(4,1);
  lcd.print(to_print);
}

////////////////////////////////////////////////////////////////
// EEPROM
//

void writeI2CByte(byte dev_addr, byte data_addr, byte data){
  Wire.beginTransmission(dev_addr);
  Wire.write(data_addr);
  Wire.write(data);
  Wire.endTransmission();
}

byte readI2CByte(byte dev_addr, byte data_addr){
  byte data = NULL;
  Wire.beginTransmission(dev_addr);
  Wire.write(data_addr);
  Wire.endTransmission();
  Wire.requestFrom(dev_addr, 1); //retrieve 1 returned byte
  delay(1);
  if(Wire.available()){
    data = Wire.read();
  }
  return data;
}

void delete_record(unsigned int sector_fbyte, byte sector_size){
  lcd_set_message("Delete record?  ", 0);
  
  if(choice() == false){
    lcd.setCursor(0,0);
    lcd_set_message("Canceled.       ", 2000);
    lcd_set_default();
    return;
  }

  lcd_set_message("Deleting...     ", 0);

  for(unsigned int bt=sector_fbyte; bt<sector_fbyte + sector_size; bt++){
    writeI2CByte(EEPROM_1_ADDR, bt, 0x00);
    delay(5);
  }
  lcd_set_message("Deleted!        ", 1000);
}

void wipe_data(){
  lcd_set_message("Wipe data?      ", 0);
  
  if(choice() == false){
    lcd.setCursor(0,0);
    lcd_set_message("Canceled.       ", 2000);
    lcd_set_default();
    return;
  }

  lcd_set_message("Wiping...       ", 0);
  
  byte symbols = 0;
  for(unsigned int i=0; i<EEPROM_1_SIZE; i++){
    writeI2CByte(EEPROM_1_ADDR, i, EEPROM_1_EMPTY_SECTOR_VAL);
    delay(4);
    Serial.print(readI2CByte(EEPROM_1_ADDR, i));
    symbols++;
    if(symbols >= EEPROM_1_SECTOR_SIZE){ symbols=0; Serial.println(); }
  }
  mem_check();
  Serial.println("\nData has been wiped.");
//  lcd.setCursor(0,0);
  lcd_set_message("Wiped!      ", 2000);
  lcd_set_default();
  lcd_set_mem_using(sectors_used_EEPROM_1, EEPROM_1_SECTORS_TOTAL);
}

byte read_data(unsigned int from_byte, unsigned int to_byte, byte buffer[]){
  for(unsigned int i=from_byte; i<=to_byte; i++){
    buffer[i - from_byte] = readI2CByte(EEPROM_1_ADDR, i);
  }
}

void serial_print_data(byte eeprom_dev_addr){
  byte symbols = 0;
    for(unsigned int i=0; i<EEPROM_1_SIZE; i++){
      Serial.print(readI2CByte(eeprom_dev_addr, i), HEX);
      symbols++;
      if(symbols >= EEPROM_1_SECTOR_SIZE){ symbols=0; Serial.println(); }
    }
}

void mem_check(){
  unsigned int sectors_taken = 0;
    for (unsigned int sector_fbyte = 0; sector_fbyte < EEPROM_1_SIZE; sector_fbyte+=EEPROM_1_SECTOR_SIZE){
      bool sector_is_free = false;
      byte current_uid[EEPROM_1_SECTOR_SIZE];
      read_data(sector_fbyte, sector_fbyte+EEPROM_1_SECTOR_SIZE-1, current_uid);
      for (byte bt = 0; bt < EEPROM_1_SECTOR_SIZE; bt++){
        if (current_uid[bt] != EEPROM_1_EMPTY_SECTOR_VAL){
          break;
        }
        if (bt == EEPROM_1_SECTOR_SIZE-1){
          sector_is_free = true;
        }
      }
      if (sector_is_free){
        continue;
      }
      sectors_taken++;
    }
    sectors_used_EEPROM_1 = sectors_taken;
}

String get_uid_str(unsigned int uid_begin_byte, unsigned int uid_end_byte){
  byte current_uid[EEPROM_1_SECTOR_SIZE];
  char current_uid_str[EEPROM_1_SECTOR_SIZE*2];
  read_data(uid_begin_byte, uid_begin_byte+EEPROM_1_SECTOR_SIZE-1, current_uid);
  array_to_string(current_uid, EEPROM_1_SECTOR_SIZE, current_uid_str);
//  Serial.print("current_uid_str: ");
//  Serial.println(current_uid_str);
  return(String(current_uid_str));
}

////////////////////////////////////////////////////////////////
// RFID
//

int rfid_read_write(){
  // Waiting
  if (!mfrc522.PICC_IsNewCardPresent())
    return;
  // Reading
  if (!mfrc522.PICC_ReadCardSerial())
    return;

  tone(PIEZO_PIN, 800, 100);
  
  char uid_str[mfrc522.uid.size*2];
  array_to_string(mfrc522.uid.uidByte, mfrc522.uid.size, uid_str);
  
  if (device_mode == 1){
    if (mfrc522.uid.size <= EEPROM_1_SECTOR_SIZE){
      if (sectors_used_EEPROM_1 == EEPROM_1_SECTORS_TOTAL){
        tone(PIEZO_PIN, 500, 25);
        delay(100);
        tone(PIEZO_PIN, 500, 100);
        lcd_set_message("Out of memory!    ", 1000);
        lcd_set_uid(uid_str);
        return 1;
      }
      for (unsigned int sector_fbyte = 0; sector_fbyte < EEPROM_1_SIZE; sector_fbyte+=EEPROM_1_SECTOR_SIZE){
        bool sector_is_free = false;
        byte current_uid[EEPROM_1_SECTOR_SIZE];
        read_data(sector_fbyte, sector_fbyte+EEPROM_1_SECTOR_SIZE-1, current_uid);
        for (byte bt = 0; bt < EEPROM_1_SECTOR_SIZE; bt++){
          if (current_uid[bt] != 0x00){
            break;
          }
          if (bt == EEPROM_1_SECTOR_SIZE-1){
            sector_is_free = true;
          }
        }
        if (sector_is_free){
          for (unsigned int bt = 0; bt < mfrc522.uid.size; bt++) {
            writeI2CByte(EEPROM_1_ADDR, (int)sector_fbyte + bt, mfrc522.uid.uidByte[bt]);
            delay(4);
          }
          break;
        }
      }
    }
    else {
      tone(PIEZO_PIN, 500, 25);
        delay(100);
        tone(PIEZO_PIN, 500, 100);
        lcd_set_message("UID > sector size!    ", 2000);
        lcd_set_uid(uid_str);
    }
  }
  
  lcd_set_uid(uid_str);
  mem_check();
  lcd_set_mem_using(sectors_used_EEPROM_1, EEPROM_1_SECTORS_TOTAL);

//  DEBUG
  Serial.print("UID = ");
  Serial.print(uid_str);
  Serial.print("  Size = ");
  Serial.print(mfrc522.uid.size);
  Serial.println();
//  Serial.print("type = ");
//  byte piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
//  Serial.print(mfrc522.PICC_GetTypeName(piccType));
//  Serial.println();
  
  delay(1000);
}

///////////////////////////////////////////////////////////////////
// BTNS
//

bool choice(){
  while(true){
    s1_btn.tick();
    s2_btn.tick();
    if (s1_btn.isHolded()) {
      tone(PIEZO_PIN, 800, 25);
      delay(100);
      tone(PIEZO_PIN, 800, 25);
      return true;
    }
    if (s2_btn.isHolded()) {
      tone(PIEZO_PIN, 400, 25);
      delay(100);
      tone(PIEZO_PIN, 400, 25);
      return false;
    }
  }
}

///////////////////////////////////////////////////////////////
// FORMATING
//

void array_to_string(byte uid[], unsigned int len, char buffer[]){
  /* Convert bytes array to chars and save them in the buffer */ 
   for (unsigned int i = 0; i < len; i++){
      byte nib1 = (uid[i] >> 4) & 0x0F;
      byte nib2 = (uid[i] >> 0) & 0x0F;
      buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
      buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
   }
   buffer[len*2] = '\0';
}
