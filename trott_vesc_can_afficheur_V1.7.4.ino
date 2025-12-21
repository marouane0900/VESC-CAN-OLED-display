#include <Arduino.h>
#include "STM32_CAN.h"
#include <U8g2lib.h>
#include <SimpleKalmanFilter.h>
#include <SPI.h>
#include <at24c02.h>

SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.012);

//soit l'un, soit l'autre:
//#include <EEPROM.h>
AT24C02 EEPROM(AT24C_ADDRESS_0);

STM32_CAN Can0(CAN1, DEF);  // Remplacez CAN1 et DEF par les pins et le port CAN appropriés

static CAN_message_t msg;

long unsigned int rxId;
unsigned char len;
unsigned char rxBuf[8];
char msgString[128];

int filteredpowa;

long linpVoltage, ldutyCycleNow, lavgInputCurrent, lavgMotorCurrent, ltempFET, ltempMotor, lWattHours;
float inpVoltage, dutyCycleNow, avgInputCurrent, avgMotorCurrent, tempFET, tempMotor, WattHours;
long erpm;
volatile int adCAN;
uint8_t adresse;
uint32_t idcalc;

// soit l'un, soit l'autre:
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ PB5); //36ms rafraichissement
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ PA4, /* dc=*/ PB4, /* reset=*/ PB5); //MOSI = PA7 et

void initialize() {
  Can0.begin();
  int spCAN;
  EEPROM.get(140, spCAN);
  if(spCAN == 0){Can0.setBaudRate(10000);}
  if(spCAN == 1){Can0.setBaudRate(20000);}
  if(spCAN == 2){Can0.setBaudRate(50000);}
  if(spCAN == 3){Can0.setBaudRate(75000);}
  if(spCAN == 4){Can0.setBaudRate(100000);}
  if(spCAN == 5){Can0.setBaudRate(125000);}
  if(spCAN == 6){Can0.setBaudRate(250000);}
  if(spCAN == 7){Can0.setBaudRate(500000);}
  if(spCAN == 8){Can0.setBaudRate(1000000);}
}

void spin() {
  //get_frame();
  int i = Can0.read(msg);
  if (msg.flags.extended) {
    adresse = adCAN;
    idcalc = 0x00000900 | adresse;
    if(msg.id == idcalc) { //09 = 9 = CAN_STATUS_1
      ldutyCycleNow = process_data_frame_vesc(msg.buf[6], msg.buf[7]);
      dutyCycleNow = ldutyCycleNow / 1000;
      lavgMotorCurrent = process_data_frame_vesc(msg.buf[4], msg.buf[5]);
      avgMotorCurrent = lavgMotorCurrent /10 ;
      unsigned char erpmvals[4];
      erpmvals[0] = msg.buf[3];
      erpmvals[1] = msg.buf[2];
      erpmvals[2] = msg.buf[1];
      erpmvals[3] = msg.buf[0];
      erpm = *(long *)erpmvals;
    }
    idcalc = 0x00001000 | adresse;
    if(msg.id == idcalc) { //10 = 16 = CAN_STATUS_4
      ltempFET = process_data_frame_vesc(msg.buf[0], msg.buf[1]);
      tempFET = ltempFET / 10;
      ltempMotor = process_data_frame_vesc(msg.buf[2], msg.buf[3]);
      tempMotor = ltempMotor / 10;
      lavgInputCurrent = process_data_frame_vesc(msg.buf[4], msg.buf[5]);
      avgInputCurrent = lavgInputCurrent / 10;
    }
    idcalc = 0x00001B00 | adresse;
    if(msg.id == idcalc) { //1B = 27 = CAN_STATUS_5
      char receivedByte[4];
      sprintf(receivedByte, "%02X%02X", msg.buf[4], msg.buf[5]);
      linpVoltage = hex2int(receivedByte);
      //inpVoltage = linpVoltage / 10;
    }
  }
}

void print_raw_can_data() {
  int len = 8;
  sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
  Serial3.print(msgString);
  for (byte i = 0; i < len; i++) {
    sprintf(msgString, " 0x%.2X", rxBuf[i]);
    Serial3.print(msgString);
  }
  Serial3.println();
}

float process_data_frame_vesc(uint8_t byte1, uint8_t byte2) {
  char receivedByte[2];
  sprintf(receivedByte, "%02X%02X", byte1, byte2);
  float output = hex2int(receivedByte);
  return output;
}

int hex2int(char buf[]) {
  return (short)strtol(buf, NULL, 16);
}


void vesc_set_duty(float duty) {
  uint32_t set_value = duty * 100000;
  CAN_message_t msg;
  adresse = adCAN;
  msg.id = 0x00000000 | adresse;
  msg.flags.extended = true;
  msg.len = 4;
  msg.buf[0] = (set_value >> 24) & 0xFF;
  msg.buf[1] = (set_value >> 16) & 0xFF;
  msg.buf[2] = (set_value >> 8) & 0xFF;
  msg.buf[3] = set_value & 0xFF;
  Can0.write(msg);
}

void vesc_set_current(float current) {
  uint32_t set_value = current * 1000;
  //0A = 10, 0B = 11 (CAN ID)
  CAN_message_t msg;
  adresse = adCAN;
  msg.id = 0x00000100 | adresse;
  msg.flags.extended = true;
  msg.len = 4;
  msg.buf[0] = (set_value >> 24) & 0xFF;
  msg.buf[1] = (set_value >> 16) & 0xFF;
  msg.buf[2] = (set_value >> 8) & 0xFF;
  msg.buf[3] = set_value & 0xFF;

  Can0.write(msg);
}

void vesc_set_current_brake(float current) {
  uint32_t set_value = current * 1000;

  CAN_message_t msg;
  adresse = adCAN;
  msg.id = 0x00000200 | adresse;
  msg.flags.extended = true;
  msg.len = 4;
  msg.buf[0] = (set_value >> 24) & 0xFF;
  msg.buf[1] = (set_value >> 16) & 0xFF;
  msg.buf[2] = (set_value >> 8) & 0xFF;
  msg.buf[3] = set_value & 0xFF;

  Can0.write(msg);
}

void vesc_set_erpm(float erpm) {
  uint32_t set_value = erpm;

  CAN_message_t msg;
  adresse = adCAN;
  msg.id = 0x00000300 | adresse;
  msg.flags.extended = true;
  msg.len = 4;
  msg.buf[0] = (set_value >> 24) & 0xFF;
  msg.buf[1] = (set_value >> 16) & 0xFF;
  msg.buf[2] = (set_value >> 8) & 0xFF;
  msg.buf[3] = set_value & 0xFF;

  Can0.write(msg);
}

void light_signal(bool b1, bool b2, bool b3, bool b4, bool b5) {
  uint8_t set_value = 0;
  set_value |= (b1 << 0);  // Encoder b1 dans le bit 0
  set_value |= (b2 << 1);  // Encoder b2 dans le bit 1
  set_value |= (b3 << 2);  // Encoder b3 dans le bit 2
  set_value |= (b4 << 3);
  set_value |= (b5 << 4);
  CAN_message_t msg;
  msg.id = 0x0002000;
  msg.flags.extended = true;
  msg.len = 4;
  msg.buf[0] = (set_value >> 24) & 0xFF;
  msg.buf[1] = (set_value >> 16) & 0xFF;
  msg.buf[2] = (set_value >> 8) & 0xFF;
  msg.buf[3] = set_value & 0xFF;

  Can0.write(msg);
}

void get_frame() {
  CAN_message_t receivedMsg;
  
  if (Can0.read(receivedMsg)) {
    msg.id = receivedMsg.id;
    msg.timestamp = receivedMsg.timestamp;
    msg.idhit = receivedMsg.idhit;
    msg.flags.extended = receivedMsg.flags.extended;
    msg.flags.remote = receivedMsg.flags.remote;
    msg.flags.overrun = receivedMsg.flags.overrun;
    msg.flags.reserved = receivedMsg.flags.reserved;
    msg.len = receivedMsg.len;
    msg.mb = receivedMsg.mb;
    msg.bus = receivedMsg.bus;
    msg.seq = receivedMsg.seq;

    for (int i = 0; i < msg.len; ++i) {
      msg.buf[i] = receivedMsg.buf[i];
    }
  }
}

volatile unsigned long int odold, odo;
float vitesse_kmh = 0.0;
unsigned long previousMillis = 0, currentMillis;
volatile float odometer = 0.0, odotraj = 0;
volatile float interval,distance;
volatile int pole, minbat, maxbat, brakeamps, maxamps, mincal, maxcal, diamroue, unit, brideforce, accelCAN, ratio, extcomodo;
volatile bool dejentre, goexit = 0, nopower = 0;

int edit1 = 0, edit2 = 0, edit3 = 0, edit4 = 0, edit5 = 0, edit6 = 0, edit7 = 0, edit8 = 0;
int poleprov, minbatprov, maxbatprov, brakeampsprov, maxampsprov, mincalprov, maxcalprov, diamroueprov, accelCANprov, ratioprov, adCANprov, adCANbprov, spCANprov, extcomodoprov, on3, on4, dl, val;
unsigned long tp = 0, tpp;
int mod = 0, out = 0;

const int clignooutg = PA1; //cligno gauche
const int clignooutd = PA2; //cligno droite
const int accelin = PA6;    //accel
const int bridein = PA8;    //bride
const int freining = PA9;   //frein
const int freinind = PA10;  //frein
const int boutokin = PB0;   //bouton ok
const int boutmoinin = PB1; //bouton -
const int boutplusin = PB3; //bouton +
const int lumierein = PB12; //lumiere
const int clignoind = PB13; //cligno droite
const int clignoing = PB14; //cligno gauche
const int klaxonin = PB15;  //klaxon

void setup() {
  u8g2.begin();
  //u8g2.setBusClock(3000000); //maxspeed
  u8g2.setBusClock(500000); //à virer pour de l'I2C
  Wire.begin();
  pinMode(clignooutg, OUTPUT);  //cligno gauche
  pinMode(clignooutd, OUTPUT);  //cligno droite
  pinMode(accelin, INPUT);   //accel
  pinMode(bridein, INPUT);   //bride
  pinMode(freining, INPUT);   //frein
  pinMode(freinind, INPUT);  //frein
  pinMode(boutokin, INPUT);   //bouton ok
  pinMode(boutmoinin, INPUT);   //bouton -
  pinMode(boutplusin, INPUT);   //bouton +
  pinMode(lumierein, INPUT);   //lumiere
  pinMode(clignoind, INPUT);   //cligno droite
  pinMode(clignoing, INPUT);   //cligno gauche
  pinMode(klaxonin, INPUT);   //klaxon
  int dejprog;
  EEPROM.get(200, dejprog);
  if(dejprog != 1){
    int odoput = 0;
    int poleput = 15;
    int minbatput = 576;
    int maxbatput = 756;
    int brakeampsput = 100;
    int maxampsput = 100;
    int mincalput = 1024;
    int maxcalput = 1024;
    int diamroueput = 80;
    int unitput = 1;
    int dejprogput = 1;
    int brideforceput = 0;
    int accelCANput = 0;
    int ratioput = 100;
    int adCANput = 10;
    int spCANput = 6;
    int extcomodoput = 0;
    EEPROM.put(0, odoput);
    EEPROM.put(20, poleput);
    EEPROM.put(30, minbatput);
    EEPROM.put(40, maxbatput);
    EEPROM.put(50, brakeampsput);
    EEPROM.put(60, maxampsput);
    EEPROM.put(70, mincalput);
    EEPROM.put(80, maxcalput);
    EEPROM.put(90, diamroueput);
    EEPROM.put(100, unitput);
    EEPROM.put(105, brideforceput);
    EEPROM.put(110, accelCANput);
    EEPROM.put(120, ratioput);
    EEPROM.put(130, adCANput);
    EEPROM.put(140, spCANput);
    EEPROM.put(150, extcomodoput);
    EEPROM.put(200, dejprogput);
  }
  EEPROM.get(0, odo);
  EEPROM.get(20, pole);
  odometer = odo;
  odold = odo;
  EEPROM.get(30, minbat);
  EEPROM.get(40, maxbat);
  EEPROM.get(50, brakeamps);
  EEPROM.get(60, maxamps);
  EEPROM.get(70, mincal);
  EEPROM.get(80, maxcal);
  EEPROM.get(90, diamroue);
  EEPROM.get(100, unit);
  EEPROM.get(105, brideforce);
  EEPROM.get(110, accelCAN);
  EEPROM.get(120, ratio);
  EEPROM.get(130, adCAN);
  EEPROM.get(150, extcomodo);
  initialize();

  int eprou = 0;
  while(eprou != 50){
    delay(1);
    int powa = analogRead(accelin);
    filteredpowa = simpleKalmanFilter.updateEstimate(powa);
    eprou++;
  }
  if(filteredpowa > (mincal + 20)){nopower = 1;}
}

volatile int menu = 1;
int ii = 0;
unsigned volatile int choix1 = 0, choix2 = 0;
char buf[10];
int fk = 0;
long tt;
bool dejap, dejlum;
int width_buf;
bool lumiere;

unsigned long previousMillisl = 0;
unsigned long previousMillis2 = 0;
const long intervall = 350;
bool ledState = LOW;
void loop() {
  while(menu == 1){
    spin();
    int kmh = (erpm * (diamroue * 0.00254) * 3.14 * 60 * 100) / ((1000 * pole) * ratio);
    if(unit == 0){kmh = kmh * 0.621371;}
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_logisoso34_tr);
    sprintf(msgString, "%d", kmh);
    int width_string = u8g2.getStrWidth(msgString);
    int x_center = (u8g2.getDisplayWidth() - width_string) / 2;
    u8g2.drawStr(x_center, 36, msgString);
    
    u8g2.setFont(u8g2_font_spleen6x12_mf);
    int factad = 0;
    if(kmh>9){factad = - 4;}
    if(kmh>19){factad = - 1;}
    if(kmh>99){factad = - 5;}
    if(kmh>199){factad = - 2;}
    int x_pos_kmh = x_center + width_string + 1 + factad;
    if(unit == 0){u8g2.drawStr(x_pos_kmh, 33, "MPH");}
    if(unit == 1){u8g2.drawStr(x_pos_kmh, 33, "KM/H");}
    
    u8g2.setFont(u8g2_font_helvB10_tr);
    int amps = lavgMotorCurrent/10;
    sprintf (buf, "%dA", amps);
    width_buf = u8g2.getStrWidth(buf);
    u8g2.drawStr(19 - (width_buf/2), 27, buf);
    
    
    u8g2.setFont(u8g2_font_profont10_tf);
    int ta = ltempFET/10;
    int tai = ltempFET - ta*10;
    sprintf (buf, "ESC:%d.%01d C", ta, tai);
    width_buf = u8g2.getStrWidth(buf);
    u8g2.drawGlyph((u8g2.getDisplayWidth()/4)-(width_buf/2) + width_buf - 10, 48, 0x00b0);
    u8g2.drawStr((u8g2.getDisplayWidth()/4)-(width_buf/2), 48, buf);
    
    ta = ltempMotor/10;
    tai = ltempMotor - ta*10;
    sprintf (buf, "MOT:%d.%01d C", ta, tai);
    width_buf = u8g2.getStrWidth(buf);
    u8g2.drawGlyph((3*u8g2.getDisplayWidth()/4)-(width_buf/2) + width_buf - 10, 48, 0x00b0);
    u8g2.drawStr((3*u8g2.getDisplayWidth()/4)-(width_buf/2), 48, buf);
    
    ta = linpVoltage/10;
    tai = linpVoltage - ta*10;
    sprintf (buf, "%d.%01dV", ta, tai);
    width_buf = u8g2.getStrWidth(buf);
    x_center = (u8g2.getDisplayWidth() - width_buf) / 2;
    u8g2.drawStr(x_center, 57, buf);
    
    currentMillis = millis();
    if(millis() > (previousMillis + 500)){
      interval = (currentMillis - previousMillis) / 3.6;
      distance = kmh * interval /100000;
      odometer += distance;
      odotraj += distance;
      previousMillis = currentMillis;
      odo = odometer;
      if(odold != odo){
        EEPROM.put(0, odo);
        odold = odo;
        }
      }
    int odoaf = odo/10;
    sprintf (buf, "00000%d", odoaf);
    if(odoaf >= 10){sprintf (buf, "0000%d", odoaf);}
    if(odoaf >= 100){sprintf (buf, "000%d", odoaf);}
    if(odoaf >= 1000){sprintf (buf, "00%d", odoaf);}
    if(odoaf >= 10000){sprintf (buf, "0%d", odoaf);}
    if(odoaf >= 100000){sprintf (buf, "%d", odoaf);}
    u8g2.drawStr(1, 57, buf);
    
    ta = odotraj /10;
    tai = odotraj - ta*10;
    if(unit == 0){sprintf (buf, "%d.%01dmi", ta, tai);}
    if(unit == 1){sprintf (buf, "%d.%01dkm", ta, tai);}
    width_buf = u8g2.getStrWidth(buf);
    u8g2.drawStr(126 - width_buf, 57, buf);
    
    u8g2.drawFrame(0,60,126,4);
    
    int batlevel = map(linpVoltage,minbat,maxbat,0,125);
    if(linpVoltage < minbat){batlevel = 0;}
    if(linpVoltage > maxbat){batlevel = 125;}
    u8g2.drawBox(1,61,batlevel,2);
    
    if(ii == 2){
      u8g2.drawBox(125,1,1,1);
      ii = 0;
      }
    ii++;
    
    int powa = analogRead(accelin);
    filteredpowa = simpleKalmanFilter.updateEstimate(powa);
    if(filteredpowa <= mincal){filteredpowa = mincal;}
    if(filteredpowa >= maxcal){filteredpowa = maxcal;}
    if(mincal >= maxcal){filteredpowa = mincal;}
    int powah = map(filteredpowa,mincal,maxcal,0,30);
    if(accelCAN != 0){
      u8g2.drawFrame(122,4,4,31);
      u8g2.drawBox(123,35-powah,2,powah);
    }
    float powaa = map(filteredpowa,mincal,maxcal,0,maxamps);
    if(powa <= mincal){powaa = 0;}
    if(powa >= maxcal){powaa = maxamps;}

    EEPROM.get(105, brideforce);
    int bride = digitalRead(bridein);
    if(brideforce == 1){
      bride = 1;
      bool cont = 0;
      bool okgo = 1;
      if((digitalRead(boutmoinin) == 0) || (digitalRead(boutokin) == 0)){dejap = 0;okgo = 0;}
      if(digitalRead(boutmoinin) == 1){if(digitalRead(boutokin) == 1){
        if(dejap == 0){tt = millis();dejap = 1;}
        if((millis() > (tt + 8000)) && (okgo == 1)){brideforce = 0; EEPROM.put(105, brideforce);u8g2.clearBuffer();u8g2.drawStr(60,30,"OK");u8g2.sendBuffer();cont = 1; while((digitalRead(boutmoinin) == 0) || (digitalRead(boutokin) == 0)) delay(10);}
        delay(10);
      }}
      else{dejap = 0;}
      if(cont == 1){
        while((digitalRead(boutmoinin) == 1) || (digitalRead(boutokin) == 1)){
        delay(10);
        cont = 0;
        dejap = 0;
        }
      }
    }
    else{if(digitalRead(boutmoinin) == 1){brideforce = 1; EEPROM.put(105, brideforce);}}
    if(bride == 1){
      u8g2.drawStr(17,35,"c");
    }


    if((bride == 1) && (kmh > 25)){
      powaa = 0;
    }



    bool frein, droite, gauche, klaxon;
    if((accelCAN != 0) && (nopower == 0)){
      if((digitalRead(freining) == 0) || (digitalRead(freinind) == 0)){
        vesc_set_current(powaa);
        frein = 0;
      }
      else{
        frein = 1;
        powaa = brakeamps/10;
        vesc_set_current_brake(powaa);
        }
    }
    else{
      if(amps < 0){frein = 1;}
      else{frein = 0;}
    }
    
    if(extcomodo == 1){
      lumiere = digitalRead(lumierein);
    }
    else{
      if((digitalRead(boutplusin)) && (dejlum == 0)){dejlum = 1; previousMillis2 = millis();}
      if((digitalRead(boutplusin)) && (dejlum == 1)){
        if(millis() > previousMillis2 + 500){if(lumiere == 0) lumiere = 1; else lumiere = 0; dejlum = 0;}
      }
      else dejlum = 0;
      if(digitalRead(boutplusin) == 0){dejlum = 0;}
    }

    droite = digitalRead(clignoind);
    gauche = digitalRead(clignoing);
    klaxon = digitalRead(klaxonin);
    unsigned long currentMillisl = millis();
    if (currentMillisl - previousMillisl >= intervall) {
      previousMillisl = currentMillisl;
      ledState = !ledState;
    }
    if(droite == 1){droite = ledState;}
    if(gauche == 1){gauche = ledState;}
    light_signal(frein, lumiere, droite, gauche, klaxon);
    digitalWrite(clignooutd, droite);
    digitalWrite(clignooutg, gauche);

    u8g2.sendBuffer();
    while((digitalRead(boutokin) == 1) && (fk == 1)){
      delay(10);
      }
    fk = 0;
    if((digitalRead(boutokin) == 1) && (digitalRead(boutmoinin) == 0)){
      menu = 10;
      delay(50);
      while(digitalRead(boutokin) == 1){delay(10);}
      }
  }
  
  int chl = 0, modee = 0;
  while(menu == 10){
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_helvR12_tr);
    u8g2.drawStr(35,19,"Settings");
    u8g2.setFont(u8g2_font_helvR08_tr);
    u8g2.drawStr(20,34,"Edit raw parameters");
    u8g2.drawStr(20,47,"Throttle calibration");
    u8g2.drawStr(20,60,"Unit:");
    u8g2.drawStr(8,35 + (choix1 * 13),"->");
    if(chl == 1){
      if(unit == 1){unit = 0;}
      else{unit = 1;}
      chl = 0;
      }
    if(unit == 1){u8g2.drawStr(48,60,"KM/H");}
    else{u8g2.drawStr(48,60,"MPH");}
    u8g2.sendBuffer();
    
    if(digitalRead(boutmoinin) == 1){
      if(choix1 != 2){choix1++;}
      else{choix1 = 0;}
      delay(50);
      while(digitalRead(boutmoinin) == 1){delay(10);}
      }
    if(digitalRead(boutplusin) == 1){
      if(choix1 != 0){choix1--;}
      else{choix1 = 2;}
      delay(50);
      while(digitalRead(boutplusin) == 1){delay(10);}
      }
    
    while((digitalRead(boutokin) == 1) && (fk == 1)){
      delay(10);
      }
    while((digitalRead(boutmoinin) == 1) && (fk == 1)){
      delay(10);
      }
    while((digitalRead(boutplusin) == 1) && (fk == 1)){
      delay(10);
      }
    fk = 0;
    if(digitalRead(boutokin) == 1){
      if(choix1 == 0){menu = 20;}
      if(choix1 == 1){menu = 30;}
      if(choix1 == 2){chl = 1; modee = 1;}
      delay(50);
      int tp1 = 0;
      while((digitalRead(boutokin) == 1) && (tp1 != 20)){delay(10);tp1++;}
      if(tp1 == 20){menu = 1;fk = 1;}
      }
  }
  if(modee == 1){
    EEPROM.put(100, unit);
    modee = 0;
    }
  if(dejentre == 0){
    edit1 = 0; edit2 = 0; edit3 = 0; edit4 = 0; edit5 = 0; edit6 = 0; edit7 = 0; edit8 = 0;
    poleprov = pole;
    minbatprov = minbat;
    maxbatprov = maxbat;
    brakeampsprov = brakeamps;
    maxampsprov = maxamps;
    mincalprov = mincal;
    maxcalprov = maxcal;
    diamroueprov = diamroue;
    accelCANprov = accelCAN;
    ratioprov = ratio;
    EEPROM.get(130, adCANprov);
    EEPROM.get(140, spCANprov);
    extcomodoprov = extcomodo;
    on3 = 0; on4 = 0;
    mod = 0;
    out = 0;
  }
  while((menu == 20) && (choix2 < 8)){
    dejentre = 1;
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_profont10_tf);
    if(out == 0){
      u8g2.drawStr(3,8 + (choix2 * 8),"->");
    }
    out = 0;
    u8g2.drawStr(3,8,"   Pole pair");
    u8g2.drawStr(3,8 + (8 * 1),"   Batt Vmin");
    u8g2.drawStr(3,8 + (8 * 2),"   Batt Vmax");
    u8g2.drawStr(3,8 + (8 * 3),"   Brake Amps");
    u8g2.drawStr(3,8 + (8 * 4),"   Max Amps");
    u8g2.drawStr(3,8 + (8 * 5),"   Calib low");
    u8g2.drawStr(3,8 + (8 * 6),"   Calib high");
    u8g2.drawStr(3,8 + (8 * 7),"   Wheel diam");
    u8g2.setFont(u8g2_font_profont10_tf);
    sprintf (buf, "%d PP",poleprov);
    width_buf = u8g2.getStrWidth(buf);
    u8g2.drawStr(100 - (width_buf/2),8 + (8 * 0),buf);
    if(edit1 == 1){
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(72, 9 + 8*0, 0x004D);
      u8g2.drawGlyph(118, 9 + 8*0, 0x004E);
      if(mod == 1){val = poleprov; mod = 0;}
      if(val < 0){val = 0;}
      poleprov = val;
      out = 1;
      }

    int ta = minbatprov/10;
    int tai = minbatprov - ta*10;
    u8g2.setFont(u8g2_font_profont10_tf);
    sprintf (buf, "%d.%01d V", ta, tai);
    width_buf = u8g2.getStrWidth(buf);
    u8g2.drawStr(100 - (width_buf/2),8 + (8 * 1),buf);
    if(edit2 == 1){
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(72, 9 + 8*1, 0x004D);
      u8g2.drawGlyph(118, 9 + 8*1, 0x004E);
      if(mod == 1){val = minbatprov; mod = 0;}
      if(val < 0){val = 0;}
      minbatprov = val;
      out = 1;
      }
    
    ta = maxbatprov/10;
    tai = maxbatprov - ta*10;
    u8g2.setFont(u8g2_font_profont10_tf);
    sprintf (buf, "%d.%01d V", ta, tai);
    width_buf = u8g2.getStrWidth(buf);
    u8g2.drawStr(100 - (width_buf/2),8 + (8 * 2),buf);
    if(edit3 == 1){
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(72, 9 + 8*2, 0x004D);
      u8g2.drawGlyph(118, 9 + 8*2, 0x004E);
      if(mod == 1){val = maxbatprov; mod = 0;}
      if(val < 0){val = 0;}
      maxbatprov = val;
      out = 1;
      }
    
    ta = brakeampsprov/10;
    tai = brakeampsprov - ta*10;
    u8g2.setFont(u8g2_font_profont10_tf);
    sprintf (buf, "%d.%01d A", ta, tai);
    width_buf = u8g2.getStrWidth(buf);
    u8g2.drawStr(100 - (width_buf/2),8 + (8 * 3),buf);
    if(edit4 == 1){
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(72, 9 + 8*3, 0x004D);
      u8g2.drawGlyph(118, 9 + 8*3, 0x004E);
      if(mod == 1){val = brakeampsprov; mod = 0;}
      if(val < 0){val = 0;}
      brakeampsprov = val;
      out = 1;
      }
    
    ta = maxampsprov/10;
    tai = maxampsprov - ta*10;
    u8g2.setFont(u8g2_font_profont10_tf);
    sprintf (buf, "%d.%01d A", ta, tai);
    width_buf = u8g2.getStrWidth(buf);
    u8g2.drawStr(100 - (width_buf/2),8 + (8 * 4),buf);
    if(edit5 == 1){
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(72, 9 + 8*4, 0x004D);
      u8g2.drawGlyph(118, 9 + 8*4, 0x004E);
      if(mod == 1){val = maxampsprov; mod = 0;}
      if(val < 0){val = 0;}
      maxampsprov = val;
      out = 1;
      }
    
    u8g2.setFont(u8g2_font_profont10_tf);
    sprintf (buf, "%d", mincalprov);
    width_buf = u8g2.getStrWidth(buf);
    u8g2.drawStr(100 - (width_buf/2),8 + (8 * 5),buf);
    if(edit6 == 1){
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(72, 9 + 8*5, 0x004D);
      u8g2.drawGlyph(118, 9 + 8*5, 0x004E);
      if(mod == 1){val = mincalprov; mod = 0;}
      if(val < 0){val = 0;}
      if(val > maxcalprov){val = maxcalprov;}
      mincalprov = val;
      out = 1;
      }
    
    u8g2.setFont(u8g2_font_profont10_tf);
    sprintf (buf, "%d", maxcalprov);
    width_buf = u8g2.getStrWidth(buf);
    u8g2.drawStr(100 - (width_buf/2),8 + (8 * 6),buf);
    if(edit7 == 1){
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(72, 9 + 8*6, 0x004D);
      u8g2.drawGlyph(118, 9 + 8*6, 0x004E);
      if(mod == 1){val = maxcalprov; mod = 0;}
      if(val < mincalprov){val = mincalprov;}
      if(val > 1024){val = 1024;}
      maxcalprov = val;
      out = 1;
      }
    
    ta = diamroueprov/10;
    tai = diamroueprov - ta*10;
    u8g2.setFont(u8g2_font_profont10_tf);
    sprintf (buf, "%d.%01d", ta, tai);
    width_buf = u8g2.getStrWidth(buf);
    u8g2.drawStr(98 - (width_buf/2),8 + (8 * 7),buf);
    u8g2.drawGlyph(98 - (width_buf/2) + width_buf, 8 + (8 * 7), 0x0022);
    if(edit8 == 1){
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(72, 9 + 8*7, 0x004D);
      u8g2.drawGlyph(118, 9 + 8*7, 0x004E);
      if(mod == 1){val = diamroueprov; mod = 0;}
      diamroueprov = val;
      out = 1;
      }
    
    u8g2.sendBuffer();
    
    if(digitalRead(boutmoinin) == 1){
      if((edit1 == 0) && (edit2 == 0) && (edit3 == 0) && (edit4 == 0) && (edit5 == 0) && (edit6 == 0) && (edit7 == 0) && (edit8 == 0)){
        if(choix2 != 7){choix2++;}
        else{choix2 = 8;}
        }
      else{
        val--;
        }
      
      if(on4 != 1){
        on4 = 1;
        tpp = millis();
        }
      dl = 200;
      tp = millis() - tpp;
      if(tp > 2000){
        dl = 10;
        }
      delay(dl);
      }

    if(digitalRead(boutplusin) == 1){
      if((edit1 == 0) && (edit2 == 0) && (edit3 == 0) && (edit4 == 0) && (edit5 == 0) && (edit6 == 0) && (edit7 == 0) && (edit8 == 0)){
        if(choix2 != 0){choix2--;}
        else{choix2 = 15;}
        }
      else{
        val++;
        }
      
      if(on3 != 1){
        on3 = 1;
        tpp = millis();
        }
      dl = 200;
      tp = millis() - tpp;
      if(tp > 2000){
        dl = 10;
        }
      delay(dl);
      }
    
    while((digitalRead(boutokin) == 1) && (fk == 1)){
      delay(10);
      }
    fk = 0;
    if(digitalRead(boutokin) == 1){
      if(choix2 == 0){edit1 = 1;mod=1;}
      if(choix2 == 1){edit2 = 1;mod=1;}
      if(choix2 == 2){edit3 = 1;mod=1;}
      if(choix2 == 3){edit4 = 1;mod=1;}
      if(choix2 == 4){edit5 = 1;mod=1;}
      if(choix2 == 5){edit6 = 1;mod=1;}
      if(choix2 == 6){edit7 = 1;mod=1;}
      if(choix2 == 7){edit8 = 1;mod=1;}
      delay(50);
      int tp1 = 0;
      while((digitalRead(boutokin) == 1) && (tp1 != 20)){delay(10);tp1++;}
      if(tp1 == 20){
        fk = 1;
        if(out == 0){
          menu = 10;
          goexit = 1;
          dejentre = 0;
        }
        else{
          edit1 = 0; edit2 = 0; edit3 = 0; edit4 = 0; edit5 = 0; edit6 = 0; edit7 = 0, edit8 = 0;
        }
      }
    }
    if(digitalRead(boutplusin) == 0){
      on3 = 0;
      }
    if(digitalRead(boutmoinin) == 0){
      on4 = 0;
      }
    
  }

  //__________________________________________________________________________________________________________________________________________________
  
  while((menu == 20) && (choix2 >= 8)){
    dejentre = 1;
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_profont10_tf);
    if(out == 0){
      u8g2.drawStr(3,8 + ((choix2 - 8) * 8),"->");
    }
    out = 0;
    u8g2.drawStr(3,8,"   Gear ratio");
    u8g2.drawStr(3,8 + (8 * 1),"   CAN control");
    u8g2.drawStr(3,8 + (8 * 2),"   CAN speed");
    u8g2.drawStr(3,8 + (8 * 3),"   CAN ID");
    u8g2.drawStr(3,8 + (8 * 4),"   ext comodo");
    u8g2.drawStr(3,8 + (8 * 5),"   -");
    u8g2.drawStr(3,8 + (8 * 6),"   -");
    u8g2.drawStr(3,8 + (8 * 7),"   -");
    
    int ta = ratioprov/100;
    int tai = ratioprov - ta*100;
    u8g2.setFont(u8g2_font_profont10_tf);
    sprintf (buf, "%d.%02d", ta, tai);
    width_buf = u8g2.getStrWidth(buf);
    u8g2.drawStr(100 - (width_buf/2),8 + (8 * 0),buf);
    if(edit1 == 1){
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(72, 9 + 8*0, 0x004D);
      u8g2.drawGlyph(118, 9 + 8*0, 0x004E);
      if(mod == 1){val = ratioprov; mod = 0;}
      if(val < 0){val = 0;}
      ratioprov = val;
      out = 1;
      }

    u8g2.setFont(u8g2_font_profont10_tf);
    if(accelCANprov == 1){sprintf (buf, "yes");}
    else{sprintf (buf, "no");}
    int width_buf = u8g2.getStrWidth(buf);
    u8g2.drawStr(100 - (width_buf/2),8 + (8 * 1),buf);
    if(edit2 == 1){
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(72, 9 + 8*1, 0x004D);
      u8g2.drawGlyph(118, 9 + 8*1, 0x004E);
      if(mod == 1){val = accelCANprov; mod = 0;}
      if(val < 0){val = 0;}
      if(val > 1){val = 1;}
      accelCANprov = val;
      out = 1;
      }
    
    u8g2.setFont(u8g2_font_profont10_tf);
    int canaff = CANvalue(spCANprov);
    sprintf (buf, "%dK", canaff);
    width_buf = u8g2.getStrWidth(buf);
    u8g2.drawStr(100 - (width_buf/2),8 + (8 * 2),buf);
    if(edit3 == 1){
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(72, 9 + 8*2, 0x004D);
      u8g2.drawGlyph(118, 9 + 8*2, 0x004E);
      if(mod == 1){val = spCANprov; mod = 0;}
      if(val < 0){val = 8;}
      if(val > 8){val = 0;}
      spCANprov = val;
      out = 1;
      }
    
    u8g2.setFont(u8g2_font_profont10_tf);
    sprintf (buf, "%d", adCANprov);
    width_buf = u8g2.getStrWidth(buf);
    u8g2.drawStr(100 - (width_buf/2),8 + (8 * 3),buf);
    if(edit4 == 1){
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(72, 9 + 8*3, 0x004D);
      u8g2.drawGlyph(118, 9 + 8*3, 0x004E);
      if(mod == 1){val = adCANprov; mod = 0;}
      if(val < 0){val = 255;}
      if(val > 255){val = 0;}
      adCANprov = val;
      out = 1;
      }
    
    u8g2.setFont(u8g2_font_profont10_tf);
    if(extcomodoprov == 1){sprintf (buf, "yes");}
    else{sprintf (buf, "no");}
    width_buf = u8g2.getStrWidth(buf);
    u8g2.drawStr(100 - (width_buf/2),8 + (8 * 4),buf);
    if(edit5 == 1){
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(72, 9 + 8*4, 0x004D);
      u8g2.drawGlyph(118, 9 + 8*4, 0x004E);
      if(mod == 1){val = extcomodoprov; mod = 0;}
      if(val < 0){val = 0;}
      if(val > 1){val = 1;}
      extcomodoprov = val;
      out = 1;
      }
    
    u8g2.setFont(u8g2_font_profont10_tf);
    sprintf (buf, "%d", mincalprov);
    width_buf = u8g2.getStrWidth(buf);
    //u8g2.drawStr(100 - (width_buf/2),8 + (8 * 5),buf);
    if(edit6 == 1){
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(72, 9 + 8*5, 0x004D);
      u8g2.drawGlyph(118, 9 + 8*5, 0x004E);
      if(mod == 1){val = mincalprov; mod = 0;}
      if(val < 0){val = 0;}
      //mincalprov = val;
      out = 1;
      }
    
    u8g2.setFont(u8g2_font_profont10_tf);
    sprintf (buf, "%d", maxcalprov);
    width_buf = u8g2.getStrWidth(buf);
    //u8g2.drawStr(100 - (width_buf/2),8 + (8 * 6),buf);
    if(edit7 == 1){
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(72, 9 + 8*6, 0x004D);
      u8g2.drawGlyph(118, 9 + 8*6, 0x004E);
      if(mod == 1){val = maxcalprov; mod = 0;}
      if(val < mincalprov){val = mincalprov;}
      if(val > 1024){val = 1024;}
      //maxcalprov = val;
      out = 1;
      }
    
    ta = diamroueprov/10;
    tai = diamroueprov - ta*10;
    u8g2.setFont(u8g2_font_profont10_tf);
    sprintf (buf, "%d.%01d", ta, tai);
    width_buf = u8g2.getStrWidth(buf);
    //u8g2.drawStr(98 - (width_buf/2),8 + (8 * 7),buf);
    //u8g2.drawGlyph(98 - (width_buf/2) + width_buf, 8 + (8 * 7), 0x0022);
    if(edit8 == 1){
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(72, 9 + 8*7, 0x004D);
      u8g2.drawGlyph(118, 9 + 8*7, 0x004E);
      if(mod == 1){val = diamroueprov; mod = 0;}
      //diamroueprov = val;
      out = 1;
      }
    
    u8g2.sendBuffer();
    
    if(digitalRead(boutmoinin) == 1){
      if((edit1 == 0) && (edit2 == 0) && (edit3 == 0) && (edit4 == 0) && (edit5 == 0) && (edit6 == 0) && (edit7 == 0) && (edit8 == 0)){
        if(choix2 != 15){choix2++;}
        else{choix2 = 0;}
        }
      else{
        if(edit2 == 1){
          if(val == 1){val = 0;}
          else{val = 1;}
        }
        else{val--;}
        }
      if(on4 != 1){
        on4 = 1;
        tpp = millis();
        }
      dl = 200;
      tp = millis() - tpp;
      if(tp > 2000){
        dl = 10;
        }
      delay(dl);
      }
    if(digitalRead(boutplusin) == 1){
      if((edit1 == 0) && (edit2 == 0) && (edit3 == 0) && (edit4 == 0) && (edit5 == 0) && (edit6 == 0) && (edit7 == 0) && (edit8 == 0)){
        if(choix2 != 8){choix2--;}
        else{choix2 = 7;}
        }
      else{
        if(edit2 == 1){
          if(val == 1){val = 0;}
          else{val = 1;}
        }
        else{val++;}
        }
      if(on3 != 1){
        on3 = 1;
        tpp = millis();
        }
      dl = 200;
      tp = millis() - tpp;
      if(tp > 2000){
        dl = 10;
        }
      delay(dl);
      }
    while((digitalRead(boutokin) == 1) && (fk == 1)){
      delay(10);
      }
    fk = 0;
    if(digitalRead(boutokin) == 1){
      if(choix2 == 8){edit1 = 1;mod=1;}
      if(choix2 == 9){edit2 = 1;mod=1;}
      if(choix2 == 10){edit3 = 1;mod=1;}
      if(choix2 == 11){edit4 = 1;mod=1;}
      if(choix2 == 12){edit5 = 1;mod=1;}
      if(choix2 == 13){edit6 = 1;mod=1;}
      if(choix2 == 14){edit7 = 1;mod=1;}
      if(choix2 == 15){edit8 = 1;mod=1;}
      delay(50);
      int tp1 = 0;
      while((digitalRead(boutokin) == 1) && (tp1 != 20)){delay(10);tp1++;}
      if(tp1 == 20){
        fk = 1;
        if(out == 0){
          menu = 10;
          goexit = 1;
          dejentre = 0;
        }
        else{
          edit1 = 0; edit2 = 0; edit3 = 0; edit4 = 0; edit5 = 0; edit6 = 0; edit7 = 0, edit8 = 0;
        }
      }
    }
    if(digitalRead(boutplusin) == 0){
      on3 = 0;
      }
    if(digitalRead(boutmoinin) == 0){
      on4 = 0;
      }
  }
  if(goexit == 1){
    EEPROM.put(20, poleprov);
    EEPROM.put(30, minbatprov);
    EEPROM.put(40, maxbatprov);
    EEPROM.put(50, brakeampsprov);
    EEPROM.put(60, maxampsprov);
    EEPROM.put(70, mincalprov);
    EEPROM.put(80, maxcalprov);
    EEPROM.put(90, diamroueprov);
    EEPROM.put(110, accelCANprov);
    EEPROM.put(120, ratioprov);
    EEPROM.put(130, adCANprov);
    EEPROM.put(140, spCANprov);
    EEPROM.put(150, extcomodoprov);
    pole = poleprov;
    minbat = minbatprov;
    maxbat = maxbatprov;
    brakeamps = brakeampsprov;
    maxamps = maxampsprov;
    mincal = mincalprov;
    maxcal = maxcalprov;
    diamroue = diamroueprov;
    accelCAN = accelCANprov;
    ratio = ratioprov;
    extcomodo = extcomodoprov;
    goexit = 0;
  }
  //__________________________________________________________________________________________________________________________________________________

  int mincalprovi = 1024, maxcalprovi = 0;
  int delcal = 0;
  while(menu == 30){
    delay(1);
    int powa = analogRead(accelin);
    filteredpowa = simpleKalmanFilter.updateEstimate(powa);
    delcal++;
    if(delcal == 100){menu = 31;}
    }
  while(menu == 31){
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_helvR12_tr);
    u8g2.drawStr(25,19,"Calibration");
    u8g2.setFont(u8g2_font_helvR08_tr);
    u8g2.drawStr(15,34,"Min:       Act:       Max:");
    int powa = analogRead(accelin);
    filteredpowa = simpleKalmanFilter.updateEstimate(powa);
    if(filteredpowa > maxcalprovi){maxcalprovi = filteredpowa;}
    if(filteredpowa < mincalprovi){mincalprovi = filteredpowa;}
    char bufi[6];
    sprintf (bufi, "%d       %d       %d", mincalprovi, filteredpowa, maxcalprovi);
    int width_bufi = u8g2.getStrWidth(bufi);
    int x_centeri = (u8g2.getDisplayWidth() - width_bufi) / 2;
    u8g2.drawStr(x_centeri, 47, bufi);
    //u8g2.drawStr(20,47,"Throttle calibration");
    u8g2.drawStr(5,60,"+ to validate /  - to abort");
    u8g2.sendBuffer();
    
    while((digitalRead(boutokin) == 1) && (fk == 1)){
      delay(10);
      }
    fk = 0;
    
    if(digitalRead(boutplusin) == 1){
      int tp1 = 0;
      while((digitalRead(boutplusin) == 1) && (tp1 != 40)){delay(10);tp1++;}
      if(tp1 == 40){
        fk = 1;
        menu = 10;
        EEPROM.put(70, mincalprovi);
        EEPROM.put(80, maxcalprovi);
        mincal = mincalprovi;
        maxcal = maxcalprovi;
      }
    }
    if(digitalRead(boutmoinin) == 1){
      int tp1 = 0;
      while((digitalRead(boutmoinin) == 1) && (tp1 != 40)){delay(10);tp1++;}
      if(tp1 == 40){
        fk = 1;
        menu = 10;
      }
    }
  }
}

int CANvalue(int CANindex){
  if(CANindex == 0){return 10;}
  if(CANindex == 1){return 20;}
  if(CANindex == 2){return 50;}
  if(CANindex == 3){return 75;}
  if(CANindex == 4){return 100;}
  if(CANindex == 5){return 125;}
  if(CANindex == 6){return 250;}
  if(CANindex == 7){return 500;}
  if(CANindex == 8){return 1000;}
}