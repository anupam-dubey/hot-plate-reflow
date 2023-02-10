//Temperature Controller
#include <MAX6675.h>

//LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SD.h>
#include <PID_v1.h>

//Rotary Encoder
#include <Encoder.h>
Encoder roundEnc(2, 3);

//Pushbutton on rotary encoder
#include <Bounce2.h>
//Bounce pushButton = Bounce();//4,5
Bounce  bouncer  = Bounce();
LiquidCrystal_I2C lcd(0x3F, 20, 4);
/*#define I2C_ADDR    0x3F  // Define I2C Address where the LCD is
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7
LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin,BACKLIGHT_PIN,HIGH);
*/
//Temperatursensor
int tCS = 7;               // CS pin on MAX6675
int tSO = 9;               // SO pin of MAX6675
int tSCK = 8;              // SCK pin of MAX6675
int units = 1;              // Units to readout temp (0 = raw, 1 = ˚C, 2 = ˚F)
int TCError = 0;            // ErrorFlag - TC Error
int tempVal = 0;            // Actual-Temperatur

//Display
unsigned long lastMeas = 0;           // Last temperature measurement
unsigned long lastDisp = 0;           // Last display change
unsigned long lastTempDisp = 0;       // Last temperature update on display
unsigned long lastLog = 0;            // Last Logstate
unsigned long logTime = 0;
int lastDispTemp = -1;
int dispOn = 0;                       // blink
int cPosX = -1;                       // x-position (cursor) on display
int cPosY = -1;                       // y-position on display
String emptyString = "                    ";

unsigned long lastDispPrg = 0;        //Zeit seit letztem Update des Programm-Displays

// Has the button on the rotary encoder been pressed?
// 0 = no
// 1 = yes
// 2 = yes, for more than 2 seconds
int buttonPressed = 0;

// Rotational Encoder
// 0 = not changed
// 1 = clockwise
// 2 = counterclockwise
int rotationState = 0;
int rotationNumber = 0;    
int lastRotationNumber = 0;
int rotationBigLeap = 0;     //was the rotationl encoder moved more than ~20°
int oldRotState = 0;

//Program state
// 0: No program
// 1: Manual Setpoint
// 2: Load from SD
// 3: not used
// 4: Program is running
int progState = 0; 
int subProgState = 0;
int lastProgState = -1;
int lastSubProgState = -1;

// Programs from SD-Card
int progCount = 0;
byte progLnCount = 0;
byte progTSp[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte progSprr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
word progTime[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//Which type of program is running at the moment?
//-1 = 0;
//0 = Single-Point
//>1 = SD-Card
int progRun = -1;
int runLine = -1;                     // Actual subprogram of the SDCard
unsigned long runTime = 0;            // How long is the step running already
unsigned long stepTime = 0;           // How much time to increase the ramp 
unsigned long lastPuls = 0;           // Last heating puls
int minutesPast = 0;

boolean SDOpen = false;

int aTsp = 0;
int Tsp = 120;              //Setpoint °C
int MaxT = 190;            // Maximal T °C
byte Sprr = 15;               //Rate °C/m
byte MaxR = 20;             // Maximale Ramp
int TAlert = 200;          // Shutown Temperature
int Tdig1 = Tsp/100;                        //Digits for Tsp 
int Tdig2 = (Tsp%100)/10;
int Tdig3 = Tsp%10;
int Rdig1 = Sprr/10;                        //Digits for Rp
int Rdig2 = Sprr%10;

// Initialize the MAX6675 Library for our chip
// Setup Serial output and LED Pin  
// MAX6675 Library already sets pin modes for MAX6675 chip!
MAX6675 temp(tCS,tSO,tSCK,units);

//PID Parameters
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

void setup() {
  //Serial.begin(9600);

  lcd.begin();
  //lcd.init();
  // Switch on the backlight
 // lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
 // lcd.setBacklight(HIGH);
  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.noBlink();
  lcd.clear ();
   // Setup the button
 pinMode( 4 ,INPUT);
 // Activate internal pull-up (optional) 
 digitalWrite( 4 ,HIGH);
 // After setting up the button, setup the object
 bouncer .attach( 4 );
 bouncer .interval(5);
  pinMode(10, OUTPUT);          //SC-Pin SD-Card
  //pinMode(53, OUTPUT);          //SC-Pin SD-Karte - Arduino mega
  //digitalWrite(53, HIGH);

  pinMode(6, OUTPUT);          //heating pin
}

void loop() {
  //disable Heating
  digitalWrite(6, LOW);
  
  //get user input
  getButtonState();
  getRotState();

  //change display
  setState();

  //Update display
  displayState();

  //Update Temperature
  //Faster measurement when program is running. In the programming stage the LCD lags
  if (progState == 4){  
    getTemp();      
    updateProgram();
    Input = tempVal;
    Setpoint = aTsp;
    myPID.Compute();
    heatingPuls();
    
    if ((lastLog + 1000) < millis()){
      updateLog();
      lastLog = millis();
    }
    
  }
  else if ((lastMeas + 10000) < millis()){
    getTemp();
    lastMeas = millis();
  }

  if ((tempVal > TAlert) || ( TCError ==1 )) { stopProgram(); delay(5000); }
}

void getButtonState(){
  unsigned long pushTime = bouncer.duration();
  bouncer.update();

  //How long was the button pressed - abort?
  boolean pushed = bouncer.fallingEdge();
  if ((pushed == true) && (pushTime < 1500) ){
    buttonPressed = 1;
  }
  else if ((pushed == true) && (pushTime >= 1500) ){
    buttonPressed = 2;
  }
  else { 
    buttonPressed = 0; 
  }
}


void getRotState(){
  //int roataionState = 0;
  //int rotationNumber = 0; - Scale 0-10     

  long tempState = roundEnc.read();

  if (tempState > oldRotState)
  {
    rotationState = 1;
    //    rotationNumber = (tempState%100)/10;
    int tempRotationNumber = (tempState%100)/10;
    if (tempRotationNumber != rotationNumber) { 
      rotationBigLeap = 1; 
    }
    rotationNumber = tempRotationNumber;
    if (rotationNumber < 0) {
      rotationNumber = 10 + rotationNumber; 
    }

    oldRotState = (tempState%100);
    roundEnc.write(oldRotState);
  }
  else if (tempState < oldRotState)
  {
    rotationState = 2;
    //    rotationNumber = (tempState%100)/10;
    int tempRotationNumber = (tempState%100)/10;
    if (tempRotationNumber != rotationNumber) { 
      rotationBigLeap = 1; 
    }
    rotationNumber = tempRotationNumber;
    if (rotationNumber < 0) {
      rotationNumber = 10 + rotationNumber; 
    }

    oldRotState = (tempState%100);
    roundEnc.write(oldRotState);   
  }
  else  { 
    rotationState = 0; 
    rotationBigLeap = 0; 
  }
}

void resetRotState(int digit){
  //For entering numbers in the LCD
  rotationNumber = digit; 
  //Failsave 
  if ((rotationNumber > 10) || (rotationNumber < 0)) {
    rotationNumber = 0; 
  }
  oldRotState = digit * 10;
  roundEnc.write(oldRotState);
}

//Verarbeite Eingaben am Display
void setState(){
  //Rotationstate 1 = right
  //Rotationstate 2 = left

  //Long pressing: abort
  if ((progState != 0) && (buttonPressed == 2)) { 
    stopProgram(); 
    buttonPressed = 0; 
    return; 
  }

  //submenu 0,1
  if ((progState == 0) && (rotationState != 0)){
    int select = rotationNumber%2 + 1;
    subProgState = select; 
    displayState(); 
    return; 
  }

  //choose menu entry
  //manual setpoint
  if ((progState == 0) && (buttonPressed == 1)){
    if (subProgState == 1) { 
      progState = 1; 
      subProgState = 0; 
      displayState(); 
      buttonPressed = 0; 
      return; 
    }
   //SD-Card 
    if (subProgState == 2) { 
      progState = 2; 
      subProgState = 0; 
      displayState(); 
      buttonPressed = 0; 
      return; 
    }
  }

  //Submenue 0,1 - manual mode
  if ((progState == 1) && (subProgState < 4) && (rotationState == 1) && (rotationBigLeap == 1)){
    subProgState = subProgState + 1; 
    if (subProgState > 3) { 
      subProgState = 0; 
    }
    displayState(); 
    rotationBigLeap = 0; 
    return;
  }
  if ((progState == 1) && (subProgState < 4) && (rotationState == 2) && (rotationBigLeap == 1)){
    subProgState = subProgState - 1; 
    if (subProgState < 0) { 
      subProgState = 3; 
    }
    displayState();
    rotationBigLeap = 0;
    return;
  }

  //Back, TSP, Rp, Start in Manual Mode
  if ((progState == 1) && (subProgState == 0) && (buttonPressed == 1)) { 
    stopProgram();  
    buttonPressed = 0; 
    return; 
  }
  if ((progState == 1) && (subProgState == 1) && (buttonPressed == 1)) { 
    subProgState = 11; 
    buttonPressed = 0; 
    return; 
  }
  if ((progState == 1) && (subProgState == 2) && (buttonPressed == 1)) { 
    subProgState = 21; 
    buttonPressed = 0; 
    return; 
  }
  if ((progState == 1) && (subProgState == 3) && (buttonPressed == 1)) { 
    startProgram(0); 
    buttonPressed = 0; 
    return; 
  }  

  //1., 2., 3. Digit T-Setpoint
  if ((progState == 1) && (subProgState == 11) && (buttonPressed == 1)) { 
    subProgState = 12; 
    buttonPressed = 0; 
    return; 
  }
  if ((progState == 1) && (subProgState == 12) && (buttonPressed == 1)) { 
    subProgState = 13; 
    buttonPressed = 0; 
    return; 
  }
  if ((progState == 1) && (subProgState == 13) && (buttonPressed == 1)) { 
    //Last digit
    subProgState = 1;
    Tsp = Tdig1 * 100 + Tdig2 * 10 + Tdig3;
    if (Tsp > MaxT) { 
      Tsp = MaxT; 
    }      //Maximal Temperature
    Tdig1 = Tsp/100;                        //Digits for Tsp 
    Tdig2 = (Tsp%100)/10;
    Tdig3 = Tsp%10;    

    buttonPressed = 0;
    return; 
  }

  //1., 2. Digit ramp
  if ((progState == 1) && (subProgState == 21) && (buttonPressed == 1)) { 
    subProgState = 22; 
    buttonPressed = 0; 
    return; 
  }
  if ((progState == 1) && (subProgState == 22) && (buttonPressed == 1)) { 
    //Last digit
    subProgState = 2;
    Sprr = Rdig1 * 10 + Rdig2;
    if (Sprr > MaxR) { 
      Sprr = MaxR; 
    }      //Maximal/minimal ramp
    if (Sprr < 1) { 
      Sprr = 1; 
    }
    Rdig1 = Sprr/10;                        //Digits for Rp
    Rdig2 = Sprr%10;

    buttonPressed = 0;
    return; 
  }

  //Submenu 1,1 - SD-Programm 
  if ((progState == 2) && (subProgState < 11) && (rotationState == 1) && (rotationBigLeap == 1)){
    subProgState = subProgState + 1; 
    if (subProgState > progCount) { 
      subProgState = 1; 
    }
    displayState(); 
    rotationBigLeap = 0;
    return;
  }
  if ((progState == 2) && (subProgState < 11) && (rotationState == 2) && (rotationBigLeap == 1)){
    subProgState = subProgState - 1; 
    if (subProgState < 1) { 
      subProgState = progCount; 
    }
    displayState();
    rotationBigLeap = 0;
    return;
  }

  if ((progState == 2) && (subProgState < 11) && (buttonPressed == 1)){ 
    startProgram(subProgState); 
    buttonPressed = 0; 
    return; 
  }
}

void stopProgram(){
  progState = 0;
  subProgState = 0;
  progRun = -1; 
  runLine = -1;
  lastSubProgState = -1;
  lastProgState = -1;
  displayState();
  myPID.SetMode(MANUAL);
}

void startProgram(int runType){
  progRun = runType;
  aTsp = tempVal;
  runTime = millis();
  runLine = 0;
  progState = 4;
  myPID.SetMode(AUTOMATIC);
  logTime = millis();
}

void updateProgram()
{
  //SD-Card
  //progLnCount
  //runline
  minutesPast = 1.0 * ((millis() - runTime) / 60000.0);
  if (progRun > 0){
    Tsp = progTSp[runLine];
    Sprr = progSprr[runLine];
    
    //Load new program?
    if (minutesPast >= progTime[runLine])
    {
      runLine++; 
      if (runLine > progLnCount) { stopProgram(); return; }
      runTime = millis();
    }
  } 

  //Ramp
  long timeStep = (1000.0 * 60.0) / (double)Sprr;

  if ((stepTime + timeStep < millis()) && (aTsp < Tsp)) { aTsp++; stepTime=millis(); }
  else if ((stepTime + timeStep < millis()) && (aTsp > Tsp)) { aTsp--; stepTime=millis(); }
}

void heatingPuls(){
  //pulse every 0.1 s
  if ((lastPuls + 100) < millis()) {
    digitalWrite(6, HIGH);
    int dly = Output * 1;
    //limit Power
    if (dly > 1000) { dly = 1000; }
    else if (dly < 0) { dly = 0; }
    //Serial.println(dly);
    delay(dly);
    digitalWrite(6, LOW);
    lastPuls = millis();
  }
}

void getTemp(){
  // Read the temp from the MAX6675
  float loop_temperature[2];
  for (int ii=0; ii<2; ii++)
  {
    loop_temperature[ii] = temp.read_temp();
    if (loop_temperature[ii] < 0) { 
      TCError = 1; 
    }
    delay(50);
  }
  tempVal = (int)((loop_temperature[0] + loop_temperature[1] ) / 2 + 0.5);
}


void updateTemp( boolean force = false, int lastPosX = -1, int lastPosY = -1 ){
  if (((millis() > (lastTempDisp + 1000)) && ( tempVal != lastDispTemp)) || (force == true))
  {
    lcd.setCursor(0, 3);
    lcd.print("T: "); 
    lcd.setCursor(3, 3);
    lcd.print(tempVal); 
    lcd.print((char)223); 
    lcd.print("C   ");
    lastTempDisp = millis();
    lastDispTemp = tempVal;

    if ((lastPosX != -1) || (lastPosY != -1)) { 
      lcd.setCursor(lastPosX, lastPosY); 
    }
  }
}

void fillNumber3(int number, String &dest3){
  if (number < -9) { dest3 += String(number); }
  else if (number < 0) { dest3 = "0"; dest3 += String(number); }
  else if (number < 10) { dest3 = "00"; dest3 += String(number); }
  else if (number < 100) { dest3 = "0"; dest3 += String(number); }
  else { 
    dest3 = String(number); 
  }
}

void fillNumber2(int number, String &dest2){
  if (number < 10) { 
    dest2 = "0"; 
    dest2 += String(number); 
  }
  else { 
    dest2 = String(number); 
  }
}

void printProgLine(byte number, String &line){
  line = (String)(number); 
  line += ": "; 
  line += (String)(progTSp[number]); 
  line += ", "; 
  line += (String)(progSprr[number]); 
  line += ", "; 
  line += (String)(progTime[number]);
  line += "    ";
}

void printProcessLine(String &line){
  String saSP; 
  fillNumber3( aTsp, saSP);
    String st; 
  fillNumber3( minutesPast , st);
    line = "TaSp: "; 
  line += saSP; 
  line += (char)223; 
  line += "C "; 
  line += " t: "; line += st;
}

void printSingleLine(String &line){
  String sTsp; 
  String sSprr; 
  fillNumber3(Tsp, sTsp);
  fillNumber2(Sprr, sSprr);

  line = "Tsp: "; 
  line += sTsp; 
  line += (char)223; 
  line += "C "; 
  line += "Rp: "; 
  line += sSprr; 
  line += (char)223; 
  line += "/m";
}

//Update display
void displayState(){
  if (progState==0) { 
    display0(); 
  }
  if (progState==1) { 
    display1(); 
  }
  if (progState==2) { 
    display2(); 
  }
  if (progState==4) { 
    display4();
  }
}

void display0(){
  if (lastProgState != 0){  
    cPosX = -1; 
    cPosY = -1;    
    lcd.noBlink();
    lcd.clear ();                        // go home
    lcd.print("Program");      // program ended
    lastProgState = 0;
    lastSubProgState = -1;          //Reset
    updateTemp(true);              //Update Temperature
  }

  //Update every second
  if ((subProgState == 0) && (millis() > (lastDisp + 1000)) && (dispOn == 0))
  {
    lcd.setCursor(0, 1);        // go to the 2nd line
    lcd.print("-");
    lastDisp = millis();
    dispOn = 1;
  }
  else if ((subProgState == 0) && (millis() > (lastDisp + 1000))){
    lcd.setCursor ( 0, 1 );        // go to the 2nd line
    lcd.print(emptyString);
    lastDisp = millis();
    dispOn = 0;
  }
  else if ((subProgState == 1) && (lastSubProgState != 1)){
    lcd.setCursor(0, 1); 
    lcd.print("Setpoint");
    lastSubProgState = 1;
    updateTemp(true);
  }
  else if ((subProgState == 2) && (lastSubProgState != 2)){
    lcd.setCursor(0, 1); 
    lcd.print("SDCard  ");
    lastSubProgState = 2;
    updateTemp(true);
  }

  updateTemp();
}

//Manual setpoint
void display1(){
  if (lastProgState != 1){
    cPosX = -1; 
    cPosY = -1;
    lcd.noBlink();
    lcd.clear ();                         // go home
    lcd.print("Setpoint"); 
    lcd.setCursor(0, 1);  
    String line; 
    printSingleLine(line);
    lcd.print(line);

    lastProgState = 1;
    lastSubProgState = -1;
    updateTemp(true);
  }

  //0. Back
  if ((subProgState == 0) && (lastSubProgState != 0)) {
    lcd.noBlink();
    lcd.setCursor(0, 2);
    lcd.print("Back ");
  }

  //1. Set Temp
  if ((subProgState == 1) && (lastSubProgState != 1)) {
    lcd.noBlink();    
    lcd.setCursor(0, 1);  
    String line; 
    printSingleLine(line);
    lcd.print(line);  
    lcd.setCursor(0, 2);
    lcd.print("TSp  ");
    lastSubProgState = 1;
  }

  //2. Set Ramp
  if ((subProgState == 2) && (lastSubProgState != 2)) {
    lcd.noBlink();    
    lcd.setCursor(0, 1);  
    String line; 
    printSingleLine(line);
    lcd.print(line);  

    lcd.setCursor(0, 2);
    lcd.print("Rp   ");
    lastSubProgState = 2;
  }

  //3. Start
  if ((subProgState == 3) && (lastSubProgState != 3)) {
    lcd.noBlink();
    lcd.setCursor(0, 2);
    lcd.print("Start");
    lastSubProgState = 3;
  }  

  //4. Change T 1. Digit
  if ((subProgState == 11) && (lastSubProgState != 11)) {
    //1. Digit cPosX = 1, cPosY = 1
    cPosX = 5; 
    cPosY = 1;
    lcd.blink();
    resetRotState(Tdig1);
    lcd.setCursor(cPosX, cPosY);
    lcd.print(Tdig1);
    lcd.setCursor(cPosX, cPosY);    
    lastSubProgState = 11;
  }
  else if (subProgState == 11) {
    lcd.setCursor(cPosX, cPosY);
    Tdig1 = rotationNumber;
    lcd.print(Tdig1);
    lcd.setCursor(cPosX, cPosY);    
  }

  //5. Change T 2. Digit
  if ((subProgState == 12) && (lastSubProgState != 12)) {
    //2. Digit cPosX = 6, cPosY = 1
    cPosX = 6; 
    cPosY = 1;
    lcd.blink();
    resetRotState(Tdig2);
    lcd.setCursor(cPosX, cPosY);
    lcd.print(Tdig2);
    lcd.setCursor(cPosX, cPosY);    
    lastSubProgState = 12;
  }
  else if (subProgState == 12) {
    lcd.setCursor(cPosX, cPosY);
    Tdig2 = rotationNumber;
    lcd.print(Tdig2);
    lcd.setCursor(cPosX, cPosY);    
  }

  //6. Change T 3. Digit
  if ((subProgState == 13) && (lastSubProgState != 13)) {
    //3. Digit cPosX = 7, cPosY = 1
    cPosX = 7; 
    cPosY = 1;
    lcd.blink();
    resetRotState(Tdig3);
    lcd.setCursor(cPosX, cPosY);
    lcd.print(Tdig3);
    lcd.setCursor(cPosX, cPosY);    
    lastSubProgState = 13;
  }
  else if (subProgState == 13) {
    lcd.setCursor(cPosX, cPosY);
    Tdig3 = rotationNumber;
    lcd.print(Tdig3);
    lcd.setCursor(cPosX, cPosY);    
  }

  //4. Change Ramp 1. Digit
  if ((subProgState == 21) && (lastSubProgState != 21)) {
    //1. Digit cPosX = 15, cPosY = 1
    cPosX = 15; 
    cPosY = 1;
    lcd.blink();
    resetRotState(Rdig1);
    lcd.setCursor(cPosX, cPosY);
    lcd.print(Rdig1);
    lcd.setCursor(cPosX, cPosY);    
    lastSubProgState = 21;
  }
  else if (subProgState == 21) {
    lcd.setCursor(cPosX, cPosY);
    Rdig1 = rotationNumber;
    lcd.print(Rdig1);
    lcd.setCursor(cPosX, cPosY);    
  }

  //5. Change Ramp 2. Digit
  if ((subProgState == 22) && (lastSubProgState != 22)) {
    //2. Digit cPosX = 16, cPosY = 1
    cPosX = 16; 
    cPosY = 1;
    lcd.blink();
    resetRotState(Rdig2);
    lcd.setCursor(cPosX, cPosY);
    lcd.print(Rdig2);
    lcd.setCursor(cPosX, cPosY);    
    lastSubProgState = 22;
  }
  else if (subProgState == 22) {
    lcd.setCursor(cPosX, cPosY);
    Rdig2 = rotationNumber;
    lcd.print(Rdig2);
    lcd.setCursor(cPosX, cPosY);    
  }

  updateTemp(false, cPosX, cPosY);
}

void display2(){
  if (lastProgState != 2){
    lcd.noBlink();
    lcd.clear ();                         // go home
    lcd.print("SDCard");
    lcd.setCursor(0, 1);  

    if (SDOpen != true) { 
      if (! SD.begin(10)) { stopProgram(); return; } 
    }
    SDOpen = true;
    progCount = 0;
    for (int ii = 1; ii < 10; ii++){
      String fname = String(ii); 
      fname += ".txt";
      char fnamec[6];
      fname.toCharArray(fnamec, 6);

      File myFile = SD.open(fnamec, FILE_READ);
 
      if (myFile) { 
        progCount++; 
      }
      myFile.close();      
    }

    if (progCount == 0) {
      stopProgram(); 
      return; 
    }

    lastProgState = 2;
    subProgState = 1;
    lastSubProgState = -1;
    updateTemp(true); 
  }

  if ((subProgState < 10) && (lastSubProgState != subProgState))
  {
    String fname = String(subProgState); 
    fname += ".txt";
    char fnamec[6];
    fname.toCharArray(fnamec, 6);

    File myFile = SD.open(fnamec, FILE_READ);
    if (! myFile) { 
      stopProgram(); 
    }

    byte line = 0;
    byte Slength = 0;
    char tempString[1];
    String HeadLine = "";
    String ProgLine = "";
  
    for (byte ii=0; ii < 10; ii++)
    {
      progTSp[ii] = 0;
      progSprr[ii] = 0;
      progTime[ii] = 0;
    }
    
    while (myFile.available()) {
      tempString[1] = myFile.read();
      if (tempString[1] == '\r' ) { 
      }
      else if (tempString[1] != '\n' ) { 
        ProgLine += tempString[1]; 
        Slength++; 
      }
      else {
        if (line == 0) { HeadLine = ProgLine; ProgLine = ""; Slength = 0; line++; }
        else {
          if ((line > 10) || (Slength > 20) ) { 
            stopProgram(); 
            return; 
          }

          char* accum;
          char buffer[Slength+1]; 
          ProgLine.toCharArray(buffer, Slength+1); 
          const char sep[] = ",";

          progTSp[line-1] = atoi(strtok_r(buffer, sep, &accum));  
          if (progTSp[line-1] > MaxT) { 
            progTSp[line-1] = MaxT; 
          }
          progSprr[line-1] = atoi(strtok_r(NULL, sep, &accum));  
          if (progSprr[line-1] > MaxR) { 
            progSprr[line-1] = MaxR; 
          }; 
          if (progSprr[line-1] < 1) { 
            progSprr[line-1] = 1; 
          }
          progTime[line-1] = atoi(strtok_r(NULL, sep, &accum));
          progLnCount = line - 1; //0-Anzahl-1
  
          Slength = 0;
          line++; 
          ProgLine = "";
          //
        }
      }
    }
    myFile.close();

    lcd.setCursor(0, 1);
    lcd.print(emptyString);
    lcd.setCursor(0, 1);
    lcd.print(HeadLine);
    printProgLine(0, ProgLine);
    lcd.setCursor(0, 2);
    lcd.print(emptyString);
    lcd.setCursor(0, 2);
    lcd.print(ProgLine);
    
    resetRotState(0);
    lastSubProgState = subProgState;
  }
  updateTemp();
}


void display4(){
  if ((lastDispPrg + 1000) > millis()) { return; }
  
  if (lastProgState != 4) {
  lcd.noBlink();
  lcd.clear ();
  updateTemp(true);
  lastProgState = 4;
  }
  
  if ( progRun == 0 )
  {
    lcd.setCursor(0, 0);
    String line;  printProcessLine(line);
    lcd.print(line);
    
    lcd.setCursor(0, 1);
    printSingleLine(line);
    lcd.print(line);
  }

  if ( progRun > 0 )
  {
    lcd.setCursor(0, 0);
    String line;  printProcessLine(line);
    lcd.print(line);

    lcd.setCursor(0, 1);
    printProgLine(runLine, line);
     
    lcd.print(line); 
    if (runLine != progLnCount) {
      lcd.setCursor(0, 2);
      printProgLine(runLine+1, line);
      lcd.print(line); 
    }
    else {
      lcd.setCursor(0, 2);
      lcd.print(emptyString);
    }
  }
  
  updateTemp();
}

void updateLog(){
   if (SDOpen != true) { 
      if (! SD.begin(10)) { return; } 
    }
    SDOpen = true;
 
    File myFile = SD.open("T.log", FILE_WRITE);
    
    if (! myFile) { return; }
    
    myFile.print(millis() - logTime);
//    myFile.print(", ");
//    myFile.print(aTsp);
    myFile.print(", ");
    myFile.println(tempVal);
    myFile.close();
} 
