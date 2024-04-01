#include <MCP48xx.h> //for the dacs
#include <Wire.h> //I2C for the screen
#include <Adafruit_GFX.h> //Graphic library for the screen
#include <Adafruit_SSD1306.h> //for interfacing with the screen
#include <Arduino.h> 
#include <BasicEncoder.h> //For the rotary encoders
#include <TimerOne.h> //Fake interrupts for the encoders
#include "OneButton.h" //for long/double presses of buttons to squeeze out more usage
#include <avr/pgmspace.h> //I think this was to get the atmega1284 working

MCP4822 dac1(4);
MCP4822 dac2(3);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

const uint8_t infoWidth = (SCREEN_WIDTH / 4) - 1; 
const uint8_t seqWidth = SCREEN_WIDTH - infoWidth;

bool invertState = true;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//io pin assignment
const uint8_t analogInPin1 = A0;  // Analog input pin that the potentiometer is attached to
const uint8_t analogInPin2 = A1;  // Analog input pin that the potentiometer is attached to
const uint8_t CVInPin1 = A2; //The first control voltage jack
const uint8_t CVInPin2 = A3; //The second control voltage jack
const uint8_t clockPin = 10;
//const uint8_t gateOut0 = 3;
BasicEncoder encoderA(19, 18);
BasicEncoder encoderB(21, 20);
OneButton encoderAButton(A4, true);
OneButton encoderBButton(A5, true);

//clock logic
bool clockState = 0;
bool lastClockState = 0;

//clock interval detection
unsigned long prevTime = 0;
unsigned long deltaTime = 0; 
int glideIncrement0 = 0;
int glideIncrement1 = 0;
unsigned int glideUpdateFreq0 = 0;
unsigned int glideUpdateFreq1 = 0;

//step counter
static const uint8_t minNumSequencers = 1;
static const uint8_t maxNumSequencers = 2;
uint8_t numSequencers = 2;
static const uint8_t minNumSteps = 3;
static const uint8_t maxNumSteps = 16;
uint8_t numSteps = 16;
uint8_t lastNumSteps = 16;

volatile uint8_t currentStep = 0;
uint8_t stepEditNum = 0;
bool sequenceEditNum = 0;
uint8_t rawSequence[maxNumSequencers][maxNumSteps]; //one for each sequencer

struct Step {
  uint8_t note;
  bool glide;
  bool noteOn;
}
sequence[maxNumSequencers][maxNumSteps];

static const uint8_t maxNote = 29;

static const uint8_t maxOffset = 11;
uint8_t offset = 0;
uint8_t lastOffset = 0;

//scales list for quantizer - stored in progmem
static const uint8_t PROGMEM scales[9][12]{
  {0, 0, 2, 4, 4, 6, 7, 7, 9, 9, 11, 11}, //Lydian
  {0, 0, 2, 4, 4, 5, 7, 7, 9, 9, 11, 11}, //Ionian
  {0, 0, 2, 4, 4, 5, 7, 7, 9, 9, 10, 10}, //Mixolydian
  {0, 0, 2, 3, 3, 5, 7, 7, 9, 9, 10, 10}, //Dorian
  {0, 0, 2, 3, 3, 5, 7, 7, 8, 8, 10, 10}, //Aeolian
  {0, 1, 1, 3, 3, 5, 7, 7, 8, 8, 10, 10}, //Phrygian
  {0, 1, 1, 3, 3, 5, 6, 6, 8, 8, 10, 10}, //Locrian
  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, //Chromatic
  {0, 0, 2, 3, 3, 5, 7, 7, 8, 8, 11, 11}  //Harmonic Minor
  };

static const uint8_t numScales = sizeof(scales) / sizeof(scales[0]);
uint8_t scaleNum = 0;
uint8_t lastScaleNum = 0;

bool menuMode = false;
uint8_t menuIndex = 0;

uint8_t pot_cv1 = 0;
uint8_t pot_cv2 = 0;
uint8_t LFO1Shape = 0;
uint8_t LFO1Rate = 1;
uint8_t LFO2Shape = 0;
uint8_t LFO2Rate = 1;
uint8_t dacOutScale = 67;
uint8_t * menuValues[] {&pot_cv1, &pot_cv2, &numSteps, &numSequencers, &scaleNum, &offset, &LFO1Shape, &LFO1Rate, &LFO2Shape, &LFO2Rate, &dacOutScale}; //might be able to move this to PROGMEM.

void timer_service() {
  encoderA.service();
  encoderB.service();
}

void setup() {
  Serial.begin(115200);
  //initialize clock pin as an input
  pinMode(clockPin, INPUT);
  //and the gate out pin as an output
  //pinMode(gateOut0, OUTPUT);
  //declare this interrupt first to prioritize clock interrupts over encoder interrupts
  attachInterrupt(digitalPinToInterrupt(clockPin), clockISR, CHANGE);

  //initialize timer2 interrupt to update sequence[0][].glides
  TCCR2B |= (1 << CS22);
  TCCR2A |= (1 << WGM21);
  OCR2A = (F_CPU / (64 * glideUpdateFreq0)) - 1;
  TIMSK2 |= (1 << OCIE2A);

  //initialize timer0 interrupt to update sequence[1][].glides
  TCCR0B |= (1 << CS01) | (1 << CS00);
  TCCR0A |= (1 << WGM01);
  OCR0A = (F_CPU / (64 * glideUpdateFreq1)) - 1;
  TIMSK0 |= (1 << OCIE0A);

  //this is neccessary to fake interrupt pins to have more inputs for the encoders
  //TODO - now that we have more interrupts on the atmega1284, can possibly be fixed in later version
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timer_service);

  //initialize the dac
  dac1.init();
  dac1.turnOnChannelA();
  dac1.turnOnChannelB();
  dac1.setGainA(MCP4822::Low);
  dac1.setGainB(MCP4822::Low);

  dac2.init();
  dac2.turnOnChannelA();
  dac2.turnOnChannelB();
  dac2.setGainA(MCP4822::Low);
  dac2.setGainB(MCP4822::Low);

  //initialize the display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  //set long press time.
  encoderAButton.setPressTicks(400);
  encoderBButton.setPressTicks(400);

  //set double click interval.
  encoderAButton.setClickTicks(200);
  encoderBButton.setClickTicks(200);

  // link the button 1 functions.
  encoderAButton.attachClick(click1);
  encoderAButton.attachDoubleClick(doubleclick1);
  encoderAButton.attachLongPressStop(longPressStop1);

  // link the button 2 functions.
  encoderBButton.attachClick(click2);
  encoderBButton.attachDoubleClick(doubleclick2);
  encoderBButton.attachLongPressStop(longPressStop2);

  display.clearDisplay();

  //drawing initilization
  updateDisplay();
}

void loop() {
  encoderAButton.tick();
  encoderBButton.tick();

  //rising edge
  clockState = digitalRead(clockPin);
  if (clockState != lastClockState){
    display.clearDisplay();
    if (clockState == HIGH) {
      clockHigh();
    }
  }
  lastClockState = clockState;

  offset = map(analogRead(analogInPin1) + analogRead(CVInPin1), 
                0,
                1000,
                0,
                maxOffset);

  if(offset != lastOffset){
    offsetChanged();
  }
  lastOffset = offset;

  numSteps = map(analogRead(analogInPin2) + analogRead(CVInPin2),
                0,
                1000,
                minNumSteps,
                maxNumSteps);

  if(numSteps != lastNumSteps){
    numStepsChanged();
  }
  lastNumSteps = numSteps;

  // scaleNum = map(constrain(analogRead(analogInPin2), 10, 1010),
  //                0,
  //                1000,
  //                0,
  //                numScales - 1);

  // if(scaleNum != lastScaleNum){
  //   scaleChanged();
  // }
  // lastScaleNum = scaleNum;

  //select step to edit
  if (encoderA.get_change()) {
    encoderAChanged();
  }

  // edit the step
  if(encoderB.get_change()){
    encoderBChanged();
  }
}

// quantizes notes to a scale
uint8_t quantizer(uint8_t unQuantized){

  uint8_t oct = unQuantized / 12;
  uint8_t modUnQ = unQuantized % 12;
  uint8_t degree = pgm_read_byte_near(&(scales[scaleNum][modUnQ]));
  uint8_t quantizedNote = degree + (oct * 12);
  return quantizedNote;
}

void processSequence(){
  for(uint8_t i = 0; i < numSequencers; i++){
    for(uint8_t j = 0; j < maxNumSteps; j++){
      sequence[i][j].note = constrain(quantizer(rawSequence[i][j] + offset), 0, maxNote);

      //if the note is off, set it to the value of the last note. This is to give a feeling of the note being tied if a longer decay is used.
      if(!sequence[i][j].noteOn){
        sequence[i][j].note == sequence[i][(j - 1) % numSteps].note;
      }
    } 
  }
}

void drawDividers(){
  for(uint8_t i = 0; i < 3; i++){
    display.drawRoundRect(0, (SCREEN_HEIGHT / 3 * i), infoWidth, (SCREEN_HEIGHT / 3) + 1, 5, WHITE);
  }


  display.drawRoundRect(infoWidth - 1, 0, seqWidth + 1, SCREEN_HEIGHT/(numSequencers), 3, WHITE);
  display.drawRoundRect(infoWidth - 1, SCREEN_HEIGHT/(numSequencers), seqWidth + 1, SCREEN_HEIGHT/(numSequencers), 3, WHITE);
}
void drawInfo(){
  //text initialization
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(3, 3);     // Start at top-left corner
  display.print(F("scl:"));
  display.setCursor(3, (SCREEN_HEIGHT / 6) + 3);

  if(scaleNum == 0){
  display.print(F("Lyd"));
  }else if (scaleNum == 1){
  display.print(F("Ion"));
  }else if (scaleNum == 2){
  display.print(F("Mix"));
  }else if (scaleNum == 3){
  display.print(F("Dor"));
  }else if (scaleNum == 4){
  display.print(F("Aeo"));
  }else if (scaleNum == 5){
  display.print(F("Phr"));
  }else if (scaleNum == 6){
  display.print("Loc");
  }else if (scaleNum == 7){
  display.print("Chr");
  }else if (scaleNum == 8){
  display.write("Har");
  }

  display.setCursor(3, ((SCREEN_HEIGHT / 6) * 2) + 3);
  display.print(F("off:"));
  display.setCursor(3, ((SCREEN_HEIGHT / 6) * 3) + 3);
  display.print(offset);

  display.setCursor(3, ((SCREEN_HEIGHT / 6) * 4) + 4);
  display.print(F("edt:"));
  display.setCursor(3, ((SCREEN_HEIGHT / 6) * 5) + 4);
  display.print(sequence[sequenceEditNum][stepEditNum].note);
}

//draw the notes to the screen
void drawNotes(){

  //   //change the colour of the note and edit marker to black if the white playhead marker is under it
  static const bool noteCol = 1;

  for (uint8_t i = 0; i < numSequencers; i++){
    for (uint8_t j = 0; j < numSteps; j++){

      uint8_t x1 = infoWidth + (seqWidth / numSteps * j);

      uint8_t y1 = map(sequence[i][j].note, 
                      0, 
                      maxNote,
                      ((SCREEN_HEIGHT / (numSequencers)) * (i + 1)) - 3, 
                      (SCREEN_HEIGHT / (numSequencers)) * i) + 1;

      uint8_t x2 = infoWidth + (seqWidth / numSteps) * (j + 1);

      uint8_t y2 = map(sequence[i][(j + 1) % numSteps].note, 
                      0, 
                      maxNote, 
                      ((SCREEN_HEIGHT / (numSequencers)) * (i + 1)) - 3, 
                      (SCREEN_HEIGHT / (numSequencers)) * i) + 1;

      uint8_t noteLength = seqWidth / numSteps;

      if(sequence[i][j].noteOn){
        if (sequence[i][j].glide) {
          display.drawLine(x1, y1, x2, y2, noteCol);
        }else{
          display.drawFastHLine(x1, y1, noteLength, noteCol);
      }
      }
    } 
  }
}

//draws a playhead indicating the current step being played
void marker(uint8_t markerPos){
  display.drawTriangle(infoWidth + (seqWidth / numSteps * markerPos), 
                       0, 
                       infoWidth + (seqWidth / numSteps * markerPos) + (seqWidth / numSteps), 
                       0, 
                       infoWidth + (seqWidth / numSteps * markerPos) + ((seqWidth / numSteps) / 2), 
                       4, 
                       WHITE);
  display.drawTriangle(infoWidth + (seqWidth / numSteps * markerPos), 
                       SCREEN_HEIGHT / numSequencers, 
                       infoWidth + (seqWidth / numSteps * markerPos) + (seqWidth / numSteps), 
                       SCREEN_HEIGHT / numSequencers, 
                       infoWidth + (seqWidth / numSteps * markerPos) + ((seqWidth / numSteps) / numSequencers), 
                       (SCREEN_HEIGHT / numSequencers) + 4, 
                       WHITE);
  
}

//draw the head indicating the step being edited
void editHead(uint8_t editHeadPos, uint8_t sequencerNo){

  static const bool headCol = 1;
  
  if(menuMode == false){


    display.drawFastVLine(infoWidth + (seqWidth / numSteps * editHeadPos),
                        ((SCREEN_HEIGHT / numSequencers) * sequencerNo) + 1,
                        (SCREEN_HEIGHT / numSequencers) - 2,
                        headCol);
    display.drawFastVLine(infoWidth + (seqWidth / numSteps * editHeadPos) + (seqWidth / numSteps) - 1,
                        ((SCREEN_HEIGHT / numSequencers) * sequencerNo) + 1,
                        (SCREEN_HEIGHT / numSequencers) -2,
                        headCol);
    display.display();
  }
}

//in menu mode, this will draw the editable items in the menu, and stores the labels for these items
void drawMenu(){

  static const char item0[] PROGMEM = "pot/cv 1:";
  static const char item1[] PROGMEM = "pot/cv 2:";
  static const char item2[] PROGMEM = "no. steps:";
  static const char item3[] PROGMEM = "no. seqs:";
  static const char item4[] PROGMEM = "scale";
  static const char item5[] PROGMEM = "offset:";
  static const char item6[] PROGMEM = "LFO 1 shape:";
  static const char item7[] PROGMEM = "LFO 1 rate:";
  static const char item8[] PROGMEM = "LFO 2 shape:";
  static const char item9[] PROGMEM = "LFO 2 rate:";
  static const char item10[] PROGMEM = "cv out scale:";

  static const char *const menuItems[] PROGMEM = {item0, item1, item2, item3, item4, item5, item6, item7, item8, item9, item10};

  char buffer[14];  // make sure this is large enough for the largest string it must hold

  static const uint8_t numOfItems = (sizeof(menuItems) / sizeof(menuItems[0]));

  uint8_t scrollAmt = 0;

  menuIndex = constrain(menuIndex, 0, numOfItems);

  menuIndex %= numOfItems; //to scroll back to first item once the last has been passed

  //start scrolling once you reach the bottom value
  if(menuIndex >= 7){
    scrollAmt = constrain(menuIndex - 6, 0, numOfItems - 7);
  }

  drawMenuMarker(menuIndex - scrollAmt);

  bool textCol = 1;

    for(uint8_t i = 0 + scrollAmt; i < 7 + scrollAmt; i++) {
      (menuIndex == i) ? textCol = 0 : textCol = 1;
      
      display.setTextColor(textCol);
      display.setCursor(1, ((SCREEN_HEIGHT/7) * (i - scrollAmt)) + 1);
      strcpy_P(buffer, (char *)pgm_read_word(&(menuItems[i])));  // Necessary casts and dereferencing.
      display.print(buffer);
      display.setCursor(110, ((SCREEN_HEIGHT/7) * (i - scrollAmt))+1);
      display.println(*menuValues[i]); //if moved to PROGMEM then pgm_read_byte_near.
    }
}

//a simple white marker to indicate what menu item is being edited
void drawMenuMarker(uint8_t _menuMarkerPos){
  display.fillRect(0, (SCREEN_HEIGHT/7) * _menuMarkerPos, SCREEN_WIDTH, SCREEN_HEIGHT/7, 1);
}

void click1 (){
  sequence[sequenceEditNum][stepEditNum].noteOn = !sequence[sequenceEditNum][stepEditNum].noteOn;

  processSequence();
  updateDisplay();
}
void doubleclick1(){
  for(uint8_t i = 0; i < maxNumSteps; i++){
    sequence[sequenceEditNum][i].noteOn = uint8_t((random(0, 4) == 0) ? 0 : 1); //25% chance of note being on

    if(sequence[sequenceEditNum][i].noteOn){ //only change the note's qualities if the step is active
      rawSequence[sequenceEditNum][i] = uint8_t(random(0, maxNote - maxOffset));
      sequence[sequenceEditNum][i].glide = uint8_t((random(0, 4) == 0) ? 1 : 0); //25% chance of note having a glide
    }

  }

  processSequence();
  updateDisplay();
}

void longPressStop1(){
  //reset the sequence
  for(uint8_t i = 0; i < maxNumSteps; i++){
    rawSequence[sequenceEditNum][i] = 0;

    sequence[sequenceEditNum][i].note = 0;
    sequence[sequenceEditNum][i].noteOn = 0;
    sequence[sequenceEditNum][i].glide = 0;
  }

  //couldn't hurt...
  encoderA.reset();
  encoderB.reset();
  updateDisplay();
}

void click2(){
  //toggle the state of the step's glide
  sequence[sequenceEditNum][stepEditNum].glide = !sequence[sequenceEditNum][stepEditNum].glide;
  updateDisplay();
}

void doubleclick2(){
  //toggle menu mode
  menuMode = !menuMode;
  updateDisplay();
}

void longPressStop2(){
  //clear all glides on the edited sequence
  for(uint8_t i = 0; i < maxNumSteps; i++){
    sequence[sequenceEditNum][i].glide = 0;
  }
  updateDisplay();
}

void encoderAChanged(){
  encoderB.reset();

  if(menuMode){
    menuIndex += encoderA.get_count();
    encoderA.reset();
  }else{
    //change which step is being edited
    stepEditNum += encoderA.get_count(); 

    //logic to scroll between sequencers
    if(stepEditNum < 0){
      stepEditNum = numSteps - 1;
      sequenceEditNum = !sequenceEditNum;
    }else if(stepEditNum == numSteps){
      stepEditNum = 0;
      sequenceEditNum = !sequenceEditNum;
    }else if(stepEditNum > numSteps){
      stepEditNum = numSteps - 1;
      sequenceEditNum = !sequenceEditNum;
    }
    encoderA.reset();
  }
  updateDisplay();
}

void encoderBChanged(){
  if(menuMode){
    *menuValues[menuIndex] = *menuValues[menuIndex] + encoderB.get_count(); //if moved to PROGMEM then pgm_read_byte_near. (both times)
    
    //this is mostly to constrain the value input by the encoder the relevant min and max values for that paremater 
    conditionMenuItem(menuValues[menuIndex]); //if moved to PROGMEM then pgm_read_byte_near.
  }else{
    //edit the selected step
    rawSequence[sequenceEditNum][stepEditNum] = constrain(rawSequence[sequenceEditNum][stepEditNum] + encoderB.get_count(), 0, maxNote);

    //if the input note isn't in the selected scale, increment it again until it is
    if(rawSequence[sequenceEditNum][stepEditNum] != quantizer(rawSequence[sequenceEditNum][stepEditNum])){
      rawSequence[sequenceEditNum][stepEditNum] += encoderB.get_count();
    }

    processSequence();
  }
  encoderB.reset();
  updateDisplay();
}

void conditionMenuItem(uint8_t* pMenuVal){
  //this function is to constrain the values edited in the menu.

  if (&*pMenuVal == &numSteps){
    *pMenuVal = constrain(*pMenuVal, minNumSteps, maxNumSteps);
  }else if(&*pMenuVal == &numSequencers){
    *pMenuVal = constrain(*pMenuVal, minNumSequencers, maxNumSequencers);
  }else if(&*pMenuVal == &offset){
    *pMenuVal = constrain(*pMenuVal, 0, maxOffset);
  }else if(&*pMenuVal == &scaleNum){
    *pMenuVal = constrain(*pMenuVal, 0, numScales);
  }else if(&*pMenuVal == &LFO1Rate){
    *pMenuVal = constrain(*pMenuVal, 1, 4);
  }else if(&*pMenuVal == &LFO1Shape){
    *pMenuVal = constrain(*pMenuVal, 0, 4);
  }else if(&*pMenuVal == &LFO2Rate){
    *pMenuVal = constrain(*pMenuVal, 1, 4);
  }else if(&*pMenuVal == &LFO2Shape){
    *pMenuVal = constrain(*pMenuVal, 0, 4);
  }else if(&*pMenuVal == &dacOutScale){
    *pMenuVal = constrain(*pMenuVal, 50, 100);
  }
}

void offsetChanged(){
  processSequence();
  updateDisplay();
}

void scaleChanged(){
  processSequence();
  updateDisplay();
}

void numStepsChanged(){
  stepEditNum = constrain(stepEditNum, 0, numSteps - 1);
  updateDisplay();
}

void clockISR(){ //Keep this as fast as possible! blocks any other processer (i.e. reading knob values/encoder)
  //advance a step, if we reach the end of the sequence, wrap around to the start
  if(digitalRead(clockPin) == HIGH){
      currentStep += 1;
      if ((currentStep % (numSteps)) == 0){
      currentStep = 0;
    }

    //reset the glide
    glideIncrement0 = 0;
    glideIncrement1 = 0;

    if(sequence[0][currentStep].glide){
      //calculate the frequency at which to update voltage1 
      unsigned int deltaMilliVolts0 = (sequence[0][(currentStep + 1) % 16].note - sequence[0][currentStep].note) * dacOutScale; 
      glideUpdateFreq0 = deltaTime / deltaMilliVolts0;
    }else if(sequence[1][currentStep].glide){
      //calculate the frequency at which to update voltage2 
      unsigned int deltaMilliVolts1 = (sequence[1][(currentStep + 1) % 16].note - sequence[1][currentStep].note) * dacOutScale; 
      glideUpdateFreq1 = deltaTime / deltaMilliVolts1;
    }

    //TODO: speed this up! setting DAC values and updating them unnecessarily is slowing this down. maybe move it to clockHigh
    //TODO: implement accents by using different DAC values, add it here.
    //convert note values to DAC voltages. Trial and error got me 1v/oct with a scaling factor of 84.
    //only set and update the DAC if there's a new note
    if(sequence[0][currentStep].noteOn == 1){
      uint8_t stepVal1 = sequence[0][currentStep].note;
      unsigned int voltage1 = constrain(stepVal1 * dacOutScale, 0, 4000);
      dac1.setVoltageB(voltage1);
      dac1.updateDAC();
      dac2.setVoltageB(4000);
      dac2.updateDAC();
    }

    if(sequence[1][currentStep].noteOn == 1){
      uint8_t stepVal2 = sequence[1][currentStep].note;
      unsigned int voltage2 = constrain(stepVal2 * dacOutScale, 0, 4000);
      dac1.setVoltageA(voltage2);
      dac1.updateDAC();
      dac2.setVoltageA(4000);
      dac2.updateDAC();
    }

    //TODO implement accents by using different DAC values, add it here.

  }else{
    if(!sequence[0][currentStep].glide){
      //digitalWrite(gateOut0, LOW);
      dac2.setVoltageB(0);
      dac2.updateDAC();
    }else if (!sequence[1][currentStep].glide){
      dac2.setVoltageA(0);
      dac2.updateDAC();
    }
  }
}
//operates similarly to the clockISR, but is not dependent on an interrupt, but rather clock state changes polled in the loop
//we do this seperately because it updates the display, which is too time consuming for an ISR.
//here we also calculate the timing between clock pulses to scale the glide into 1 clock pulse.
void clockHigh(){
  unsigned long now = millis();
  deltaTime = now - prevTime;
  prevTime = now;

  Serial.println("clock!");

  updateDisplay();
}

//update dac for glides[0] based on timer2 interrupt
ISR(TIMER2_COMPA_vect){

  uint8_t thisStep0 = sequence[0][currentStep].note;
  uint8_t nextStep0 = sequence[0][(currentStep + 1) % 16].note;
  if(sequence[0][currentStep].glide && thisStep0 != nextStep0){ // no point in gliding a note to itself
    updateDACGlide0(thisStep0, nextStep0);
  }

  //if glides[0] and glides[1] have the same update frequency, the timer0 interrupt for glides[1] will be overruled by timer2. 
  //in the event of this, we update both channels using timer2.
  //this is just to mitigate race conditions
  if(sequence[1][currentStep].glide && glideUpdateFreq1 == glideUpdateFreq0){
    uint8_t thisStep1 = sequence[1][currentStep].note;
    uint8_t nextStep1 = sequence[1][(currentStep + 1) % 16].note;
    if(thisStep1 != nextStep1){ // no point in gliding a note to itself
      updateDACGlide1(thisStep1, nextStep1);
    }

  }
}

void updateDACGlide0(uint8_t _thisStep, uint8_t _nextStep){
  if(_nextStep > _thisStep){
    glideIncrement0 += 1;
  }else if(_nextStep < _thisStep){
    glideIncrement0 -= 1;
  }

  unsigned int voltage1 = constrain((_thisStep * dacOutScale) + glideIncrement0, 0, 4000);
  dac1.setVoltageB(voltage1);
  dac1.updateDAC();
}

//update dac for glides[0] based on timer0 interrupt
ISR(TIMER0_COMPA_vect) {

  uint8_t thisStep = sequence[1][currentStep].note;
  uint8_t nextStep = sequence[1][(currentStep + 1) % 16].note;
  if(sequence[1][currentStep].glide && thisStep != nextStep){
    updateDACGlide1(thisStep, nextStep);
  }
}

void updateDACGlide1(uint8_t _thisStep, uint8_t _nextStep){
  if(_nextStep > _thisStep){
    glideIncrement1 += 1;
  }else if(_nextStep < _thisStep){
    glideIncrement1 -= 1;
  }

  unsigned int voltage2 = constrain((_thisStep * dacOutScale) + glideIncrement1, 0, 4000);
  dac1.setVoltageA(voltage2);
  dac1.updateDAC();
}

//Update the display - use sparingly, e.g only when a user input is changed.
void updateDisplay(){
  display.clearDisplay();
  if(menuMode){
    drawMenu();
  }else{
    drawDividers();
    drawInfo();
    marker(currentStep);
    drawNotes();
    editHead(stepEditNum, sequenceEditNum);
  }
  display.display();
}

//void plotLFO(bool lfoNum, uint8_t rate){
  // rate = rate * 2;
  // rate = constrain(rate, 2, 8);

  // if(LFO1Shape == 0){ //sine
  //   return;
  // }else if(LFO1Shape == 1){
  //   for(uint8_t i = 0; i < numSteps; i++){
  //     float x = ((rate * PI) / numSteps) * i;
  //     uint8_t y = uint8_t((sin(x) * (maxNote / 2)) + (maxNote / 2));
  //     rawSequence[lfoNum][i] = y;
  //     sequence[lfoNum][i].glide = 1;
  //   }
  // }
  // processSequence();
//}
