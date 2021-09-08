
struct PulseInput {
  const uint8_t PIN;
  //uint32_t numberOfPulses;
  float val;
  float total;
  bool pressed;
  ///float val,total;  
};
//float val = 0 ,total = 0;  
unsigned long previousMillis = 0; 
long interval = 5000;

PulseInput pulseIn1 = {15, 0,0, false};
PulseInput pulseIn2 = {13, 0,0, false};
PulseInput pulseIn3 = {14, 0,0, false};
PulseInput pulseIn4 = {25, 0,0, false};
PulseInput pulseIn5 = {34, 0,0, false};
PulseInput pulseIn6 = {35, 0,0, false};
// PulseInput pulseIn7 = {36, 0, false};
// PulseInput pulseIn8 = {39, 0, false};
///////////////////////////// Hardware Interrupts calling/////////////////////////////////

void IRAM_ATTR isr1() {
  pulseIn1.total += 1;
  pulseIn1.val += 1;
  // pulseIn1.pressed = true;
}

void IRAM_ATTR isr2() {
  pulseIn2.total += 1;
  pulseIn2.val += 1;
  // pulseIn2.pressed = true;
}

void IRAM_ATTR isr3() {
  pulseIn3.total += 1;
  pulseIn3.val += 1;
  // pulseIn3.pressed = true;
}

void IRAM_ATTR isr4() {
  pulseIn4.total += 1;
  pulseIn4.val += 1;
  // pulseIn4.pressed = true;
}

void IRAM_ATTR isr5() {
 pulseIn5.total += 1;
  pulseIn5.val += 1;
  // pulseIn5.pressed = true;
}

void IRAM_ATTR isr6() {
  pulseIn6.total += 1;
  pulseIn6.val += 1;
  // pulseIn6.pressed = true;
}

// void IRAM_ATTR isr7() {
//   pulseIn1.totol += 1;
//   pulseIn1.val += 1;
//   // pulseIn7.pressed = true;
// }

// void IRAM_ATTR isr8() {
//   pulseIn1.totol += 1;
//   pulseIn1.val += 1;
//   // pulseIn8.pressed = true;
// }

void setup(){
  Serial.begin(115200);
  pinMode(pulseIn1.PIN, INPUT_PULLUP);  
  pinMode(pulseIn2.PIN, INPUT_PULLUP);
  pinMode(pulseIn3.PIN, INPUT_PULLUP);
  pinMode(pulseIn4.PIN, INPUT_PULLUP);
  pinMode(pulseIn5.PIN, INPUT_PULLUP);
  pinMode(pulseIn6.PIN, INPUT_PULLUP);
  // pinMode(pulseIn7.PIN, INPUT_PULLUP);
  // pinMode(pulseIn8.PIN, INPUT_PULLUP);

  // Attach interrupt on falling edge, thus each pulse is counted at falling edge of the signal
  attachInterrupt(pulseIn1.PIN, isr1, FALLING);
  attachInterrupt(pulseIn2.PIN, isr2, FALLING);
  attachInterrupt(pulseIn3.PIN, isr3, FALLING);
  attachInterrupt(pulseIn4.PIN, isr4, FALLING);
  attachInterrupt(pulseIn5.PIN, isr5, FALLING);
  attachInterrupt(pulseIn6.PIN, isr6, FALLING);
  // attachInterrupt(pulseIn7.PIN, isr7, FALLING);
  // attachInterrupt(pulseIn8.PIN, isr8, FALLING);
}

void loop()
{
unsigned long  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    
      
    Serial.println("----FM1-----");
    Serial.print(pulseIn1.total*0.0496);
    Serial.println("ml");
    Serial.print(pulseIn1.val*0.0496*60);
    Serial.println("ml/m");
    Serial.println("------------------");
    pulseIn1.val = 0;
    Serial.println("----FM2-----");
    Serial.print(pulseIn2.total*0.0496);
    Serial.println("ml");
    Serial.print(pulseIn2.val*0.0496*60);
    Serial.println("ml/m");
    Serial.println("------------------");
    pulseIn2.val = 0;
    Serial.println("----FM3-----");
    Serial.print(pulseIn3.total*0.0496);
    Serial.println("ml");
    Serial.print(pulseIn3.val*0.0496*60);
    Serial.println("ml/m");
    Serial.println("------------------");
    pulseIn3.val = 0;
    Serial.println("----FM4-----");
    Serial.print(pulseIn4.total*0.0496);
    Serial.println("ml");
    Serial.print(pulseIn4.val*0.0496*60);
    Serial.println("ml/m");
    Serial.println("------------------");
    pulseIn4.val = 0;
    Serial.println("----FM5-----");
    Serial.print(pulseIn5.total*0.0496);
    Serial.println("ml");
    Serial.print(pulseIn5.val*0.0496*60);
    Serial.println("ml/m");
    Serial.println("------------------");
    pulseIn5.val = 0;
    Serial.println("----FM6-----");
    Serial.print(pulseIn6.total*0.0496);
    Serial.println("ml");
    Serial.print(pulseIn6.val*0.0496*60);
    Serial.println("ml/m");
    Serial.println("------------------");
    pulseIn6.val = 0;

    previousMillis = millis();
    
    
  }
}
