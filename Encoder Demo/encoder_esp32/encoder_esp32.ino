/*
---------------------------------------------
𝑬-𝒎𝒂𝒊𝒍: aslamhshakil20@gmail.com
𝑭𝒂𝒄𝒆𝒃𝒐𝒐𝒌: https://www.facebook.com/RoboTechInnovator/
𝑾𝒉𝒂𝒕'𝒔 𝑨𝒑𝒑: https://wa.me/c/8801647467658 (Get Encoder Motor)
Instagram: https://www.instagram.com/robotech_innovator?igsh=aXk4Z2JrZHc4NjJu
*/

//ENCODER PIN DEFINITIONS
#define encoder_left_A 5 //(green wire)
#define encoder_left_B 18 //(yellow wire)
#define encoder_right_A 22 //(yellow wire)
#define encoder_right_B 23 //(green wire)

//ENCODER COUNTERS
volatile long left_pulses  = 0;
volatile long right_pulses = 0;
//LAST QUADRATURE STATES
volatile uint8_t lastStateL = 0;
volatile uint8_t lastStateR = 0;
//TASK HANDLE
TaskHandle_t EncoderTaskHandle;

//pulse decode and Count
inline int8_t decodeQuadrature(uint8_t last, uint8_t current) {
  if ((last == 0b00 && current == 0b01) ||
      (last == 0b01 && current == 0b11) ||
      (last == 0b11 && current == 0b10) ||
      (last == 0b10 && current == 0b00)) {
    return +1;
  }
  if ((last == 0b00 && current == 0b10) ||
      (last == 0b10 && current == 0b11) ||
      (last == 0b11 && current == 0b01) ||
      (last == 0b01 && current == 0b00)) {
    return -1;
  }
  return 0;
}

//ENCODER TASK ON CORE 0
void EncoderTask(void *parameter) {
  // Initialize last states
  lastStateL = (digitalRead(encoder_left_A) << 1) | digitalRead(encoder_left_B);
  lastStateR = (digitalRead(encoder_right_A) << 1) | digitalRead(encoder_right_B);

  while (true) {
    // -------- LEFT ENCODER --------
    uint8_t currentStateL = (digitalRead(encoder_left_A) << 1) | digitalRead(encoder_left_B);

    int8_t dirL = decodeQuadrature(lastStateL, currentStateL);
    left_pulses += dirL;
    lastStateL = currentStateL;

    // -------- RIGHT ENCODER --------
    uint8_t currentStateR = (digitalRead(encoder_right_A) << 1) | digitalRead(encoder_right_B);

    int8_t dirR = decodeQuadrature(lastStateR, currentStateR);
    right_pulses += dirR;
    lastStateR = currentStateR;

    //delay
    vTaskDelay(1);
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(encoder_left_A,  INPUT_PULLUP);
  pinMode(encoder_left_B,  INPUT_PULLUP);
  pinMode(encoder_right_A, INPUT_PULLUP);
  pinMode(encoder_right_B, INPUT_PULLUP);

  // Create encoder task on Core 0
  xTaskCreatePinnedToCore(
    EncoderTask,         // Task function
    "EncoderTask",
    4096,                // Stack size
    NULL,
    2,                   // Priority
    &EncoderTaskHandle,
    0                    // Core 0
  );
}


void loop() {
  Serial.print("Left Pulses: ");
  Serial.print(left_pulses);
  Serial.print(" | Right Pulses: ");
  Serial.println(right_pulses);
}
