/* TX, RX and SF change ping test
 *
 * Behavior:
 *  - TX: send packets at TX_INTERVAL_MS rate
 *  - RX: listen for packets
 *  - Short button press: cycle SF (7 -> 8 ... 11 -> 12 -> 7 ...)
 *  - Long button press: toggle TX/RX mode
 *  - Display shows:
 *      Mode + SF
 *      Last TX: N s ago
 *      Last RX: N s ago
 *      Last RSSI: -XXX dBm
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_st7735.h"

// -------------------- CONFIG --------------------

// LoRa frequency (adjust for your region)
#define RF_FREQUENCY        915000000 // Hz (915 MHz US; use 868e6 etc. as needed)

// LoRa basic params (except SF which changes)
#define TX_OUTPUT_POWER     14         // dBm
#define LORA_BANDWIDTH      0         // 0:125kHz, 1:250kHz, 2:500kHz
#define LORA_CODINGRATE     1         // 1:4/5,2:4/6,3:4/7,4:4/8
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT  0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON      false

#define RX_TIMEOUT_VALUE    2000
#define BUFFER_SIZE         64

// Button pin: many Heltec boards use GPIO0 for the user/BOOT button.
// Change if your board uses a different pin.
#define BUTTON_PIN          0

// TX interval in TX mode (ms)
#define TX_INTERVAL_MS      1000

// Long-press threshold (ms)
#define LONG_PRESS_MS       800

// Display Constants
#define FONT_WIDTH          11
#define FONT_HEIGHT         18
#define MODE_SF_COLOR       ST7735_WHITE
#define LAST_TX_COLOR       ST7735_GREEN
#define LAST_RX_COLOR       ST7735_CYAN
#define RSSI_COLOR          ST7735_YELLOW
#define BACKGROUND          ST7735_BLACK

// -------------------- GLOBALS --------------------

// ST7735 display objec
HT_st7735 st7735;

// Radio buffers and events
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

// App state machine (idle after waiting for receive or transmitting until transmit complete)
enum AppState {
  APP_STATE_IDLE,
  APP_STATE_TX,
  APP_STATE_RX
};

AppState state;

// App transmit / receive mode
enum AppMode {
  APP_MODE_TX,
  APP_MODE_RX
};

AppMode currentMode = APP_MODE_TX;

// Spreading factor list and current index
const uint8_t SF_LIST[] = {7, 8, 9, 10, 11, 12};
const size_t  SF_COUNT  = sizeof(SF_LIST) / sizeof(SF_LIST[0]);
size_t currentSfIndex   = 0;  // index into SF_LIST

// Stats
int16_t lastRxRssi = 0;
uint16_t lastRxSize = 0;

uint32_t lastTxMillis = 0;
uint32_t lastRxMillis = 0;
bool hasTx = false;
bool hasRx = false;

uint32_t txCounter = 0;

// Display tracking
FontDef font = Font_11x18;

AppMode modePrev = currentMode;
uint8_t sfPrev = SF_LIST[currentSfIndex];
unsigned long txSecsPrev = -1;
unsigned long rxSecsPrev = -1; 
int16_t lastRxRssiPrev = 999;

// Button tracking
bool     buttonPrev = HIGH;
uint32_t buttonPressStart = 0;
bool     buttonLongHandled = false;

// For periodic TX in TX mode
uint32_t lastTxAttemptMillis = 0;

// For periodic screen updates
uint32_t lastScreenUpdateMillis = 0;
const uint32_t SCREEN_UPDATE_MS = 1000;

// SF Updates
int8_t queuedSfIndex = -1;

// TX Sending (awaiting TX Done)
bool sending = false;

// -------------------- HELPERS --------------------

static uint8_t currentSF() {
  return SF_LIST[currentSfIndex];
}

static uint32_t secondsSince(uint32_t t) {
  if (t == 0) return 0;
  return (millis() - t) / 1000;
}

// Configure Radio for current SF and mode
void configureRadioForCurrentSF() {
  uint8_t sf = currentSF();

  // TX config
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    sf, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  // RX config
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, sf,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  Serial.print("Configured Radio SF");
  Serial.println(sf);
}

// Switch between TX and RX modes
void switchMode(AppMode newMode) {
  currentMode = newMode;

  if (currentMode == APP_MODE_TX) {
    Serial.println("Switched to TX mode");
    state = APP_STATE_TX;
    lastTxAttemptMillis = millis();
  } else {
    Serial.println("Switched to RX mode");
    state = APP_STATE_RX;
  }
}

// -------------------- DISPLAY --------------------

void initDrawScreen() {
  // Clear screen
  st7735.st7735_fill_screen(ST7735_BLACK);

  char line[40];
  uint8_t sf = currentSF();

  // Line 0: Mode + SF
  snprintf(line, sizeof(line), "Mode: %s  SF%d",
        (currentMode == APP_MODE_TX) ? "TX" : "RX", sf);
  st7735.st7735_write_str(0, 0, line, font, MODE_SF_COLOR, BACKGROUND);

  // Line 1: Last TX
  if (hasTx) {
    snprintf(line, sizeof(line), "Last TX: %lus", (unsigned long)secondsSince(lastTxMillis));
  } else {
    snprintf(line, sizeof(line), "Last TX: -");
  }
  st7735.st7735_write_str(0, 18, line, font, LAST_TX_COLOR, BACKGROUND);

  // Line 2: Last RX
  if (hasRx) {
    snprintf(line, sizeof(line), "Last RX: %lus", (unsigned long)secondsSince(lastRxMillis));
  } else {
    snprintf(line, sizeof(line), "Last RX: -");
  }
  st7735.st7735_write_str(0, 36, line, font, LAST_RX_COLOR, BACKGROUND);

  // Line 3: RSSI
  if (hasRx) {
    snprintf(line, sizeof(line), "RSSI: %d dBm", lastRxRssi);
  } else {
    snprintf(line, sizeof(line), "RSSI: -");
  }
  st7735.st7735_write_str(0, 54, line, font, RSSI_COLOR, BACKGROUND);
}

void drawDiff(uint16_t x, uint16_t y, const char *old, const char *str, uint16_t color) {
  uint8_t newLen  = strlen(str);
  uint8_t oldLen  = strlen(old);

  if (oldLen > newLen) {
    uint8_t diffLen = oldLen - newLen;
    st7735.st7735_fill_rectangle(x + (newLen * FONT_WIDTH), y, diffLen * FONT_WIDTH, FONT_HEIGHT, BACKGROUND);
  }
  st7735.st7735_write_str(x, y, str, font, color, BACKGROUND);
}

void drawScreen() {
  // Clear screen
  // st7735.st7735_fill_screen(ST7735_BLACK);

  char line[10];
  char prev[10];
  uint8_t sf = currentSF();

  // Line 0: Mode + SF
  if (modePrev != currentMode) {
    st7735.st7735_write_str(6 * FONT_WIDTH, 0, (currentMode == APP_MODE_TX) ? "TX" : "RX", Font_11x18, ST7735_WHITE, ST7735_BLACK);

    modePrev = currentMode;
  }

  if (sfPrev != sf) {
    snprintf(prev, sizeof(prev), "%d", sfPrev);
    snprintf(line, sizeof(line), "%d", sf);

    drawDiff(12 * FONT_WIDTH, 0, prev, line, MODE_SF_COLOR);

    sfPrev = sf;
  }

  // Line 1: Last TX
  if (hasTx) {
    unsigned long txSecsNew = (unsigned long)secondsSince(lastTxMillis);

    if (txSecsPrev != txSecsNew) {
      snprintf(prev, sizeof(prev), "%lus", txSecsPrev);
      snprintf(line, sizeof(line), "%lus", txSecsNew);

      drawDiff(9 * FONT_WIDTH, 18, prev, line, LAST_TX_COLOR);

      txSecsPrev = txSecsNew;
    }
  }

  // Line 2: Last RX
  if (hasRx) {
    unsigned long rxSecsNew = (unsigned long)secondsSince(lastRxMillis);

    if (rxSecsPrev != rxSecsNew) {
      snprintf(prev, sizeof(prev), "%lus", txSecsPrev);
      snprintf(line, sizeof(line), "%lus", rxSecsNew);

      drawDiff(9 * FONT_WIDTH, 36, prev, line, LAST_RX_COLOR);

      rxSecsPrev = rxSecsNew;
    }
  }

  // Line 3: RSSI
  if (hasRx && lastRxRssiPrev != lastRxRssi) {
    snprintf(prev, sizeof(prev), "%d dBm", lastRxRssiPrev);
    snprintf(line, sizeof(line), "%d dBm", lastRxRssi);

    drawDiff(6 * FONT_WIDTH, 54, prev, line, RSSI_COLOR);

    lastRxRssiPrev = lastRxRssi;
  }
}

// -------------------- BUTTON --------------------

void handleButton() {
  bool nowState = digitalRead(BUTTON_PIN); // active-low button

  if (nowState == LOW && buttonPrev == HIGH) {
    // Just pressed
    buttonPressStart = millis();
    buttonLongHandled = false;
  } else if (nowState == LOW && buttonPrev == LOW) {
    // Held
    if (!buttonLongHandled && (millis() - buttonPressStart >= LONG_PRESS_MS)) {
      // Long press → toggle mode
      buttonLongHandled = true;
      AppMode newMode = (currentMode == APP_MODE_TX) ? APP_MODE_RX : APP_MODE_TX;
      switchMode(newMode);
      drawScreen();
    }
  } else if (nowState == HIGH && buttonPrev == LOW) {
    // Released
    uint32_t pressDuration = millis() - buttonPressStart;
    if (pressDuration < LONG_PRESS_MS) {
      // Short press → cycle SF
      currentSfIndex = (currentSfIndex + 1) % SF_COUNT;
      
      if (!sending) {
        configureRadioForCurrentSF();
        Serial.print("SF changed to ");
        Serial.println(currentSF());
      } else {
        queuedSfIndex = (currentSfIndex + 1) % SF_COUNT; 
        Serial.println("New SF queued");
      }

      drawScreen();
    }
  }

  buttonPrev = nowState;
}

// -------------------- SETUP --------------------

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("LoRa + TFT SF/Mode test starting...");

  // Board init (from pingpong example)
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  // Init TFT (from ST7735_SPI example)
  st7735.st7735_init();
  Serial.println("TFT ready");

  // Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Setup radio events
  RadioEvents.TxDone   = OnTxDone;
  RadioEvents.TxTimeout= OnTxTimeout;
  RadioEvents.RxDone   = OnRxDone;

  // Init radio
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  configureRadioForCurrentSF();

  // Start in TX mode
  switchMode(APP_MODE_TX);

  initDrawScreen();
}

// -------------------- MAIN LOOP --------------------

void loop() {
  handleButton();

  // Radio state machine similar to pingpong, but respecting currentMode
  switch (state) {
    case APP_STATE_TX: {
      if (currentMode != APP_MODE_TX) {
        // If mode changed while we were here, redirect to RX
        state = APP_STATE_RX;
        break;
      }

      uint32_t now = millis();
      if (now - lastTxAttemptMillis >= TX_INTERVAL_MS) {
        lastTxAttemptMillis = now;

        txCounter++;
        snprintf(txpacket, BUFFER_SIZE, "hello %ld", (long)txCounter);
        Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",
                      txpacket, strlen(txpacket));
        Radio.Send((uint8_t *)txpacket, strlen(txpacket));
        state = APP_STATE_IDLE;

        sending = true;
      }
      break;
    }

    case APP_STATE_RX: {
      if (currentMode != APP_MODE_RX) {
        // Mode changed -> go TX
        state = APP_STATE_TX;
        break;
      }
      Serial.println("Into RX mode");
      Radio.Rx(0);      // continuous RX
      state = APP_STATE_IDLE;
      break;
    }

    case APP_STATE_IDLE:
    default:
      // Handle radio interrupts / callbacks
      Radio.IrqProcess();
      break;
  }

  // Periodic screen refresh for "N s ago"
  if (millis() - lastScreenUpdateMillis >= SCREEN_UPDATE_MS) {
    lastScreenUpdateMillis = millis();
    drawScreen();
  }

  // Short delay to avoid hammering CPU
  delay(5);
}

// -------------------- RADIO CALLBACKS --------------------

void OnTxDone(void) {
  Serial.println("TX done");
  hasTx = true;
  lastTxMillis = millis();

  if (queuedSfIndex != -1) {
    currentSfIndex = queuedSfIndex;
    queuedSfIndex = -1;

    configureRadioForCurrentSF();
    Serial.print("SF changed to ");
    Serial.println(currentSF());
  }

  sending = false;

  // After TX, schedule next action depending on mode
  if (currentMode == APP_MODE_TX) {
    state = APP_STATE_TX;
  } else {
    state = APP_STATE_RX;
  }
}

void OnTxTimeout(void) {
  Serial.println("TX timeout");
  Radio.Sleep();

  if (queuedSfIndex != -1) {
    currentSfIndex = queuedSfIndex;
    queuedSfIndex = -1;

    configureRadioForCurrentSF();
    Serial.print("SF changed to ");
    Serial.println(currentSF());
  }

  sending = false;

  // Try again according to mode
  if (currentMode == APP_MODE_TX) {
    state = APP_STATE_TX;
  } else {
    state = APP_STATE_RX;
  }
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  if (size >= BUFFER_SIZE) size = BUFFER_SIZE - 1;
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';

  lastRxRssi = rssi;
  lastRxSize = size;
  hasRx = true;
  lastRxMillis = millis();

  Radio.Sleep();

  Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n",
                rxpacket, lastRxRssi, lastRxSize);

  // Stay in RX mode if user wants RX; otherwise bounce back to TX
  if (currentMode == APP_MODE_RX) {
    state = APP_STATE_RX;
  } else {
    state = APP_STATE_TX;
  }
}
