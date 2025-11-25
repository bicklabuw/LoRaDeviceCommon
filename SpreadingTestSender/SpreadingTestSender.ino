/* HELTEC PER TEST - SENDER (MASTER) - INVERTED COLORS */
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_st7735.h"

// --- CONFIGURATION ---
#define RF_FREQUENCY        915000000 // Hz
#define TX_OUTPUT_POWER     14        // dBm
#define PACKETS_PER_SF      10        // Packets per batch
#define SF_MIN              7
#define SF_MAX              12
#define RX_TIMEOUT_VALUE    1000      // Radio hardware timeout
#define SAFETY_TIMEOUT      2500      // Software watchdog

// Hardware Objects
HT_st7735 st7735;
#define BUFFER_SIZE 60
char txpacket[BUFFER_SIZE];
char displayBuf[50];

// Variables
volatile int currentSF = SF_MAX;
volatile int currentPacketIndex = 1;
volatile int missedPackets = 0;
volatile int totalMissed = 0;
volatile int lastRssi = 0;
unsigned long txStartTime = 0;

typedef enum { STATE_TX, STATE_WAIT_ACK, STATE_NEXT_SF, STATE_STOP, STATE_LOWPOWER } States_t;
volatile States_t state = STATE_TX;
static RadioEvents_t RadioEvents;

void ConfigRadio(int sf) {
    Radio.Sleep();
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, 0, sf, 1, 8, false, true, 0, 0, false, 3000);
    Radio.SetRxConfig(MODEM_LORA, 0, sf, 1, 0, 8, 0, false, 0, true, 0, 0, false, true);
}

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    st7735.st7735_init();
    // Inverted: White background looks black
    st7735.st7735_fill_screen(ST7735_WHITE);

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    
    ConfigRadio(currentSF);
    state = STATE_TX;
}

void loop() {
    switch(state) {
        case STATE_TX:
            delay(200); 
            
            // Draw Status (Inverted Colors)
            st7735.st7735_fill_screen(ST7735_WHITE); // Looks Black
            // Use Yellow text to look Blue
            sprintf(displayBuf, "TX SF: %d", currentSF);
            st7735.st7735_write_str(0, 0, displayBuf, Font_7x10, ST7735_YELLOW);
            sprintf(displayBuf, "Pkt: %d / %d", currentPacketIndex, PACKETS_PER_SF);
            st7735.st7735_write_str(0, 20, displayBuf, Font_7x10, ST7735_YELLOW);
            sprintf(displayBuf, "Missed: %d", missedPackets);
            st7735.st7735_write_str(0, 40, displayBuf, Font_7x10, ST7735_YELLOW);
            sprintf(displayBuf, "Last RSSI:%d", lastRssi);
            st7735.st7735_write_str(0, 60, displayBuf, Font_7x10, ST7735_YELLOW);

            // Send Packet
            sprintf(txpacket, "SF_TEST:%d:%d:%d", currentSF, currentPacketIndex, PACKETS_PER_SF);
            Serial.printf("Sending: %s\n", txpacket);
            
            txStartTime = millis(); 
            Radio.Send((uint8_t *)txpacket, strlen(txpacket));
            state = STATE_WAIT_ACK;
            break;

        case STATE_WAIT_ACK:
            Radio.IrqProcess(); 
            if (millis() - txStartTime > SAFETY_TIMEOUT) {
                Serial.println("!!! WATCHDOG TIMEOUT !!!");
                Radio.Sleep();
                missedPackets++;
                currentPacketIndex++;
                if (currentPacketIndex > PACKETS_PER_SF) state = STATE_NEXT_SF;
                else state = STATE_TX;
            }
            break;

        case STATE_NEXT_SF:
            totalMissed += missedPackets;
            if (currentSF > SF_MIN) {
                currentSF--;
                currentPacketIndex = 1;
                missedPackets = 0;
                ConfigRadio(currentSF);
                delay(1000); 
                state = STATE_TX;
            } else {
                // Finished screen
                st7735.st7735_fill_screen(ST7735_WHITE);
                st7735.st7735_write_str(0, 0, "TEST COMPLETE", Font_7x10, ST7735_YELLOW);
                sprintf(displayBuf, "Tot Missed: %d", totalMissed);
                st7735.st7735_write_str(0, 20, displayBuf, Font_7x10, ST7735_YELLOW);
                state = STATE_STOP;
            }
            break;

        case STATE_STOP:
            Radio.Sleep();
            break;
    }
}

// --- INTERRUPT HANDLERS ---
void OnTxDone(void) { Radio.Rx(RX_TIMEOUT_VALUE); }
void OnTxTimeout(void) { Radio.Sleep(); state = STATE_TX; }
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    Radio.Sleep();
    lastRssi = rssi;
    currentPacketIndex++;
    if (currentPacketIndex > PACKETS_PER_SF) state = STATE_NEXT_SF;
    else state = STATE_TX;
}
void OnRxTimeout(void) {
    Radio.Sleep();
    missedPackets++;
    currentPacketIndex++; 
    if (currentPacketIndex > PACKETS_PER_SF) state = STATE_NEXT_SF;
    else state = STATE_TX;
}