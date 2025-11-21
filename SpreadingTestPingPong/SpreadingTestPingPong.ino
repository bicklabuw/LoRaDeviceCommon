/* Heltec Automation Spreading Factor Test (Ping-Pong Style)
 * Modified for Bidirectional Verification and RSSI Threshold
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_st7735.h"

#define RF_FREQUENCY        915000000 // Hz
#define TX_OUTPUT_POWER     14        // dBm
#define LORA_BANDWIDTH      0         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_CODINGRATE     1         // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE    2000      // Increased timeout for ACK wait
#define BUFFER_SIZE         50        // Increased buffer size for ACK data
#define SF_MIN              7
#define SF_MAX              12
#define WORST_CASE_RSSI     -90      // Stop test if RSSI is worse (lower) than this

HT_st7735 st7735;
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

int currentSF = SF_MAX;
int bestSF = SF_MAX;
int bestRSSI = -999;
int16_t txNumber = 0;
bool isSendingAck = false; // Flag to distinguish between Test Packet and ACK

typedef enum {
    LOWPOWER,
    STATE_RX,
    STATE_TX,
    STATE_WAIT_ACK, // New state to wait for reply
    STATE_STOP      // New state to stop test
} States_t;

States_t state = STATE_TX;
static RadioEvents_t RadioEvents;

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
    st7735.st7735_init();
    st7735.st7735_fill_screen(ST7735_BLACK);
    st7735.st7735_write_str(0, 0, "SF Test Init");
    
    txNumber = 0;
    bestRSSI = -999;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout; // Added RX Timeout handler

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    
    // Initial Config
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
        currentSF, LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
        true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, currentSF,
        LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
        LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
        0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
        
    state = STATE_TX;
    Serial.println("Ping-Pong SF Test Initialized");
}

void loop() {
    switch(state) {
        case STATE_TX:
            delay(1000); // Small delay before next test
            txNumber++;
            
            // Prepare Packet: "SF_TEST:SF:Count"
            sprintf(txpacket, "SF_TEST:%d:%d", currentSF, txNumber);
            
            st7735.st7735_fill_screen(ST7735_BLACK);
            st7735.st7735_write_str(0, 0, "TX SF" + String(currentSF));
            st7735.st7735_write_str(0, 20, txpacket);
            st7735.st7735_write_str(0, 40, "Best RSSI: " + String(bestRSSI));
            
            Serial.printf("Sending Test: %s\n", txpacket);
            
            // Ensure TX config matches current SF
            Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                currentSF, LORA_CODINGRATE,
                LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
            
            isSendingAck = false; // We are sending a TEST, not an ACK
            Radio.Send((uint8_t *)txpacket, strlen(txpacket));
            state = LOWPOWER;
            break;

        case STATE_WAIT_ACK:
            Serial.println("Waiting for ACK...");
            // Ensure RX config matches current SF
            Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, currentSF,
                LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
            
            Radio.Rx(RX_TIMEOUT_VALUE); // Wait with timeout
            state = LOWPOWER;
            break;

        case STATE_RX:
            Serial.println("Listening for SF_TEST...");
            Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, currentSF,
                LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
            
            Radio.Rx(0); // Continuous RX
            state = LOWPOWER;
            break;
            
        case STATE_STOP:
            // Test halted
            Radio.Sleep();
            break;

        case LOWPOWER:
            Radio.IrqProcess();
            break;

        default:
            break;
    }
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    Radio.Sleep();
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';
    Serial.printf("RX: %s | RSSI: %d\n", rxpacket, rssi);

    int sf_rec = 0; 
    int val2 = 0;

    // --- CASE 1: We are the SENDER, receiving an ACK ---
    if (strncmp(rxpacket, "ACK", 3) == 0) {
        // Parse: "ACK:SF:RemoteRSSI"
        if (sscanf(rxpacket, "ACK:%d:%d", &sf_rec, &val2) == 2) {
             Serial.printf("ACK Received. Local RSSI: %d, Remote RSSI: %d\n", rssi, val2);
             
             // Check Worst Case RSSI
             if (rssi < WORST_CASE_RSSI || val2 < WORST_CASE_RSSI) {
                 Serial.println("STOP: Signal too weak.");
                 st7735.st7735_fill_screen(ST7735_BLACK);
                 st7735.st7735_write_str(0, 0, "TEST FAIL");
                 st7735.st7735_write_str(0, 20, "RSSI LOW");
                 st7735.st7735_write_str(0, 40, "L:" + String(rssi) + " R:" + String(val2));
                 state = STATE_STOP;
                 return;
             }
             
             // Successful Round Trip!
             if (rssi > bestRSSI) { bestRSSI = rssi; bestSF = currentSF; }
             
             // Prepare for next SF
             if (currentSF > SF_MIN) {
                 currentSF--;
                 state = STATE_TX; // We drive the next test
             } else {
                 Serial.println("TEST COMPLETE");
                 st7735.st7735_fill_screen(ST7735_BLACK);
                 st7735.st7735_write_str(0, 0, "DONE");
                 state = STATE_STOP;
             }
        }
    }
    // --- CASE 2: We are the RECEIVER, receiving a TEST packet ---
    else if (strncmp(rxpacket, "SF_TEST", 7) == 0) {
        if (sscanf(rxpacket, "SF_TEST:%d:%d", &sf_rec, &val2) == 2) {
            Serial.printf("Test Packet Received. SF: %d\n", sf_rec);
            
            // Display info
            st7735.st7735_fill_screen(ST7735_BLACK);
            st7735.st7735_write_str(0, 0, "RX SF" + String(sf_rec));
            st7735.st7735_write_str(0, 20, "RSSI: " + String(rssi));
            
            // Send ACK containing the RSSI we just saw
            char ack[BUFFER_SIZE];
            sprintf(ack, "ACK:%d:%d", sf_rec, rssi);
            
            Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                sf_rec, LORA_CODINGRATE,
                LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
            
            isSendingAck = true; // Flag that we are sending an ACK
            Radio.Send((uint8_t *)ack, strlen(ack));
            
            // Move to next SF *after* sending ACK
            if (currentSF > SF_MIN) {
                currentSF--;
            }
            // We stay in RX mode to wait for the next packet (logic handled in OnTxDone)
        }
    }
}

void OnTxDone(void) {
    Radio.Sleep();
    Serial.println("TX done.");
    
    if (isSendingAck) {
        // We just sent an ACK, so we should go back to listening for the next SF test
        state = STATE_RX; 
    } else {
        // We just sent a TEST packet, so we must wait for the ACK
        state = STATE_WAIT_ACK;
    }
}

void OnTxTimeout(void) {
    Radio.Sleep();
    Serial.println("TX Timeout");
    state = STATE_TX; // Retry
}

void OnRxTimeout(void) {
    Radio.Sleep();
    if (state == STATE_WAIT_ACK) {
        Serial.println("ACK Timeout - Retry same SF");
        state = STATE_TX; // Retry the same SF
    } else {
        Serial.println("RX Timeout - Continue Listening");
        Radio.Rx(0); // Keep listening
    }
}