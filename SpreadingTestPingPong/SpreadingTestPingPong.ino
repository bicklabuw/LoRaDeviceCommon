/* Heltec Automation Spreading Factor Test (Packet Error Rate & Reliability)
 * * Features:
 * 1. Sends a batch of packets (PACKETS_PER_SF) at each SF level.
 * 2. Counts missed packets (PER) rather than just RSSI.
 * 3. "Last Packet Retry" logic ensures devices don't get out of sync.
 * 4. Worst-case RSSI safety stop.
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_st7735.h"

// --- CONFIGURATION ---
#define RF_FREQUENCY        915000000 // Hz
#define TX_OUTPUT_POWER     14        // dBm
#define PACKETS_PER_SF      10        // How many packets to test per SF
#define WORST_CASE_RSSI     -115      // Stop test if signal drops below this
#define MAX_LAST_PKT_RETRY  5         // How many times to retry the FINAL packet of a batch
#define SF_MIN              7
#define SF_MAX              12

// LoRa Settings
#define LORA_BANDWIDTH      0         // [0: 125 kHz]
#define LORA_CODINGRATE     1         // [1: 4/5]
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE    2000      // ms to wait for ACK
#define BUFFER_SIZE         60

HT_st7735 st7735;
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

// Test State Variables
int currentSF = SF_MAX;
int bestRSSI = -999;
int currentPacketIndex = 1; 
int missedPackets = 0;
int retryCount = 0;           // Counter for last-packet retries
bool isSendingAck = false;    // Flag to distinguish TX type

typedef enum {
    LOWPOWER,
    STATE_RX,
    STATE_TX,
    STATE_WAIT_ACK,
    STATE_STOP
} States_t;

States_t state = STATE_TX;
static RadioEvents_t RadioEvents;

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
    st7735.st7735_init();
    st7735.st7735_fill_screen(ST7735_WHITE);
    st7735.st7735_write_str(0, 0, "PER Test Init");

    currentSF = SF_MAX;
    currentPacketIndex = 1;
    missedPackets = 0;
    retryCount = 0;
    bestRSSI = -999;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;

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
    Serial.println("Test Initialized. Starting at SF12.");
}

void loop() {
    switch(state) {
        case STATE_TX:
            delay(500); 

            // --- 1. Check if Batch is Complete ---
            if (currentPacketIndex > PACKETS_PER_SF) {
                Serial.printf("SF%d Finished. Missed: %d\n", currentSF, missedPackets);
                
                // Decrease SF
                if (currentSF > SF_MIN) {
                    currentSF--;
                    currentPacketIndex = 1;
                    missedPackets = 0;
                    retryCount = 0;
                    bestRSSI = -999;

                    // Reconfigure Radio
                    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                        currentSF, LORA_CODINGRATE,
                        LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                        true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
                    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, currentSF,
                        LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                        LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                        0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
                } else {
                    // Test Complete
                    st7735.st7735_fill_screen(ST7735_WHITE);
                    st7735.st7735_write_str(0, 0, "TEST COMPLETE");
                    state = STATE_STOP;
                    break;
                }
            }

            // --- 2. Prepare Packet ---
            // Format: "SF_TEST:SF:CurrentIndex:TotalPackets"
            sprintf(txpacket, "SF_TEST:%d:%d:%d", currentSF, currentPacketIndex, PACKETS_PER_SF);
            
            // Display Status
            st7735.st7735_fill_screen(ST7735_WHITE);
            st7735.st7735_write_str(0, 0, "TX SF" + String(currentSF));
            st7735.st7735_write_str(0, 20, "Pkt: " + String(currentPacketIndex) + "/" + String(PACKETS_PER_SF));
            st7735.st7735_write_str(0, 40, "Missed: " + String(missedPackets));
            st7735.st7735_write_str(0, 60, "BestRSSI: " + String(bestRSSI));
            if(retryCount > 0) st7735.st7735_write_str(0, 80, "Retrying...");

            Serial.printf("Sending: %s\n", txpacket);
            
            isSendingAck = false; 
            Radio.Send((uint8_t *)txpacket, strlen(txpacket));
            state = LOWPOWER; // Wait for OnTxDone
            break;

        case STATE_WAIT_ACK:
            Serial.println("Waiting for ACK...");
            Radio.Rx(RX_TIMEOUT_VALUE);
            state = LOWPOWER;
            break;

        case STATE_RX:
            Radio.Rx(0); // Continuous Listen
            state = LOWPOWER;
            break;

        case STATE_STOP:
            Radio.Sleep();
            break;

        case LOWPOWER:
            Radio.IrqProcess();
            break;
            
        default: break;
    }
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    Radio.Sleep();
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';
    Serial.printf("RX: %s | RSSI: %d\n", rxpacket, rssi);

    int sf_rec = 0, val1 = 0, val2 = 0;

    // --- SENDER LOGIC (Received ACK) ---
    if (strncmp(rxpacket, "ACK", 3) == 0) {
        if (sscanf(rxpacket, "ACK:%d:%d", &sf_rec, &val1) == 2) {
            // Check Safety Threshold
            if (rssi < WORST_CASE_RSSI || val1 < WORST_CASE_RSSI) {
                st7735.st7735_fill_screen(ST7735_WHITE);
                st7735.st7735_write_str(0, 0, "FAIL: RSSI LOW");
                state = STATE_STOP;
                return;
            }
            if (rssi > bestRSSI) bestRSSI = rssi;

            // Success! Move to next packet
            retryCount = 0; // Clear retry counter
            currentPacketIndex++; 
            state = STATE_TX; 
        }
    }
    // --- RECEIVER LOGIC (Received TEST Packet) ---
    else if (strncmp(rxpacket, "SF_TEST", 7) == 0) {
        if (sscanf(rxpacket, "SF_TEST:%d:%d:%d", &sf_rec, &val1, &val2) == 3) {
            Serial.printf("Received Packet %d/%d at SF%d\n", val1, val2, sf_rec);

            st7735.st7735_fill_screen(ST7735_WHITE);
            st7735.st7735_write_str(0, 0, "RX SF" + String(sf_rec));
            st7735.st7735_write_str(0, 20, "Pkt: " + String(val1) + "/" + String(val2));
            st7735.st7735_write_str(0, 40, "RSSI: " + String(rssi));

            // Send ACK
            char ack[BUFFER_SIZE];
            sprintf(ack, "ACK:%d:%d", sf_rec, rssi);
            
            // Ensure TX config matches received SF for the reply
            Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                sf_rec, LORA_CODINGRATE,
                LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

            isSendingAck = true;
            Radio.Send((uint8_t *)ack, strlen(ack));

            // CHECK: Is this the LAST packet?
            if (val1 == val2) {
               // If we received the last packet (e.g. 10 of 10), we prepare to switch SF.
               // However, we DO NOT switch yet. We wait for OnTxDone to ensure ACK is sent.
               if (currentSF > SF_MIN) {
                   Serial.println("Last packet! Flagging SF switch.");
                   currentSF--; 
                   // Radio config will be updated in OnTxDone
               }
            }
        }
    }
}

void OnTxDone(void) {
    Radio.Sleep();
    if (isSendingAck) {
        // We are Receiver, just sent ACK.
        // Now we update the Radio Config (in case SF changed) and go back to listening
        Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, currentSF,
            LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
            LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
            0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
        state = STATE_RX;
    } else {
        // We are Sender, just sent TEST. Wait for ACK.
        state = STATE_WAIT_ACK;
    }
}

void OnTxTimeout(void) {
    Radio.Sleep();
    state = STATE_TX; // Hardware TX error, just retry
}

void OnRxTimeout(void) {
    Radio.Sleep();
    if (state == STATE_WAIT_ACK) {
        Serial.println("ACK Timeout.");
        
        // --- THE FIX: Last Packet Handling ---
        if (currentPacketIndex == PACKETS_PER_SF) {
            // This was the critical last packet.
            if (retryCount < MAX_LAST_PKT_RETRY) {
                retryCount++;
                Serial.printf("Retrying Last Packet (Attempt %d)...\n", retryCount);
                state = STATE_TX; // Resend the SAME packet (do not increment index)
                return;
            } else {
                Serial.println("Max Retries reached on Last Packet. Forcing next SF.");
                missedPackets++;
                currentPacketIndex++; // Force move to next SF
                state = STATE_TX;
            }
        } else {
            // Normal packet: Count as miss and move on
            missedPackets++;
            currentPacketIndex++;
            state = STATE_TX;
        }
    } else {
        // Receiver RX timeout: just keep listening
        Radio.Rx(0);
    }
}