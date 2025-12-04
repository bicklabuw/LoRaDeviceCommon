/*
 * Heltec LoRa Mesh-Style Discovery and Spreading Factor Test
 *
 * This single application allows multiple devices to discover each other sequentially.
 *
 * --- OPERATION ---
 * 1. Flash this code onto all your Heltec LoRa v3 devices.
 * 2. On startup, each device enters IDLE mode.
 * 3. Short press the user button to change the device's ID (1-254).
 * 4. Long press the user button on ONE device to make it the "Base Node".
 *
 * --- PROCESS ---
 * - The Base Node broadcasts a Discovery Request.
 * - Nearby nodes not yet in the "seen" list will reply after a random delay.
 * - The Base Node pairs with the first responder and begins an SF test (SF12 down to SF7).
 * - It records the best SF (lowest number) and worst RSSI for that link.
 * - Upon completion, it commands the tested node to become the new "Discoverer".
 * - The process repeats, with each new node finding the next, until no new nodes are found.
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_st7735.h"

// --- LORA CONFIGURATION ---
#define RF_FREQUENCY          915000000 // Hz
#define TX_OUTPUT_POWER       14        // dBm
#define LORA_PREAMBLE_LENGTH  8
#define LORA_SYMBOL_TIMEOUT   0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON  false

// --- TEST PARAMETERS ---
#define PACKETS_PER_SF      10
#define SF_MIN              7
#define SF_MAX              12
#define RX_TIMEOUT_VALUE    2000 // ms, for waiting for an ACK
#define DISCOVERY_TIMEOUT   10000 // ms, for waiting for a discovery response

// --- DEVICE & PINS ---
#define MAX_SEEN_NODES      30
#define BUTTON_PIN          0

// --- GLOBAL OBJECTS ---
HT_st7735 st7735;
static RadioEvents_t RadioEvents;

// --- STATE MACHINE (REFACTORED FOR MESH) ---
typedef enum {
    ROLE_SETUP,
    ROLE_SENDER,
    ROLE_RECEIVER
} DeviceRole_t;

typedef enum {
    SENDER_IDLE,
    SENDER_DISCOVERY_BROADCAST, // New state to broadcast and find nodes
    SENDER_WAIT_FOR_REQUEST,    // New state to listen for TEST_REQ from nodes
    SENDER_TEST_IN_PROGRESS,    // Encapsulates the entire SF test for one node
    SENDER_FINALIZE_TEST,       // Stores result and decides what to do next
    SENDER_DONE
} SenderState_t;

typedef enum {
    RECEIVER_IDLE,          // Passively listening for a discovery broadcast
    RECEIVER_BACKOFF,       // Heard a broadcast, waiting for a random time
    RECEIVER_AWAITING_TEST, // Sent a TEST_REQ, waiting for sender to start
    RECEIVER_TEST_IN_PROGRESS // Actively being tested by a sender
} ReceiverState_t;

volatile DeviceRole_t deviceRole = ROLE_SETUP;
volatile SenderState_t senderState = SENDER_IDLE;
volatile ReceiverState_t receiverState = RECEIVER_IDLE;


// --- DEVICE & NETWORK STATE ---
uint8_t deviceId = 1;
uint8_t partnerId = 0;

// --- MESH TOPOLOGY ---
struct NodeTestResult {
  uint8_t nodeId;
  int bestSF;
};
NodeTestResult testResults[MAX_SEEN_NODES];
int resultCount = 0;


// --- SPREADING FACTOR TEST STATE ---
volatile int currentSF = SF_MAX;
volatile int currentPacketIndex = 1;
volatile int missedPackets = 0;
int bestSF = -1;
int worstRssi = -999;
unsigned long stateTimer = 0;
unsigned long backoffEndTime = 0;

// --- TIMING & BUTTONS ---
unsigned long buttonPressTime = 0;
volatile bool buttonPressed = false;

// --- BUFFERS ---
char txBuffer[128];
char rxBuffer[128];
char displayBuffer[50];

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnTxDone(void);
void OnRxTimeout(void);
void OnTxTimeout(void);
void handleButton();
void updateDisplay();
void configRadio(int sf);


void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    st7735.st7735_init();
    st7735.st7735_fill_screen(ST7735_BLACK);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(BUTTON_PIN, handleButton, CHANGE);

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);

    randomSeed(analogRead(0));

    deviceRole = ROLE_SETUP;
    updateDisplay();
}

void loop() {
    if (deviceRole == ROLE_SETUP) {
        Radio.Sleep();
        // --- Button Handling Logic for Setup ---
        if (buttonPressed) {
            bool isLongPress = (digitalRead(BUTTON_PIN) == LOW) && (millis() - buttonPressTime > 1000);
            bool isShortPress = (digitalRead(BUTTON_PIN) == HIGH);

            if (isLongPress) {
                if (deviceId == 1) {
                    Serial.println("Long press on ID 1. Becoming SENDER.");
                    deviceRole = ROLE_SENDER;
                    senderState = SENDER_DISCOVERY_BROADCAST; // Start discovery
                    configRadio(SF_MAX);
                } else {
                    Serial.printf("Long press on ID %d. Becoming RECEIVER.\n", deviceId);
                    deviceRole = ROLE_RECEIVER;
                    receiverState = RECEIVER_IDLE;
                    configRadio(SF_MAX); // Listen on robust SF
                    Radio.Rx(0); // Continuous receive
                }
                updateDisplay();
                buttonPressed = false;
            } else if (isShortPress) {
                deviceId++;
                if (deviceId > 254) deviceId = 1;
                Serial.printf("Short press. New ID: %d\n", deviceId);
                updateDisplay();
                buttonPressed = false;
            }
        }
    }
    else if (deviceRole == ROLE_SENDER) {
        switch(senderState) {
            case SENDER_DISCOVERY_BROADCAST:
                Serial.println("SENDER: Broadcasting for discovery...");
                configRadio(SF_MAX);
                sprintf(txBuffer, "SND_SYNC|%d|%d", deviceId, SF_MAX);
                Radio.Send((uint8_t *)txBuffer, strlen(txBuffer));

                senderState = SENDER_WAIT_FOR_REQUEST;
                stateTimer = millis(); // Start the 10-second discovery window
                updateDisplay();
                break;

            case SENDER_WAIT_FOR_REQUEST:
                Radio.IrqProcess();
                // After 10 seconds without a new test, discovery phase ends
                if (millis() - stateTimer > 10000) {
                    Serial.println("SENDER: Discovery window closed.");
                    senderState = SENDER_DONE; // End of all operations for now
                    updateDisplay();
                }
                break;

            case SENDER_TEST_IN_PROGRESS:
                 // This state now functions like the old SENDER_TX/WAIT_ACK/NEXT_SF loop
                 // It is advanced by the OnRxDone/OnRxTimeout callbacks
                 Radio.IrqProcess();
                 break;

            case SENDER_FINALIZE_TEST:
                Serial.printf("SENDER: Test with %d complete. Best SF: %d\n", partnerId, bestSF);
                
                // Store the result
                if (resultCount < MAX_SEEN_NODES) {
                    testResults[resultCount].nodeId = partnerId;
                    testResults[resultCount].bestSF = bestSF;
                    resultCount++;
                }

                // Tell the tested node we are done
                sprintf(txBuffer, "SND_DONE|%d", deviceId);
                Radio.Send((uint8_t*)txBuffer, strlen(txBuffer));

                // Go back to discovering the next node
                senderState = SENDER_DISCOVERY_BROADCAST;
                updateDisplay();
                break;

            case SENDER_DONE:
            case SENDER_IDLE:
                 Radio.Sleep();
                 break;
        }
    }
    else if (deviceRole == ROLE_RECEIVER) {
        Radio.IrqProcess();

        if (receiverState == RECEIVER_BACKOFF) {
            if (millis() > backoffEndTime) {
                Serial.println("Backoff complete. Sending test request.");
                sprintf(txBuffer, "TEST_REQ|%d", deviceId);
                Radio.Send((uint8_t *)txBuffer, strlen(txBuffer));
                
                receiverState = RECEIVER_AWAITING_TEST;
                stateTimer = millis(); // Start a new timer to wait for the test to start
                updateDisplay();
            }
        }
        else if (receiverState == RECEIVER_AWAITING_TEST) {
            // If we've been waiting too long for the sender to start the test,
            // assume it's busy with another node. Go back to idle and wait for the next discovery broadcast.
            if (millis() - stateTimer > 5000) { // 5-second timeout
                Serial.println("Awaiting test timeout. Sender is busy. Returning to idle.");
                receiverState = RECEIVER_IDLE;
                Radio.Rx(0); // Go back to continuous listen
                updateDisplay();
            }
        }
    }
}

void OnTxDone(void) {
    if (deviceRole == ROLE_SENDER) {
        if (senderState == SENDER_WAIT_FOR_REQUEST) {
            // Discovery broadcast was sent. Now listen for requests.
             Radio.Rx(0);
        } else if (senderState == SENDER_TEST_IN_PROGRESS) {
            // Test data was sent, wait for ACK
            Radio.Rx(RX_TIMEOUT_VALUE);
        }
    } else if (deviceRole == ROLE_RECEIVER) {
        // RECEIVER: This is called after we send an ACK during a test, or after a TEST_REQ
        if (receiverState == RECEIVER_TEST_IN_PROGRESS) {
            // We just sent an ACK during a test. Check if it's time to switch SFs.
            if (currentPacketIndex == PACKETS_PER_SF && currentSF > SF_MIN) {
                currentSF--;
                configRadio(currentSF);
            }
        }
        // For all cases (ACK sent, TEST_REQ sent), go back to listening.
        Radio.Rx(0);
    }
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    memcpy(rxBuffer, payload, size);
    rxBuffer[size] = '\0';
    Serial.printf("RX: %s (RSSI: %d)\n", rxBuffer, rssi);

    if (deviceRole == ROLE_SENDER) {
        uint8_t senderId;
        if (senderState == SENDER_WAIT_FOR_REQUEST) {
            // We are discovering and heard a TEST_REQ. Let's start the test.
            if (sscanf(rxBuffer, "TEST_REQ|%hhu", &senderId) == 1) {
                partnerId = senderId;
                Serial.printf("SENDER: Received test request from %d. Starting test.\n", partnerId);

                // Reset test state variables
                currentSF = SF_MAX;
                currentPacketIndex = 1;
                missedPackets = 0;
                bestSF = -1;
                worstRssi = -999;
                
                senderState = SENDER_TEST_IN_PROGRESS;

                // Start the test by sending the first data packet
                configRadio(currentSF);
                sprintf(txBuffer, "SND_DATA|%d|%d|%d|%d", deviceId, currentSF, currentPacketIndex, PACKETS_PER_SF);
                Radio.Send((uint8_t *)txBuffer, strlen(txBuffer));
                updateDisplay();
            }
        } else if (senderState == SENDER_TEST_IN_PROGRESS) {
            // We are in a test and received an ACK
            if (sscanf(rxBuffer, "ACK|%hhu", &senderId) == 1 && senderId == partnerId) {
                currentPacketIndex++;
                if (currentPacketIndex > PACKETS_PER_SF) {
                    // Move to next SF
                     if (missedPackets == 0) bestSF = currentSF;
                     currentSF--;
                     if (currentSF < SF_MIN) {
                         senderState = SENDER_FINALIZE_TEST;
                     } else {
                         currentPacketIndex = 1;
                         missedPackets = 0;
                         configRadio(currentSF);
                         sprintf(txBuffer, "SND_DATA|%d|%d|%d|%d", deviceId, currentSF, currentPacketIndex, PACKETS_PER_SF);
                         Radio.Send((uint8_t *)txBuffer, strlen(txBuffer));
                     }
                } else {
                    // Send next packet
                    sprintf(txBuffer, "SND_DATA|%d|%d|%d|%d", deviceId, currentSF, currentPacketIndex, PACKETS_PER_SF);
                    Radio.Send((uint8_t *)txBuffer, strlen(txBuffer));
                }
                updateDisplay();
            }
        }
    } else if (deviceRole == ROLE_RECEIVER) {
        uint8_t senderId;
        int receivedSF;
        int pkt_idx, total_pkts;

        if (receiverState == RECEIVER_IDLE) {
            // Look for the initial discovery broadcast
            if (sscanf(rxBuffer, "SND_SYNC|%hhu|%d", &senderId, &receivedSF) == 2) {
                Serial.println("Heard discovery broadcast, starting backoff.");
                receiverState = RECEIVER_BACKOFF;
                backoffEndTime = millis() + random(500, 3000); // Wait 0.5-3 seconds
                updateDisplay();
            }
        } else if (receiverState == RECEIVER_AWAITING_TEST) {
            // We've sent our TEST_REQ, now we are waiting for the test to start.
            // A DATA packet with our ID means the test is for us.
             if (sscanf(rxBuffer, "SND_DATA|%hhu", &senderId) == 1) {
                partnerId = senderId;
                Serial.println("Sender has accepted our test request.");
                receiverState = RECEIVER_TEST_IN_PROGRESS;
                sscanf(rxBuffer, "SND_DATA|%hhu|%d|%d|%d", &senderId, &receivedSF, &pkt_idx, &total_pkts);
                currentPacketIndex = pkt_idx;
                if (receivedSF != currentSF) {
                    currentSF = receivedSF;
                    configRadio(currentSF);
                }
                sprintf(txBuffer, "ACK|%d", deviceId);
                Radio.Send((uint8_t *)txBuffer, strlen(txBuffer));
             }
        } else if (receiverState == RECEIVER_TEST_IN_PROGRESS) {
            // We are in a test, so we expect DATA packets.
            if (sscanf(rxBuffer, "SND_DATA|%hhu|%d|%d|%d", &senderId, &receivedSF, &pkt_idx, &total_pkts) == 4) {
                partnerId = senderId;
                currentPacketIndex = pkt_idx;
                if(receivedSF != currentSF) {
                    configRadio(receivedSF);
                    currentSF = receivedSF;
                }
                sprintf(txBuffer, "ACK|%d", deviceId);
                Radio.Send((uint8_t *)txBuffer, strlen(txBuffer));
            } else if (sscanf(rxBuffer, "SND_DONE|%hhu", &senderId) == 1) {
                Serial.println("Test is done. Returning to idle.");
                receiverState = RECEIVER_IDLE;
                currentSF = SF_MAX;
                configRadio(currentSF);
                Radio.Rx(0);
            }
        }
        updateDisplay();
    }
}

void OnTxTimeout(void) {
    Radio.Sleep();
    Serial.println("TX Timeout!");
    if (deviceRole == ROLE_SENDER) {
        senderState = SENDER_DISCOVERY_BROADCAST; // Retry discovery
    }
}

void OnRxTimeout(void) {
    Radio.Sleep();
    if (deviceRole == ROLE_SENDER) {
        if(senderState == SENDER_TEST_IN_PROGRESS) {
            Serial.println("RX Timeout (Missed ACK)");
            missedPackets++;
            currentPacketIndex++;
             if (currentPacketIndex > PACKETS_PER_SF) {
                if (missedPackets == 0) bestSF = currentSF;
                 currentSF--;
                 if (currentSF < SF_MIN) {
                     senderState = SENDER_FINALIZE_TEST;
                 } else {
                     currentPacketIndex = 1;
                     missedPackets = 0;
                     configRadio(currentSF);
                 }
            }
             // Need to re-trigger send in main loop. Best way is to just go back to test state.
            senderState = SENDER_TEST_IN_PROGRESS; 
        }
    }
}

void configRadio(int sf) {
    Radio.Sleep();
    // From working SpreadingTest example
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, 0, sf, 1, 8, false, true, 0, 0, false, 3000);
    Radio.SetRxConfig(MODEM_LORA, 0, sf, 1, 0, 8, 0, false, 0, true, 0, 0, false, true);
    Serial.printf("Radio configured for SF%d\n", sf);
}

void handleButton() {
    // ISR should only set flags, not perform complex logic.
    if (digitalRead(BUTTON_PIN) == LOW) {
        // Capture press time only on the falling edge
        if (!buttonPressed) {
            buttonPressTime = millis();
            buttonPressed = true;
        }
    }
    // Note: Release is handled in the main loop polling digitalRead
}

void updateDisplay() {
    st7735.st7735_fill_screen(ST7735_BLACK);
    sprintf(displayBuffer, "ID: %d", deviceId);
    st7735.st7735_write_str(0, 0, displayBuffer, Font_7x10, ST7735_CYAN);

    switch(deviceRole) {
        case ROLE_SETUP:
            st7735.st7735_write_str(0, 20, "SETUP", Font_11x18, ST7735_WHITE);
            st7735.st7735_write_str(0, 40, "Short: Change ID", Font_7x10, ST7735_GREEN);
            st7735.st7735_write_str(0, 55, "Long: Set Role", Font_7x10, ST7735_GREEN);
            break;
        case ROLE_SENDER:
            st7735.st7735_write_str(0, 15, "SENDER", Font_11x18, ST7735_YELLOW);
            if (senderState == SENDER_DONE) {
                st7735.st7735_write_str(0, 30, "Discovery Done", Font_7x10, ST7735_GREEN);
            } else if (senderState == SENDER_WAIT_FOR_REQUEST) {
                st7735.st7735_write_str(0, 30, "Discovering...", Font_7x10, ST7735_WHITE);
            } else if (senderState == SENDER_TEST_IN_PROGRESS) {
                sprintf(displayBuffer, "Testing: %d", partnerId);
                st7735.st7735_write_str(0, 30, displayBuffer, Font_7x10, ST7735_YELLOW);
                sprintf(displayBuffer, "SF:%d Pkt:%d", currentSF, currentPacketIndex);
                st7735.st7735_write_str(0, 45, displayBuffer, Font_7x10, ST7735_WHITE);
            }
            // Display results
            for (int i = 0; i < resultCount; i++) {
                sprintf(displayBuffer, "%d:%d", testResults[i].nodeId, testResults[i].bestSF);
                st7735.st7735_write_str(i*30, 60, displayBuffer, Font_7x10, ST7735_CYAN);
            }
            break;
        case ROLE_RECEIVER:
            st7735.st7735_write_str(0, 15, "RECEIVER", Font_11x18, ST7735_GREEN);
            switch(receiverState) {
                case RECEIVER_IDLE:
                    sprintf(displayBuffer, "Listening SF%d", currentSF);
                    st7735.st7735_write_str(0, 35, displayBuffer, Font_7x10, ST7735_WHITE);
                    break;
                case RECEIVER_BACKOFF:
                    st7735.st7735_write_str(0, 35, "Heard beacon", Font_7x10, ST7735_WHITE);
                    sprintf(displayBuffer, "Backoff...");
                    st7735.st7735_write_str(0, 50, displayBuffer, Font_7x10, ST7735_YELLOW);
                    break;
                case RECEIVER_AWAITING_TEST:
                    st7735.st7735_write_str(0, 35, "Test requested", Font_7x10, ST7735_WHITE);
                    sprintf(displayBuffer, "Awaiting turn...");
                    st7735.st7735_write_str(0, 50, displayBuffer, Font_7x10, ST7735_YELLOW);
                    break;
                case RECEIVER_TEST_IN_PROGRESS:
                     st7735.st7735_write_str(0, 35, "TEST IN PROGRESS", Font_7x10, ST7735_CYAN);
                     sprintf(displayBuffer, "From: %d SF:%d", partnerId, currentSF);
                     st7735.st7735_write_str(0, 50, displayBuffer, Font_7x10, ST7735_WHITE);
                    break;
            }
            break;
    }
}