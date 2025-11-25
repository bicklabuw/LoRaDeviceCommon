/* HELTEC PER TEST - RECEIVER (SLAVE) - INVERTED COLORS */
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_st7735.h"

#define RF_FREQUENCY      915000000
#define TX_OUTPUT_POWER   14
#define SF_RESET_TIMEOUT  30000 // Reset to SF12 if silence for 30s

HT_st7735 st7735;
char rxpacket[60];
char ackpacket[60];
char dispBuf[50];

volatile int currentSF = 12;
volatile int rxRssi = 0;
volatile uint32_t lastRxTime = 0;
volatile bool updateScreen = false;

static RadioEvents_t RadioEvents;

void ConfigRadio(int sf) {
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, 0, sf, 1, 8, false, true, 0, 0, false, 3000);
    Radio.SetRxConfig(MODEM_LORA, 0, sf, 1, 0, 8, 0, false, 0, true, 0, 0, false, true);
    Serial.printf("Radio Set to SF%d\n", sf);
}

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    st7735.st7735_init();
    // Inverted BG
    st7735.st7735_fill_screen(ST7735_WHITE);
    // Inverted Text (Yellow appears Blue)
    st7735.st7735_write_str(0, 0, "RX MODE START", Font_7x10, ST7735_YELLOW);

    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxDone = OnTxDone;
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    
    ConfigRadio(currentSF);
    Radio.Rx(0); // Continuous RX
    lastRxTime = millis();
}

void loop() {
    if (updateScreen) {
        updateScreen = false;
        // Inverted BG
        st7735.st7735_fill_screen(ST7735_WHITE);
        // Inverted Text (Yellow appears blue)
        sprintf(dispBuf, "RX SF: %d", currentSF);
        st7735.st7735_write_str(0, 0, dispBuf, Font_7x10, ST7735_YELLOW);
        sprintf(dispBuf, "RSSI: %d", rxRssi);
        st7735.st7735_write_str(0, 20, dispBuf, Font_7x10, ST7735_YELLOW);
        st7735.st7735_write_str(0, 40, "Sending ACK...", Font_7x10, ST7735_YELLOW);
    }

    // Safety Reset
    if (millis() - lastRxTime > SF_RESET_TIMEOUT && currentSF != 12) {
        currentSF = 12;
        ConfigRadio(currentSF);
        // Inverted RED Alert screen (Use CYAN BG, BLACK Text)
        st7735.st7735_fill_screen(ST7735_CYAN); 
        st7735.st7735_write_str(0, 0, "TIMEOUT RESET", Font_7x10, ST7735_BLACK);
        st7735.st7735_write_str(0, 20, "Back to SF12", Font_7x10, ST7735_BLACK);
        lastRxTime = millis();
        Radio.Rx(0);
    }

    Radio.IrqProcess();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';
    rxRssi = rssi;
    lastRxTime = millis();

    int senderSF, pktIdx, totalPkts;
    if (sscanf(rxpacket, "SF_TEST:%d:%d:%d", &senderSF, &pktIdx, &totalPkts) == 3) {
        sprintf(ackpacket, "ACK:%d:%d", currentSF, rssi);
        Radio.Send((uint8_t *)ackpacket, strlen(ackpacket));
        updateScreen = true;
    } else {
        Radio.Rx(0);
    }
}

void OnTxDone(void) {
    int senderSF, pktIdx, totalPkts;
    sscanf(rxpacket, "SF_TEST:%d:%d:%d", &senderSF, &pktIdx, &totalPkts);
    if (pktIdx == totalPkts && currentSF > 7) {
        currentSF--;
        ConfigRadio(currentSF);
    }
    Radio.Rx(0);
}