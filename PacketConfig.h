#ifndef PACKET_CONFIG_H
#define PACKET_CONFIG_H

// --- USER CONFIGURATION ---
// Total time (in ms) for the Base Station to wait for replies.
// Lora Nodes will randomly backoff between 100ms and this value.
#define DISCOVERY_WINDOW_MS 10000  
#define DISCOVERY_ROUNDS    5       

// --- LORA SETTINGS ---
#define RF_FREQUENCY        915000000 
#define TX_OUTPUT_POWER     14        
#define LORA_BANDWIDTH      0         
#define LORA_CODINGRATE     1         
#define LORA_PREAMBLE_LEN   8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

// --- DATA RATES ---
const int DATA_RATES[] = { 5468, 3125, 1757, 976, 537, 293 }; 

// --- TIMING ---
#define RX_TIMEOUT_VALUE    2000
#define REMOTE_OP_TIMEOUT   120000  

// --- PROTOCOL ---
#define PKT_DISCOVERY_REQ   0x01
#define PKT_DISCOVERY_RESP  0x02
#define PKT_SF_TEST         0x03
#define PKT_SCAN_CMD        0x04
#define PKT_REPORT_NODE     0x05
#define PKT_SCAN_DONE       0x06

#endif