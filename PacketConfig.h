#ifndef PACKET_CONFIG_H
#define PACKET_CONFIG_H

// --- LORA RADIO SETTINGS ---
#define RF_FREQUENCY        915000000 // Hz
#define TX_OUTPUT_POWER     14        // dBm
#define LORA_BANDWIDTH      0         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_CODINGRATE     1         // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LEN   8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

// --- TIMING CONFIGURATION ---
#define RX_TIMEOUT_VALUE    2000   // Hardware timeout (ms)
#define PACKET_WAIT_DELAY   500    // Delay between generic packets
#define DISCOVERY_TIMEOUT   5000   // Time (ms) to listen for responses after a scan request
#define BACKOFF_MAX_DELAY   2000   // Max random delay (ms) for nodes to reply to avoid collisions

// --- ADDRESS DEFINITIONS ---
#define BASE_STATION_ID     0x00
#define BROADCAST_ADDR      0xFF

// --- PACKET TYPES ---
// 1. Discovery Phase
#define PKT_DISCOVERY_REQ   0x01  // "Who is out there?" (Broadcast or Direct)
#define PKT_DISCOVERY_RESP  0x02  // "I am here, ID: X" (Response to REQ)

// 2. Link Testing (Optional/Future)
#define PKT_SF_TEST         0x03  // SF Test Packet

// 3. Remote Scanning / BFS Logic
#define PKT_REPORT_NODE     0x04  // "I found a neighbor: ID Y" (Node reporting to Base)
#define PKT_SCAN_CMD        0x05  // "Start scanning your area now" (Base commanding Node)
#define PKT_SCAN_DONE       0x06  // "I finished scanning my area" (Node reporting completion)

#endif // PACKET_CONFIG_H