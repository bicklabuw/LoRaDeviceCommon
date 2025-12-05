#ifndef PACKET_CONFIG_H
#define PACKET_CONFIG_H

// --- LORA SETTINGS ---
#define RF_FREQUENCY 915000000 // Hz
#define TX_OUTPUT_POWER 14     // dBm
#define LORA_BANDWIDTH 0       // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_CODINGRATE 1      // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LEN 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

// --- TIMING ---
#define RX_TIMEOUT_VALUE 2000
#define PACKET_WAIT_DELAY 500  // Delay between SF test packets
#define DISCOVERY_TIMEOUT 5000 // How long to listen for discovery responses
#define BACKOFF_MAX_DELAY 2000 // Max random delay for nodes to reply to discovery

// --- PACKET TYPES ---
#define PKT_DISCOVERY_REQ 0x01  // "Who is out there?"
#define PKT_DISCOVERY_RESP 0x02 // "I am here, ID: X"
#define PKT_SF_TEST 0x03        // SF Test Packet
#define PKT_SF_ACK 0x04         // SF Test Acknowledgement
#define PKT_CMD_SEARCH 0x05     // "Node X, search your neighbors now"
#define PKT_REPORT_NODE 0x06    // "Base, I found Node Y with Capacity Z"
#define PKT_SEARCH_DONE 0x07    // "Base, I finished searching"

// --- ADDRESSING ---
#define BROADCAST_ADDR 0xFF
#define BASE_STATION_ID 0x00

// --- PACKET STRUCTURE ---
// [0]: Destination ID
// [1]: Sender ID
// [2]: Packet Type
// [3]: Data 1 (e.g., SF level or target ID)
// [4]: Data 2 (e.g., RSSI or Capacity)
// [5+]: Payload (Optional)

#endif