#include "src/arduino-dw1000-ng/src/DW1000Ng.hpp"
#include "src/arduino-dw1000-ng/src/DW1000NgUtils.hpp"

// CONFIGURATION
#define ANCHOR_NUM 4 // # anchors in the system

const uint8_t PIN_RST = 27;  
const uint8_t PIN_IRQ = 34;  
const uint8_t PIN_SS = 4;    

#define CIR_LEN 3
#define ALL_MSG_SN_IDX 2
#define SINGLE_LEN 13
#define RX_ANT_DLY 32880
#define TX_ANT_DLY 32880
#define NET_PANID 0xF0F2
#define RX_TIMEOUT 10000
#define FRAME_LEN_MAX 127

// --- ANCHOR-OVERHEARING STRUCTURE TO INCLUDE WITH TAG ---
struct AnchorOverhearingPacket {
    uint16_t real; 
    uint16_t imag;
    uint8_t phase;
    uint8_t rxpc;
    uint16_t maxgc;
    uint64_t rxtime;
};

struct AnchorOverhearing {
    struct AnchorOverhearingPacket aopacket[ANCHOR_NUM - 1]; 
};

// --- MATLAB-COMPATIBLE LOGGING STRUCTURE FOR TAG ---
struct RoundLog {
    uint8_t seq;
    bool readyToPrint;
    bool printed;
    bool has_data[ANCHOR_NUM];
    AnchorOverhearing ao[ANCHOR_NUM]; // <--- Data overheard by anchors
    uint16_t real[ANCHOR_NUM];
    uint16_t imag[ANCHOR_NUM];
    uint8_t phase[ANCHOR_NUM];
    uint8_t rxpc[ANCHOR_NUM];
    uint16_t maxgc[ANCHOR_NUM];
    uint64_t rxtime[ANCHOR_NUM];
    int32_t ci[ANCHOR_NUM]; // <--- Carrier Integrator for Tag
};
RoundLog roundLogs[16]; 
// ---------------------------------------------------

uint8_t rx_buffer[FRAME_LEN_MAX];
uint16_t frame_len;
static uint8_t cir_buffer[(4 * CIR_LEN + 1)];

static uint8_t frame_seq_nb = 0;
static uint16_t current_tx;
static uint16_t current_freq = 1;

device_configuration_t TAG_CH1 = {
  true,   
  false, 
  true,  
  true,   // frameCheck ON 
  false,  
  SFDMode::DECAWAVE_SFD,
  Channel::CHANNEL_1,
  DataRate::RATE_6800KBPS,
  PulseFrequency::FREQ_64MHZ,
  PreambleLength::LEN_64,
  PreambleCode::CODE_9 
};

device_configuration_t TAG_CH3 = {
  true,
  false,
  true,
  true,   // frameCheck ON
  false,
  SFDMode::DECAWAVE_SFD,
  Channel::CHANNEL_3,
  DataRate::RATE_6800KBPS,
  PulseFrequency::FREQ_64MHZ,
  PreambleLength::LEN_64,
  PreambleCode::CODE_9 
};

interrupt_configuration_t interruptConfig = {
  .interruptOnSent = false,                        
  .interruptOnReceived = true,                     
  .interruptOnReceiveFailed = true,                
  .interruptOnReceiveTimeout = true,               
  .interruptOnReceiveTimestampAvailable = false,   
  .interruptOnAutomaticAcknowledgeTrigger = false  
};

void dw_init() {
  Serial.setTxBufferSize(2048);
  Serial.begin(2000000);
  Serial.println(F("### DW1000Ng-arduino-TAG-Passive ###"));
  
  DW1000Ng::initializeNoInterrupt(PIN_SS);
  DW1000Ng::applyConfiguration(TAG_CH1);
  DW1000Ng::enableLedBlinking();
  DW1000Ng::setDeviceAddress(0x00); 
  DW1000Ng::setNetworkId(NET_PANID);
  DW1000Ng::setRxAntennaDelay(RX_ANT_DLY); 
  DW1000Ng::setTxAntennaDelay(TX_ANT_DLY); 
  DW1000Ng::applyInterruptConfiguration(interruptConfig);

  Serial.println(F("Committed configuration ..."));
}

uint8_t ledPin = 2;

void setup() {
  pinMode(0, ledPin);
  dw_init();

  for(int i = 0; i < 16; i++) {
      roundLogs[i].seq = 255;
      roundLogs[i].readyToPrint = false;
      roundLogs[i].printed = true; 
  }

  // START RECEIVER ONCE AT THE END OF SETUP
  DW1000Ng::setPreambleDetectionTimeout(0);
  DW1000Ng::setReceiveFrameWaitTimeoutPeriod(RX_TIMEOUT);
  DW1000Ng::startReceive();
}

void loop() {
  bool isDone = false;
  
  // Wait for an event
  while (!((isDone = DW1000Ng::isReceiveDone()) || DW1000Ng::isReceiveFailed() || DW1000Ng::isReceiveTimeout())) {
      yield(); 
  }

  if (isDone) {
      frame_len = DW1000Ng::getReceivedDataLength();
      if (frame_len <= FRAME_LEN_MAX) {
          DW1000Ng::getReceivedData(rx_buffer, frame_len);
      }

      if (frame_len < 10 || rx_buffer[8] >= ANCHOR_NUM) {
          isDone = false; 

        DW1000Ng::clearTransmitStatus();
        DW1000Ng::clearReceiveStatus(); 

        // --- BACKGROUND RECEIVE FIX ---
        // Instantly tell the DW1000 to start listening again! It will catch Anchor 1 
        // in the background while the ESP32 is busy doing Serial.printf!
        DW1000Ng::startReceive(); 
      }
  } else {
      // If packet failed/timeout, clear flags and restart receiver
      DW1000Ng::clearReceiveFailedStatus();
      DW1000Ng::clearReceiveTimeoutStatus();
      DW1000Ng::startReceive();
  }

  if (isDone) {
      current_tx = rx_buffer[8];
      frame_seq_nb = rx_buffer[ALL_MSG_SN_IDX];
      uint64_t rx_ts = DW1000Ng::getReceiveTimestamp(); 

      uint8_t phase_cal;
      DW1000Ng::readRCPhase(&phase_cal); 
      uint16_t maxGC = DW1000Ng::getCirPwrBytes();
      uint8_t rxPC = DW1000Ng::getPreambleAccumulationCount();
      int32_t carrier_integrator = DW1000Ng::getCarrierIntegrator();

      uint16_t fp_index = DW1000Ng::getFPPathIdx() >> 6;
      DW1000Ng::getAccData(cir_buffer, CIR_LEN * 4 + 1, (fp_index) * 4); 

    DW1000Ng::clearTransmitStatus();
    DW1000Ng::clearReceiveStatus(); 

    // --- BACKGROUND RECEIVE FIX ---
    // Instantly tell the DW1000 to start listening again! It will catch Anchor 1 
    // in the background while the ESP32 is busy doing Serial.printf!
    DW1000Ng::startReceive(); 

      // ----------------------------------------------------
      // --- POPULATE THE MATLAB LOG BUFFER FOR THIS ROUND --
      // ----------------------------------------------------
      uint8_t idx = frame_seq_nb % 16;
      if (roundLogs[idx].seq != frame_seq_nb) {
          uint8_t prev_idx = (frame_seq_nb + 15) % 16;
          if (roundLogs[prev_idx].seq == (uint8_t)(frame_seq_nb - 1)) {
              roundLogs[prev_idx].readyToPrint = true;
          }
          
          roundLogs[idx].seq = frame_seq_nb;
          roundLogs[idx].readyToPrint = false;
          roundLogs[idx].printed = false;
          for(int i = 0; i < ANCHOR_NUM; i++) roundLogs[idx].has_data[i] = false;
      }
      
      if (current_tx < ANCHOR_NUM) {
          roundLogs[idx].has_data[current_tx] = true;
          roundLogs[idx].real[current_tx] = cir_buffer[5] | (cir_buffer[6] << 8); 
          roundLogs[idx].imag[current_tx] = cir_buffer[7] | (cir_buffer[8] << 8); 
          roundLogs[idx].phase[current_tx] = phase_cal;
          roundLogs[idx].rxpc[current_tx] = rxPC;
          roundLogs[idx].maxgc[current_tx] = maxGC;
          roundLogs[idx].rxtime[current_tx] = rx_ts;
          roundLogs[idx].ci[current_tx] = carrier_integrator;
          
          for (int i = 0; i < ANCHOR_NUM - 1; i++) { 
            int rx_offset = 10 + (SINGLE_LEN * i);
            roundLogs[idx].ao[current_tx].aopacket[i].real = rx_buffer[rx_offset] | (rx_buffer[rx_offset + 1] << 8); 
            roundLogs[idx].ao[current_tx].aopacket[i].imag = rx_buffer[rx_offset + 2] | (rx_buffer[rx_offset + 3] << 8); 
            roundLogs[idx].ao[current_tx].aopacket[i].phase = rx_buffer[rx_offset + 4];
            roundLogs[idx].ao[current_tx].aopacket[i].rxpc = rx_buffer[rx_offset + 5];
            roundLogs[idx].ao[current_tx].aopacket[i].maxgc = (rx_buffer[rx_offset + 6] << 8) | rx_buffer[rx_offset + 7];
            
            uint64_t rx_time = 0;
            for (int j = 0; j < 5; j++) {
                rx_time |= ((uint64_t)rx_buffer[rx_offset + 8 + j] << (8 * j));
            }
            roundLogs[idx].ao[current_tx].aopacket[i].rxtime = rx_time;
          }
      }

      // ----------------------------------------------------
      // --- FREQUENCY HOPPING LOGIC ---
      // ----------------------------------------------------
      if (current_tx == ANCHOR_NUM - 1) {
          if (frame_seq_nb % 2 == 1) {
              if (frame_seq_nb % 4 == 1) {
                  current_freq = 3;
                  DW1000Ng::forceTRxOff();
                  DW1000Ng::applyConfiguration(TAG_CH3);
                  DW1000Ng::startReceive(); // Restart RX after hop!
              }
              else if (frame_seq_nb % 4 == 3) {
                  current_freq = 1;
                  DW1000Ng::forceTRxOff();
                  DW1000Ng::applyConfiguration(TAG_CH1);
                  DW1000Ng::startReceive(); // Restart RX after hop!
              }
          }
      }
  } 

  // ----------------------------------------------------
  // --- FLUSH COMPLETED ROUNDS TO SERIAL FOR MATLAB ----
  // ----------------------------------------------------
  for(int i = 0; i < 16; i++) {
      if (roundLogs[i].readyToPrint && !roundLogs[i].printed) {
          
          uint8_t outBuf[234]; // Exact size of one MULoc binary round
          uint16_t p = 0;

          // 1. Overheard Data from Anchors (160 bytes)
          for (int anc = 0; anc < ANCHOR_NUM; anc++) {
              if (roundLogs[i].has_data[anc]) {
                  for (int j = 0; j < ANCHOR_NUM - 1; j++) {
                      // real (16-bit little-endian)
                      outBuf[p++] = roundLogs[i].ao[anc].aopacket[j].real & 0xFF;
                      outBuf[p++] = (roundLogs[i].ao[anc].aopacket[j].real >> 8) & 0xFF;
                      // imag (16-bit little-endian)
                      outBuf[p++] = roundLogs[i].ao[anc].aopacket[j].imag & 0xFF;
                      outBuf[p++] = (roundLogs[i].ao[anc].aopacket[j].imag >> 8) & 0xFF;
                      // phase (8-bit)
                      outBuf[p++] = roundLogs[i].ao[anc].aopacket[j].phase;
                      // rxpc (8-bit)
                      outBuf[p++] = roundLogs[i].ao[anc].aopacket[j].rxpc;
                      // maxgc (16-bit big-endian)
                      outBuf[p++] = (roundLogs[i].ao[anc].aopacket[j].maxgc >> 8) & 0xFF;
                      outBuf[p++] = roundLogs[i].ao[anc].aopacket[j].maxgc & 0xFF;
                      // rxtime (40-bit little-endian)
                      uint64_t rt = roundLogs[i].ao[anc].aopacket[j].rxtime;
                      outBuf[p++] = rt & 0xFF;
                      outBuf[p++] = (rt >> 8) & 0xFF;
                      outBuf[p++] = (rt >> 16) & 0xFF;
                      outBuf[p++] = (rt >> 24) & 0xFF;
                      outBuf[p++] = (rt >> 32) & 0xFF;
                  }
              } else {
                  // Zero pad if we missed this anchor (3 targets * 13 bytes = 39 bytes)
                  for (int k = 0; k < 39; k++) outBuf[p++] = 0;
              }
              // Sequence number for this anchor group
              outBuf[p++] = roundLogs[i].seq;
          }

          // 2. Tag Data (68 bytes)
          for (int a = 0; a < ANCHOR_NUM; a++) {
              if (roundLogs[i].has_data[a]) {
                  // real (16-bit little-endian)
                  outBuf[p++] = roundLogs[i].real[a] & 0xFF;
                  outBuf[p++] = (roundLogs[i].real[a] >> 8) & 0xFF;
                  // imag (16-bit little-endian)
                  outBuf[p++] = roundLogs[i].imag[a] & 0xFF;
                  outBuf[p++] = (roundLogs[i].imag[a] >> 8) & 0xFF;
                  // phase (8-bit)
                  outBuf[p++] = roundLogs[i].phase[a];
                  // rxpc (8-bit)
                  outBuf[p++] = roundLogs[i].rxpc[a];
                  // maxgc (16-bit big-endian)
                  outBuf[p++] = (roundLogs[i].maxgc[a] >> 8) & 0xFF;
                  outBuf[p++] = roundLogs[i].maxgc[a] & 0xFF;
                  
                  // rxtime (40-bit BIG-ENDIAN! - Required by utils.py)
                  uint64_t rt = roundLogs[i].rxtime[a];
                  outBuf[p++] = (rt >> 32) & 0xFF;
                  outBuf[p++] = (rt >> 24) & 0xFF;
                  outBuf[p++] = (rt >> 16) & 0xFF;
                  outBuf[p++] = (rt >> 8) & 0xFF;
                  outBuf[p++] = rt & 0xFF;
                  
                  // ci - carrier integrator (32-bit big-endian)
                  uint32_t ci = (uint32_t)roundLogs[i].ci[a];
                  outBuf[p++] = (ci >> 24) & 0xFF;
                  outBuf[p++] = (ci >> 16) & 0xFF;
                  outBuf[p++] = (ci >> 8) & 0xFF;
                  outBuf[p++] = ci & 0xFF;
              } else {
                  // Zero pad if we missed the tag data (17 bytes)
                  for (int k = 0; k < 17; k++) outBuf[p++] = 0;
              }
          }
          
          // 3. Final sequence byte for the Tag data group (1 byte)
          outBuf[p++] = roundLogs[i].seq;

          // 4. Footer / Sync Prefix (5 bytes)
          // The Python script consumes these 5 bytes via "curLen += 5" to keep frames aligned
          outBuf[p++] = 0x07;
          outBuf[p++] = 0x06;
          outBuf[p++] = 0x05;
          outBuf[p++] = 0x04;
          outBuf[p++] = 0x03;

          // Blast the binary data array over the Serial port
          Serial.write(outBuf, p);
          
          roundLogs[i].printed = true;
      }
  }
}