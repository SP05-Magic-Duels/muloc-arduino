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
  Serial.begin(921600);
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

      DW1000Ng::clearTransmitStatus();
      DW1000Ng::clearReceiveStatus(); 

      // --- BACKGROUND RECEIVE FIX ---
      // Instantly tell the DW1000 to start listening again! It will catch Anchor 1 
      // in the background while the ESP32 is busy doing Serial.printf!
      DW1000Ng::startReceive(); 

      if (frame_len < 10 || rx_buffer[8] >= ANCHOR_NUM) {
          isDone = false; 
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
          
          for (int anc = 0; anc < ANCHOR_NUM; anc++) {
              if (roundLogs[i].has_data[anc]) {
                  for (int j = 0; j < ANCHOR_NUM - 1; j++) {
                      int target_id = (j < anc) ? j : j + 1;
                      Serial.printf("%04x,%04x,%02x,%02x,%04x,%010llx,%d,",
                          roundLogs[i].ao[anc].aopacket[j].real,
                          roundLogs[i].ao[anc].aopacket[j].imag,
                          roundLogs[i].ao[anc].aopacket[j].phase,
                          roundLogs[i].ao[anc].aopacket[j].rxpc,
                          roundLogs[i].ao[anc].aopacket[j].maxgc,
                          roundLogs[i].ao[anc].aopacket[j].rxtime,
                          target_id);
                  }
              } else {
                  for (int j = 0; j < ANCHOR_NUM - 1; j++) {
                      int target_id = (j < anc) ? j : j + 1;
                      Serial.printf("0000,0000,00,00,0000,0000000000,%d,", target_id);
                  }
              }
              Serial.printf("%02x\n", roundLogs[i].seq);
          }

          for (int a = 0; a < ANCHOR_NUM; a++) {
              if (roundLogs[i].has_data[a]) {
                  Serial.printf("%04x,%04x,%02x,%02x,%04x,%010llx,%08x,%d,",
                      roundLogs[i].real[a], roundLogs[i].imag[a], roundLogs[i].phase[a],
                      roundLogs[i].rxpc[a], roundLogs[i].maxgc[a], roundLogs[i].rxtime[a],
                      roundLogs[i].ci[a], a);
              } else {
                  Serial.printf("0000,0000,00,00,0000,0000000000,00000000,%d,", a);
              }
          }
          Serial.printf("%02x\n", roundLogs[i].seq);
          
          roundLogs[i].printed = true;
      }
  }
}