#include "src/arduino-dw1000-ng/src/DW1000Ng.hpp"
#include "src/arduino-dw1000-ng/src/DW1000NgUtils.hpp"

// Number of anchors used in the system
#define ANCHOR_NUM 4
#define ANCHOR_ID 0 // <--- CHANGE THIS FOR EACH ANCHOR (0, 1, 2, or 3)

const uint8_t PIN_RST = 27;  
const uint8_t PIN_IRQ = 34;  
const uint8_t PIN_SS = 4;    

/* Payload format (CIR and receiving timestamps of messages from other anchors)
|---CIR(4 byte)---|--RxTime(5 byte)--|******|---CIR(4 byte)---|--RxTime(5 byte)--|-IDX(1 byte)-|
|---------------------------13 byte * (Anchor Number-1)--------------------------|----1 byte---| 
*/
#define SINGLE_LEN 13
#define POA_LEN 4
#define END_LEN 0

#define ANCHOR_LISTEN 0
#define ANCHOR_SEND 2

#define ALL_MSG_COMMON_LEN 10
#define CIR_LEN 3
#define ALL_MSG_SN_IDX 2
#define SENDING_TX_TS_IDX 10
#define NET_PANID 0xF0F2
#define RX_ANT_DLY 0
#define TX_ANT_DLY 32880
#define DELAY_TIME 5000
#define DELAY_TIME_TURN 5000
#define RX_AFTER_TX_DELAY 450
#define RX_TIMEOUT 10000
#define FRAME_LEN_MAX 127
#define UUS_TO_DWT_TIME 65536

uint8_t rx_buffer[FRAME_LEN_MAX];
uint32_t status_reg = 0;
uint16_t frame_len;

uint8_t msg_payload[(ANCHOR_NUM - 1) * SINGLE_LEN + END_LEN + 4];
static uint8_t msg_common[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21 };
static uint8_t sending_msg[sizeof(msg_common) + sizeof(msg_payload)];

device_configuration_t CONFIG_1 = {
  true,   
  false,  
  true,   
  true,   // frameCheck ON to reject RF Noise
  false,  
  SFDMode::DECAWAVE_SFD,
  Channel::CHANNEL_1,
  DataRate::RATE_6800KBPS,
  PulseFrequency::FREQ_64MHZ,
  PreambleLength::LEN_64,
  PreambleCode::CODE_9
};

device_configuration_t CONFIG_2 = {
  true,   
  false,  
  true,   
  true,   // frameCheck ON to reject RF Noise
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

enum TX_CONFIG { CH_1, CH_2, CH_3, CH_7 };

void set_tx_config(TX_CONFIG config) {
  switch (config) {
    case TX_CONFIG::CH_1:
      DW1000Ng::setTXPower(0x17171717);
      DW1000Ng::setTCPGDelay(0xc9);
      break;
    case TX_CONFIG::CH_2:
      DW1000Ng::setTXPower(0x15151515);
      DW1000Ng::setTCPGDelay(0xc9);
      break;
    case TX_CONFIG::CH_3:
      DW1000Ng::setTXPower(0x2b2b2b2b);
      DW1000Ng::setTCPGDelay(0xc5);
      break;
    case TX_CONFIG::CH_7:
      DW1000Ng::setTXPower(0xD1D1D1D1);
      DW1000Ng::setTCPGDelay(0x93);
      break;
  }
}

void dw_init() {
  DW1000Ng::initializeNoInterrupt(PIN_SS);
  DW1000Ng::applyConfiguration(CONFIG_1);
  set_tx_config(TX_CONFIG::CH_2);
  DW1000Ng::setDeviceAddress(1);
  DW1000Ng::setNetworkId(NET_PANID);
  DW1000Ng::setRxAntennaDelay(RX_ANT_DLY);
  DW1000Ng::setTxAntennaDelay(TX_ANT_DLY);
  DW1000Ng::applyInterruptConfiguration(interruptConfig);
  DW1000Ng::enableDebounceClock();
  DW1000Ng::enableLedBlinking();
}

static uint8_t anchor_state = ANCHOR_LISTEN;
uint8_t current_idx = 0;
bool is_last_anchor = false;
uint8_t ret = 0;
uint8_t ledPin = 2;

static uint8_t frame_seq_nb = 0;
uint64_t rx_ts;
static uint64_t tx_ts;
static uint8_t cir_buffer[4 * CIR_LEN + 1];
static uint16_t current_freq = 1;
static uint16_t err_num = 0;
static uint16_t rec_cnt = 0;

uint8_t phase_cal;
uint16_t maxGC;
uint8_t rxPC;

static uint8_t current_tx;
static uint8_t offset;

void final_msg_set_ts(uint8_t *ts_field, uint64_t ts) {
  for (int i = 0; i < 5; i++) {
    ts_field[i] = (uint8_t)ts;
    ts >>= 8;
  }
}

void setup() {
  pinMode(0, ledPin);
  msg_common[8] = ANCHOR_ID;

  dw_init();

  for (int i = 0; i < 10; i++) {
    sending_msg[i] = msg_common[i];
  }

  if (ANCHOR_ID == 0) {
    anchor_state = ANCHOR_SEND;
    sending_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

    DW1000Ng::setTransmitData(sending_msg, sizeof(sending_msg));
    DW1000Ng::setWait4Response(RX_AFTER_TX_DELAY);
    DW1000Ng::setPreambleDetectionTimeout(0);
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(RX_TIMEOUT);
    DW1000Ng::startTransmit(); 
  }
}

void loop() {
  if (anchor_state == ANCHOR_LISTEN) {
    DW1000Ng::setPreambleDetectionTimeout(0);
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(RX_TIMEOUT);
    DW1000Ng::startReceive();
  }

  bool isDone = false;
  while (!((isDone = DW1000Ng::isReceiveDone()) || DW1000Ng::isReceiveFailed() || DW1000Ng::isReceiveTimeout())) {
  }

  if (anchor_state == ANCHOR_SEND) {
    anchor_state = ANCHOR_LISTEN;
  }

  if (isDone) {
    frame_len = DW1000Ng::getReceivedDataLength();
    DW1000Ng::clearTransmitStatus(); 
    DW1000Ng::clearReceiveStatus();

    if (frame_len <= FRAME_LEN_MAX) {
      DW1000Ng::getReceivedData(rx_buffer, frame_len);
    }

    if (frame_len < 10 || rx_buffer[8] >= ANCHOR_NUM || rx_buffer[8] == ANCHOR_ID) {
        isDone = false; 
    }
  }

  if (isDone) {
    err_num = 0;
    rec_cnt++;

    current_tx = rx_buffer[8];
    frame_seq_nb = rx_buffer[ALL_MSG_SN_IDX];
    rx_ts = DW1000Ng::getReceiveTimestamp();

    DW1000Ng::readRCPhase(&phase_cal); // Check if this is the right byte (instead of getRawTemp?)
    maxGC = DW1000Ng::getCirPwrBytes();
    rxPC = DW1000Ng::getPreambleAccumulationCount();

    uint16_t fp_index = DW1000Ng::getFPPathIdx() >> 6; 
    DW1000Ng::getAccData(cir_buffer, CIR_LEN * 4 + 1, (fp_index)*4);

    if (current_tx < ANCHOR_ID) {
      offset = SINGLE_LEN * current_tx + POA_LEN + 4;
    } else {
      offset = SINGLE_LEN * (current_tx - 1) + POA_LEN + 4;
    }
    final_msg_set_ts(&msg_payload[offset], rx_ts);

    if (((current_tx + 1) % ANCHOR_NUM == ANCHOR_ID) && !((0 == ANCHOR_ID) && (frame_seq_nb % 2 == 1))) {
      uint32_t delay_time;

      if (0 == ANCHOR_ID) {
        delay_time = DELAY_TIME_TURN;
        frame_seq_nb++;
      } else {
        delay_time = DELAY_TIME;
      }

      uint64_t tx_time = rx_ts + ((uint64_t)delay_time * UUS_TO_DWT_TIME);
      byte futureTimeBytes[LENGTH_TIMESTAMP];
      DW1000NgUtils::writeValueToBytes(futureTimeBytes, tx_time, LENGTH_TIMESTAMP);
      DW1000Ng::setDelayedTRX(futureTimeBytes);

      sending_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
      DW1000Ng::setTransmitData(sending_msg, sizeof(sending_msg));
      DW1000Ng::setReceiveFrameWaitTimeoutPeriod(RX_TIMEOUT);
      DW1000Ng::setWait4Response(RX_AFTER_TX_DELAY);

      if ((ANCHOR_ID + 1 == ANCHOR_NUM) && (frame_seq_nb % 2 == 1)) {
        DW1000Ng::setWait4Response(0);  
        DW1000Ng::startTransmit(TransmitMode::DELAYED);
        is_last_anchor = 1;
      } else {
        DW1000Ng::startTransmit(TransmitMode::DELAYED);
        anchor_state = ANCHOR_SEND;
      }

      if (ANCHOR_NUM - 1 == ANCHOR_ID) {
        is_last_anchor = 1;
      }
    }

    if (current_tx < ANCHOR_ID) {
      memcpy(msg_payload + SINGLE_LEN * current_tx, cir_buffer + 1 + 4, 4);
      msg_payload[SINGLE_LEN * current_tx + 4] = phase_cal;
      msg_payload[SINGLE_LEN * current_tx + 5] = (uint8_t)rxPC;
      msg_payload[SINGLE_LEN * current_tx + 6] = (uint8_t)(maxGC >> 8);
      msg_payload[SINGLE_LEN * current_tx + 7] = (uint8_t)maxGC;
    } else {
      memcpy(msg_payload + SINGLE_LEN * (current_tx - 1), cir_buffer + 1 + 4, 4);
      msg_payload[SINGLE_LEN * (current_tx - 1) + 4] = phase_cal;
      msg_payload[SINGLE_LEN * (current_tx - 1) + 5] = (uint8_t)rxPC;
      msg_payload[SINGLE_LEN * (current_tx - 1) + 6] = (uint8_t)(maxGC >> 8);
      msg_payload[SINGLE_LEN * (current_tx - 1) + 7] = (uint8_t)maxGC;
    }

    if ((current_tx + 1 == ANCHOR_NUM) || ((current_tx + 2 == ANCHOR_NUM) && (ANCHOR_ID + 1 == ANCHOR_NUM) && is_last_anchor)) {
      is_last_anchor = 0;

      for (int i = 10; i < sizeof(sending_msg); i++) {
        sending_msg[i] = msg_payload[i - 10];
      }
      memset(msg_payload, 0, (ANCHOR_NUM - 1) * SINGLE_LEN + END_LEN + 4);

      if (frame_seq_nb % 2 == 1 && anchor_state != ANCHOR_SEND) {
        if (ANCHOR_ID + 1 == ANCHOR_NUM) {
          while (!(DW1000Ng::isTransmitDone())) {
              yield(); 
          };
        }

        if (frame_seq_nb % 4 == 1) {
          current_freq = 3;
          DW1000Ng::forceTRxOff();
          DW1000Ng::applyConfiguration(CONFIG_2);
          set_tx_config(TX_CONFIG::CH_3);
        } else if (frame_seq_nb % 4 == 3) {
          current_freq = 1;
          DW1000Ng::forceTRxOff();
          DW1000Ng::applyConfiguration(CONFIG_1);
          set_tx_config(TX_CONFIG::CH_2);
        }

        if (ANCHOR_ID + 1 == ANCHOR_NUM) {
          DW1000Ng::setPreambleDetectionTimeout(0);
          DW1000Ng::setReceiveFrameWaitTimeoutPeriod(RX_TIMEOUT);
          DW1000Ng::startReceive();
          anchor_state = ANCHOR_SEND;
        }

        if (0 == ANCHOR_ID) {
          uint32_t delay_time = DELAY_TIME_TURN + 600;
          frame_seq_nb++;

          uint64_t tx_time = rx_ts + ((uint64_t)delay_time * UUS_TO_DWT_TIME);
          byte futureTimeBytes[LENGTH_TIMESTAMP];
          DW1000NgUtils::writeValueToBytes(futureTimeBytes, tx_time, LENGTH_TIMESTAMP);
          DW1000Ng::setDelayedTRX(futureTimeBytes);

          sending_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
          DW1000Ng::setTransmitData(sending_msg, sizeof(sending_msg));
          DW1000Ng::setReceiveFrameWaitTimeoutPeriod(RX_TIMEOUT);
          DW1000Ng::setWait4Response(RX_AFTER_TX_DELAY);

          DW1000Ng::startTransmit(TransmitMode::DELAYED);
          anchor_state = ANCHOR_SEND;
        }
      }
    }
  } else {
    err_num++;

    if (0 == ANCHOR_ID) {
      if (err_num >= 1) {
        rec_cnt = 0;
        err_num = 0;
        frame_seq_nb = 0;
        current_freq = 1;
        DW1000Ng::forceTRxOff();
        DW1000Ng::applyConfiguration(CONFIG_1);
        set_tx_config(TX_CONFIG::CH_2);
      } else {
        delay(3);
      }

      DW1000Ng::clearReceiveFailedStatus();
      DW1000Ng::clearReceiveTimeoutStatus();

      anchor_state = ANCHOR_SEND;
      sending_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
      DW1000Ng::setTransmitData(sending_msg, sizeof(sending_msg));
      DW1000Ng::setWait4Response(RX_AFTER_TX_DELAY);
      DW1000Ng::setReceiveFrameWaitTimeoutPeriod(RX_TIMEOUT);
      DW1000Ng::setPreambleDetectionTimeout(0);

      DW1000Ng::startTransmit();
    } else {
      if (err_num >= 1) {
        rec_cnt = 0;
        err_num = 0;
        current_freq = 1;
        DW1000Ng::forceTRxOff();
        DW1000Ng::applyConfiguration(CONFIG_1);
        set_tx_config(TX_CONFIG::CH_2);
      }
      DW1000Ng::clearReceiveFailedStatus();
      DW1000Ng::clearReceiveTimeoutStatus();
    }
  }
}