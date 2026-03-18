/*
 * MIT License
 * 
 * Copyright (c) 2018 Michele Biondi, Andrea Salvatori
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file BasicReceiver.ino
 * Use this to test simple sender/receiver functionality with two
 * DW1000:: Complements the "BasicSender" example sketch.
 * 
 * @todo
 *  - move strings to flash (less RAM consumption)
 *  
 */

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>

// Debugging
#define TRACE_ENABLE 0

#if (TRACE_ENABLE == 1)
#define DEBUG_LOG_TRACE(msg, var) Serial.print(msg); Serial.println(var);
#else
#define DEBUG_LOG_TRACE(msg, var) {};
#endif

// Number of anchors used in the system
#define ANCHOR_NUM 3
#define ANCHOR_ID 2

const uint8_t PIN_RST = 27;  // reset pin
const uint8_t PIN_IRQ = 34;  // irq pin
const uint8_t PIN_SS = 4;    // spi select pin

/* Payload format (CIR and receiving timestamps of messages from other anchors)

|---CIR(4 byte)---|--RxTime(5 byte)--|******|---CIR(4 byte)---|--RxTime(5 byte)--|-IDX(1 byte)-|
|---------------------------13 byte * (Anchor Number-1)--------------------------|----1 byte---| 
*/
#define SINGLE_LEN 13
#define POA_LEN 4
#define END_LEN 0

#define ANCHOR_LISTEN 0
#define ANCHOR_SEND 2

/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
#define CIR_LEN 3
/* Indexes to access some of the fields in the frames defined above. */
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
/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 ? and 1 ? = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536


struct LogData {
    uint8_t anchor;
    uint8_t seq;
    uint8_t phase;
    uint16_t rxpc;
};
LogData logBuffer[10]; // Buffer for 10 messages
uint8_t logIndex = 0;

uint8_t rx_buffer[FRAME_LEN_MAX];
uint32_t status_reg = 0;
uint16_t frame_len;

uint8_t msg_payload[(ANCHOR_NUM - 1) * SINGLE_LEN + END_LEN + 4];
static uint8_t msg_common[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21 };
static uint8_t sending_msg[sizeof(msg_common) + sizeof(msg_payload)];

device_configuration_t CONFIG_1 = {
  true,   // extendedFrameLength
  false,  // receiverAutoReenable (unsure)
  true,   // smartPower
  true,  // frameCheck (unsure)
  false,  // nlos (unsure)
  SFDMode::DECAWAVE_SFD,
  Channel::CHANNEL_1,
  DataRate::RATE_6800KBPS,
  PulseFrequency::FREQ_64MHZ,
  PreambleLength::LEN_64,
  PreambleCode::CODE_9
};

// The only meaningful difference between this and CONFIG_1 is using CHANNEL_3
device_configuration_t CONFIG_2 = {
  true,   // extendedFrameLength
  false,  // receiverAutoReenable (unsure)
  true,   // smartPower
  true,  // frameCheck (unsure)
  false,  // nlos (unsure)
  SFDMode::DECAWAVE_SFD,
  Channel::CHANNEL_3,
  DataRate::RATE_6800KBPS,
  PulseFrequency::FREQ_64MHZ,
  PreambleLength::LEN_64,
  PreambleCode::CODE_9
};

interrupt_configuration_t interruptConfig = {
  .interruptOnSent = false,                        // No DWT_INT_TFRS
  .interruptOnReceived = true,                     // DWT_INT_RFCG
  .interruptOnReceiveFailed = true,                // DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE
  .interruptOnReceiveTimeout = true,               // DWT_INT_RFTO
  .interruptOnReceiveTimestampAvailable = false,   // No specific flag present (usually implied by Rx Good)
  .interruptOnAutomaticAcknowledgeTrigger = false  // No DWT_INT_AAT
};

enum TX_CONFIG {
  CH_1,
  CH_2,
  CH_3,
  CH_7
};

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
  // DEBUG monitoring
  Serial.begin(921600);
  Serial.println(F("### DW1000Ng-arduino-receiver-test ###"));
  // initialize the driver
  DW1000Ng::initializeNoInterrupt(PIN_SS);
  Serial.println(F("DW1000Ng initialized ..."));

  DW1000Ng::applyConfiguration(CONFIG_1);
  set_tx_config(TX_CONFIG::CH_2);

  DW1000Ng::enableLedBlinking();

  // TODO: make sure no addr conflicts?
  DW1000Ng::setDeviceAddress(1);
  DW1000Ng::setNetworkId(NET_PANID);

  DW1000Ng::setRxAntennaDelay(RX_ANT_DLY);
  DW1000Ng::setTxAntennaDelay(TX_ANT_DLY);

  // TODO: library != exact set of bits in original firmware
  DW1000Ng::applyInterruptConfiguration(interruptConfig);

  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000Ng::getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: ");
  Serial.println(msg);
  DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: ");
  Serial.println(msg);
  DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: ");
  Serial.println(msg);
  DW1000Ng::getPrintableDeviceMode(msg);
  Serial.print("Device mode: ");
  Serial.println(msg);
}

static uint8_t anchor_state = ANCHOR_LISTEN;

// The index of anchor that is currently sending message
uint8_t current_idx = 0;
// True or False
bool is_last_anchor = false;
uint8_t ret = 0;
uint8_t ledPin = 2;

/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Timestamps of frames transmission/reception. */
uint64_t rx_ts;
static uint64_t tx_ts;

static uint8_t cir_buffer[4 * CIR_LEN + 1];

static uint16_t current_freq = 1;

// Anchor 0 start error number
static uint16_t err_num = 0;

// Anchor 0 start error number
static uint16_t rec_cnt = 0;

/* Phase value measured from SFD */
uint8_t phase_cal;

uint16_t maxGC;
uint8_t rxPC;

static uint8_t current_tx;
static uint8_t offset;

void final_msg_set_ts(uint8_t *ts_field, uint64_t ts) {
  int i;
  for (i = 0; i < 5; i++) {
    ts_field[i] = (uint8_t)ts;
    ts >>= 8;
  }
}

void setup() {
  pinMode(0, ledPin);
  msg_common[8] = ANCHOR_ID;

  dw_init();

  // Write the common segments of UWB message into tx buffer
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
    DEBUG_LOG_TRACE("Anchor ID is 0, state is SEND. About to start transmit.", "")
    DW1000Ng::startTransmit();  // TODO: missing flag DWT_RESPONSE_EXPECTED. looks like the lib auto sets it if you call wait4resp?
  }
}

void loop() {
  if (anchor_state == ANCHOR_LISTEN) {
    DW1000Ng::setPreambleDetectionTimeout(0);
    /* Clear reception timeout to start next ranging process. */
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(RX_TIMEOUT);
    /* Activate reception immediately. */
    DW1000Ng::startReceive();
  }

  // Waiting for reception completion
  bool isDone = false;
  while (!((isDone = DW1000Ng::isReceiveDone()) || DW1000Ng::isReceiveFailed() || DW1000Ng::isReceiveTimeout())) {
  }

  // If the anchor have received the message, then change it to listening state
  if (anchor_state == ANCHOR_SEND) {
    anchor_state = ANCHOR_LISTEN;
  }

  if (isDone) {
    // DEBUG_LOG_TRACE("rx nb ", frame_seq_nb)
    if (logIndex < 10) {
      logBuffer[logIndex].anchor = current_tx;
      logBuffer[logIndex].seq = frame_seq_nb;
      logBuffer[logIndex].phase = phase_cal;
      logBuffer[logIndex].rxpc = rxPC;
      logIndex++;
    }
    err_num = 0;
    rec_cnt++;

    frame_len = DW1000Ng::getReceivedDataLength();
    DW1000Ng::clearTransmitStatus();  // TODO: they only clear TXFRS, this clears more bits
    DW1000Ng::clearReceiveStatus();

    if (frame_len <= FRAME_LEN_MAX) {
      DW1000Ng::getReceivedData(rx_buffer, frame_len);
    }

    current_tx = rx_buffer[8];
    frame_seq_nb = rx_buffer[ALL_MSG_SN_IDX];
    rx_ts = DW1000Ng::getReceiveTimestamp();

    // Copy the receiving timestamps into buffer
    if (current_tx < ANCHOR_ID) {
      offset = SINGLE_LEN * current_tx + POA_LEN + 4;
    } else {
      // UWB anchors cannot receive the messages sent by themself
      offset = SINGLE_LEN * (current_tx - 1) + POA_LEN + 4;
    }
    final_msg_set_ts(&msg_payload[offset], rx_ts);

    // It's our turn to send a message. We should send later if we are anchor 0 and it's time for hooping
    if (((current_tx + 1) % ANCHOR_NUM == ANCHOR_ID) && !((0 == ANCHOR_ID) && (frame_seq_nb % 2 == 1))) {
      // Time delay between the messages from different anchors
      uint32_t delay_time;

      // If we are anchor 0, the frame number should +1 and the delay time should be set to DELAY_TIME_TURN
      if (0 == ANCHOR_ID) {
        delay_time = DELAY_TIME_TURN;
        frame_seq_nb++;
      } else {
        delay_time = DELAY_TIME;
      }

      // The tx time is set to "delay_time" seconds after the reception of the last message using delayed transmission
      uint64_t tx_time = rx_ts + ((uint64_t)delay_time * UUS_TO_DWT_TIME);
      byte futureTimeBytes[LENGTH_TIMESTAMP]; // LENGTH_TIMESTAMP is 5 bytes
      DW1000NgUtils::writeValueToBytes(futureTimeBytes, tx_time, LENGTH_TIMESTAMP);
      DW1000Ng::setDelayedTRX(futureTimeBytes);

      // Config the frame sequence number
      sending_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

      // Write tx data into tx buffer
      DW1000Ng::setTransmitData(sending_msg, sizeof(sending_msg));
      DW1000Ng::setReceiveFrameWaitTimeoutPeriod(RX_TIMEOUT);
      DW1000Ng::setWait4Response(RX_AFTER_TX_DELAY);

      if ((ANCHOR_ID + 1 == ANCHOR_NUM) && (frame_seq_nb % 2 == 1)) {
        // If we are the last anchor and should perform hopping, we should not start reception before hopping.
        DW1000Ng::setWait4Response(0);  // clears w4r so it won't automatically start listening
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

    phase_cal = DW1000Ng::getRawTemperature();  // TODO: verify that this is the right byte!
    maxGC = DW1000Ng::getCirPwrBytes();
    rxPC = DW1000Ng::getPreambleAccumulationCount();

    // Read the first path CIR and save it to local buffer
    uint16_t fp_index = DW1000Ng::getFPPathIdx();
    DW1000Ng::getAccData(cir_buffer, CIR_LEN * 4 + 1, (fp_index)*4);

    // Copy CIR to msg_payload
    if (current_tx < ANCHOR_ID) {
      memcpy(msg_payload + SINGLE_LEN * current_tx, cir_buffer + 1 + 4, 4);
    } else {
      memcpy(msg_payload + SINGLE_LEN * (current_tx - 1), cir_buffer + 1 + 4, 4);
    }

    // Copy phase_cal to msg_payload
    if (current_tx < ANCHOR_ID) {
      msg_payload[SINGLE_LEN * current_tx + 4] = phase_cal;
    } else {
      msg_payload[SINGLE_LEN * (current_tx - 1) + 4] = phase_cal;
    }

    // Copy rxPC to msg_payload
    if (current_tx < ANCHOR_ID) {
      msg_payload[SINGLE_LEN * current_tx + 5] = (uint8_t)rxPC;
    } else {
      msg_payload[SINGLE_LEN * (current_tx - 1) + 5] = (uint8_t)rxPC;
    }

    // Copy maxGC to msg_payload
    if (current_tx < ANCHOR_ID) {
      msg_payload[SINGLE_LEN * current_tx + 6] = (uint8_t)(maxGC >> 8);
      msg_payload[SINGLE_LEN * current_tx + 7] = (uint8_t)maxGC;
    } else {
      msg_payload[SINGLE_LEN * (current_tx - 1) + 6] = (uint8_t)(maxGC >> 8);
      msg_payload[SINGLE_LEN * (current_tx - 1) + 7] = (uint8_t)maxGC;
    }

    if ((current_tx + 1 == ANCHOR_NUM) || ((current_tx + 2 == ANCHOR_NUM) && (ANCHOR_ID + 1 == ANCHOR_NUM) && is_last_anchor)) {
      // The last anchor should write buffer after transmission
      is_last_anchor = 0;


      // Copy the payload to the sending buffer
      for (int i = 10; i < sizeof(sending_msg); i++) {
        // The first 10 bytes in UWB message are MAC
        sending_msg[i] = msg_payload[i - 10];
      }

      memset(msg_payload, 0, (ANCHOR_NUM - 1) * SINGLE_LEN + END_LEN + 4);

      // It's time for hopping
      if (frame_seq_nb % 2 == 1 && anchor_state != ANCHOR_SEND) {
        // DEBUG_LOG_TRACE("hop ", current_freq)

        if (ANCHOR_ID + 1 == ANCHOR_NUM) {
          // If we are the last anchor, we should not start hopping until the message is successfully sent
          while (!(DW1000Ng::isTransmitDone())) {};
        }

        if (frame_seq_nb % 4 == 1) {
          // Switch to channel 3
          current_freq = 3;

          DW1000Ng::forceTRxOff();
          DW1000Ng::applyConfiguration(CONFIG_2);
          set_tx_config(TX_CONFIG::CH_3);
        } else if (frame_seq_nb % 4 == 3) {
          // Switch to channel 1
          current_freq = 1;
          DW1000Ng::forceTRxOff();
          DW1000Ng::applyConfiguration(CONFIG_1);
          set_tx_config(TX_CONFIG::CH_2);
        }

        if (ANCHOR_ID + 1 == ANCHOR_NUM) {
          DW1000Ng::setPreambleDetectionTimeout(0);
          /* Clear reception timeout to start next ranging process. */
          DW1000Ng::setReceiveFrameWaitTimeoutPeriod(RX_TIMEOUT);
          // If we are the last anchor, we should start receive after hopping
          DW1000Ng::startReceive();
          anchor_state = ANCHOR_SEND;
        }

        if (0 == ANCHOR_ID) {
          // If we are anchor 0, we should start transmission after hopping
          // Time delay between the messages from different anchors
          uint32_t delay_time;

          delay_time = DELAY_TIME_TURN + 600;
          frame_seq_nb++;

          // The tx time is set to "delay_time" seconds after the reception of the last message using delayed transmission
          uint64_t tx_time = rx_ts + ((uint64_t)delay_time * UUS_TO_DWT_TIME);
          byte futureTimeBytes[LENGTH_TIMESTAMP]; // LENGTH_TIMESTAMP is 5 bytes
          DW1000NgUtils::writeValueToBytes(futureTimeBytes, tx_time, LENGTH_TIMESTAMP);
          DW1000Ng::setDelayedTRX(futureTimeBytes);


          // Config the frame sequence number
          sending_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

          // Write tx data into tx buffer
          DW1000Ng::setTransmitData(sending_msg, sizeof(sending_msg));

          DW1000Ng::setReceiveFrameWaitTimeoutPeriod(RX_TIMEOUT);

          // Enable rx RX_AFTER_TX_DELAY after the transmission
          DW1000Ng::setWait4Response(RX_AFTER_TX_DELAY);


          DW1000Ng::startTransmit(TransmitMode::DELAYED);
          anchor_state = ANCHOR_SEND;
          // DEBUG_LOG_TRACE("hop tx ", frame_seq_nb);
        }
      }
    }
  } else {
    err_num++;
    DEBUG_LOG_TRACE("err ", err_num);

    // Handling packet loss
    if (0 == ANCHOR_ID) {
      // Hooping Now! If received more than ANCHOR_NUM-2 messages.
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

      // Restart from anchor 0 if any packet is lost
      anchor_state = ANCHOR_SEND;

      sending_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
      DW1000Ng::setTransmitData(sending_msg, sizeof(sending_msg));

      DW1000Ng::setWait4Response(RX_AFTER_TX_DELAY);

      DW1000Ng::setReceiveFrameWaitTimeoutPeriod(RX_TIMEOUT);
      DW1000Ng::setPreambleDetectionTimeout(0);

      // Start transmission immediately
      DW1000Ng::startTransmit();
      // DEBUG_LOG_TRACE("err tx ", frame_seq_nb);
    } else {
      // Hopping Now! If received more than ANCHOR_NUM-2 messages
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

  if (logIndex >= 10) {
    for(int i = 0; i < logIndex; i++) {
        Serial.printf("Tx:%d Seq:%d Ph:%d PC:%d\n", 
                      logBuffer[i].anchor, logBuffer[i].seq, 
                      logBuffer[i].phase, logBuffer[i].rxpc);
    }
    logIndex = 0; // Reset the buffer
  }
}
