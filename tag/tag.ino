#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>

// CONFIGURATION - CHANGE THESE DEPENDING ON SETUP
#define ANCHOR_NUM 4 // # anchors in the system


#define FRAME_LEN_MAX 127
#define ALL_MSG_SN_IDX 2
#define SINGLE_LEN 13
#define END_LEN 0
/* Length of the common part of the message (up to and including the function code). */
#define ALL_MSG_COMMON_LEN 10
/* Length of CIR read from the accumulator buffer*/
#define CIR_LEN 3

#define RX_ANT_DLY 32880
#define TX_ANT_DLY 32880
#define NET_PANID 0xF0F2

const uint8_t PIN_RST = 27;  // reset pin
const uint8_t PIN_IRQ = 34;  // irq pin
const uint8_t PIN_SS = 4;    // spi select pin


#define LCD_BUFF_LEN (200)
static uint8_t usbVCOMout[LCD_BUFF_LEN];


static uint8_t cir_buffer[(4 * CIR_LEN + 1) * ANCHOR_NUM];
static uint64_t rx_ts[ANCHOR_NUM]; // Timestamp
uint8_t rx_buffer[FRAME_LEN_MAX];

/* Payload format (CIR and receiving timestamps of messages from other anchors)

|---CIR(4 byte)---|--RxTime(5 byte)--|******|---CIR(4 byte)---|--RxTime(5 byte)--|--TxTime(5 byte)--|-IDX(1 byte)-|
|---------------------------13 byte * (Anchor Number-1)--------------------------|--------------6 byte------------| 
*/
static uint8_t msg_buffer[((ANCHOR_NUM - 1) * SINGLE_LEN + END_LEN) * ANCHOR_NUM];
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

static int n = 0;
static uint16_t current_tx;

static uint16_t offset1;
static uint16_t offset2;

static uint16_t current_freq = 1;

static int32_t ci[ANCHOR_NUM];

/* Phase compenstation value measured using SFD */
uint8_t phase_cal[ANCHOR_NUM];

/* RSSI value for timestamp compensation */
uint16_t maxGC[ANCHOR_NUM];
uint8_t rxPC[ANCHOR_NUM];


// GPT set up a CH1 and CH3
device_configuration_t TAG_CH1 = {
  true,   // extendedFrameLength: keep consistent with anchors (safe if you piggyback AO data)
  true,   // receiverAutoReenable: tag should keep RX alive across rapid packets
  true,  // smartPower: irrelevant if tag is passive
  true,   // frameCheck: drop bad CRC frames
  false,  // nlos: start false; only enable after testing
  SFDMode::DECAWAVE_SFD,
  Channel::CHANNEL_1,
  DataRate::RATE_6800KBPS,
  PulseFrequency::FREQ_64MHZ,
  PreambleLength::LEN_64,
  PreambleCode::CODE_9 // must be valid for ch.1/PRF in your DW1000Ng mapping
};

device_configuration_t TAG_CH3 = {
  true,
  true,
  true,
  true,
  false,
  SFDMode::DECAWAVE_SFD,
  Channel::CHANNEL_3,
  DataRate::RATE_6800KBPS,
  PulseFrequency::FREQ_64MHZ,
  PreambleLength::LEN_64,
  PreambleCode::CODE_9 // pick the valid code for ch.3 (or rely on _setValidPreambleCode()) // TODO: anchor uses CODE_9 but I don't know what that means - Maddie
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

interrupt_configuration_t interruptConfig = {
  .interruptOnSent = false,                        // No DWT_INT_TFRS
  .interruptOnReceived = true,                     // DWT_INT_RFCG
  .interruptOnReceiveFailed = true,                // DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE
  .interruptOnReceiveTimeout = true,               // DWT_INT_RFTO
  .interruptOnReceiveTimestampAvailable = false,   // No specific flag present (usually implied by Rx Good)
  .interruptOnAutomaticAcknowledgeTrigger = false  // No DWT_INT_AAT
};

// LIBRARY ADDED FUNCTIONS NOTES
  // uint16_t MULoc_dwRead16BitOffsetReg(byte regFileID, uint16_t offset) // TODO: Do we need to reorder and sum bytes like the MULoc function does?
  
void dw_init() {
  // DEBUG monitoring
  Serial.begin(115200);
  Serial.println(F("### DW1000Ng-arduino-sender-test ###"));
  // initialize the driver
  DW1000Ng::initializeNoInterrupt(PIN_SS);
  Serial.println(F("DW1000Ng initialized ..."));

  DW1000Ng::applyConfiguration(TAG_CH1);
  // Did not set_tx_config(TX_CONFIG::CH_2); as it didn't seem required. Double check?

  DW1000Ng::enableLedBlinking();

  DW1000Ng::setDeviceAddress(1);
  DW1000Ng::setNetworkId(NET_PANID);

  DW1000Ng::setRxAntennaDelay(RX_ANT_DLY); // Double check both delay values?
  DW1000Ng::setTxAntennaDelay(TX_ANT_DLY); 

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

uint8_t ledPin = 2;
void setup() {
  pinMode(0, ledPin);
  dw_init();

}

void loop() {
    // put your main code here, to run repeatedly:

    // Check on: Set preamble, set Rx stuff
    DW1000Ng::setPreambleDetectionTimeout(0);

	/* Clear reception timeout to start next ranging process. */
	DW1000Ng::setReceiveFrameWaitTimeoutPeriod(0);

	/* Activate reception immediately. */
	DW1000Ng::startReceive();

    while (!(DW1000Ng::isReceiveDone() || DW1000Ng::isReceiveFailed())) {}
 
    if (!DW1000Ng::isReceiveFailed()) { // TODO: These don't check the exact same bits as MULoc code

        int frame_len = DW1000Ng::getReceivedDataLength();  // Assuming bytes?

        DW1000Ng::clearTransmitStatus();
        DW1000Ng::clearReceiveStatus(); // Registers checked are slightly different. MULoc also does this in the if statement

        if (frame_len <= FRAME_LEN_MAX) {
            DW1000Ng::getReceivedData(rx_buffer, frame_len); // In bytes
            frame_seq_nb = rx_buffer[ALL_MSG_SN_IDX];
        }

        current_tx = rx_buffer[8];
        frame_seq_nb = rx_buffer[ALL_MSG_SN_IDX];

        memcpy(msg_buffer + ((ANCHOR_NUM - 1) * SINGLE_LEN + END_LEN) * current_tx, rx_buffer + ALL_MSG_COMMON_LEN, (ANCHOR_NUM - 1) * SINGLE_LEN + END_LEN);

        rx_ts[current_tx] = DW1000Ng::getReceiveTimestamp(); // 64 bits

        uint16_t fp_index = DW1000Ng::getFPPathIdx();
        DW1000Ng::getAccData(cir_buffer + current_tx * (CIR_LEN * 4 + 1), CIR_LEN * 4 + 1, (fp_index) * 4); // Just copied this over

        // Read Carrier integer for clock drift estimation
        ci[current_tx] = DW1000Ng::getCarrierIntegrator();

        // Read RCPHASE for phase compensation
        DW1000Ng::readRCPhase(phase_cal + current_tx);

        // Read max growth cir and rxPC for RSSI estimation
        maxGC[current_tx] = DW1000Ng::getCirPwrBytes();
        rxPC[current_tx] = DW1000Ng::getPreambleAccumulationCount();

        // Ranging finished, send the ranging data to SoC/PC over USB
        if (ANCHOR_NUM == current_tx + 1)
        {
            int xx = 0;

            // AO estimates acquired from anchor i
            for (uint8_t i = 0; i < ANCHOR_NUM; i++)
            {

                // Offset1 is the byte offset
                offset1 = ((ANCHOR_NUM - 1) * SINGLE_LEN + END_LEN) * i;

                // AO estimates of anchor j acquired from anchor i
                for (uint8_t j = 0; j < ANCHOR_NUM; j++)
                {
                    uint64_t rx_time;

                    if (j < i)
                    {
                        offset2 = j * SINGLE_LEN;
                    }
                    else if (j > i)
                    {
                        offset2 = (j - 1) * SINGLE_LEN;
                    }
                    else if (j == i)
                    {
                        continue;
                    }

                    /* 4 byte CIR + 1 byte RPHASE + 1 byte rxPC + 2 byte maxGC + 5 byte rx_time */
                    memcpy(usbVCOMout + n, msg_buffer + offset1 + offset2, SINGLE_LEN);
                    n += SINGLE_LEN;
                }

                usbVCOMout[n] = frame_seq_nb;
                n += 1;
            }

            // Then TO estimates
            for (uint8_t i = 0; i < ANCHOR_NUM; i++)
            {

                // CIR
                memcpy(usbVCOMout + n, cir_buffer + (4 * CIR_LEN + 1) * i + 1 + 4, 4);
                n += 4;

                // Phase correction value
                usbVCOMout[n] = phase_cal[i];
                n += 1;

                usbVCOMout[n] = rxPC[i];
                n += 1;

                usbVCOMout[n] = (uint8_t)(maxGC[i] >> 8);
                usbVCOMout[n + 1] = (uint8_t)(maxGC[i]);
                n += 2;

                // Receiving timestamps
                for (int k = 0; k < 5; k++)
                {
                    usbVCOMout[n] = (uint8_t)(rx_ts[i] >> ((4 - k) * 8));
                    n += 1;
                }

                // Carrier integer number
                for (int k = 0; k < 4; k++)
                {
                    usbVCOMout[n] = (uint8_t)(ci[i] >> ((3 - k) * 8));
                    n += 1;
                }
            }

            // Frame sequence number
            usbVCOMout[n] = frame_seq_nb;
            n += 1;

            // Used for data segmentation: 7,6,5,4,3
            for (int i = 0; i < 5; i++)
            {
                usbVCOMout[n] = 7 - i;
                n += 1;
            }

            // Print to serial- later send via bluetooth or something
            for (int i = 0; i < 200; i++) {
                Serial.print(usbVCOMout[i]);
            }
            Serial.println();
            

            n = 0;

            // Perform frequence hooping every two round of localization
            if (frame_seq_nb % 2 == 1)
            {
                if (frame_seq_nb % 4 == 1)
                {
                    
                    // Switch to channel 3
                    current_freq = 3;

                    DW1000Ng::forceTRxOff();
                    DW1000Ng::applyConfiguration(TAG_CH3);
                    set_tx_config(TX_CONFIG::CH_3);
                }
                else if (frame_seq_nb % 4 == 3)
                {

                    // Switch to channel 1
                    current_freq = 1;

                    DW1000Ng::forceTRxOff();
                    DW1000Ng::applyConfiguration(TAG_CH1);
                    set_tx_config(TX_CONFIG::CH_2);
                }
            }
        }
	} else {
        DW1000Ng::clearReceiveFailedStatus();
        // TODO: Check if order and bits will be an issue (clear, check if failed, transmit)

    }
}
				
        


    





