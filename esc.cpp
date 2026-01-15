#include <Arduino.h>
#include <SPI.h>
#include "mcp2515.h"

#define ESC_CAN_ID 104
#define NODE_CAN_ID 36 // Your device's CAN ID

// Relevant CAN command IDs
typedef enum {
  CAN_PACKET_PROCESS_SHORT_BUFFER = 8,
  CAN_PACKET_FILL_RX_BUFFER = 5,
  CAN_PACKET_PROCESS_RX_BUFFER = 7,
  CAN_PACKET_STATUS_6 = 58  // ADC values broadcast
} CAN_PACKET_ID;

class ESC {
  private:
    MCP2515 mcp2515;
    struct can_frame rxFrame;
    uint8_t rxData[50];
    uint8_t rxLen = 0;

  public:
    // Realtime vars
    int32_t erpm = 0;
    double voltage = 0.0;
    double dutyCycle = 0.0;

    // ADC vars (normalized 0.0-1.0 from STATUS_6)
    double adc1 = 0.0;
    double adc2 = 0.0;
  
    bool adcDataAvailable = false;

    // Footpad detection
    bool footpadTriggered = false;
    double footpadThreshold = 0.30;  // Adjust based on your sensor (0.0-1.0)

    ESC() : mcp2515(10) {} // CS pin for MCP2515

    void setup() {
      SPI.begin();
      mcp2515.reset();
      mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
      mcp2515.setNormalMode();
    }

    // Called periodically (e.g. every 100ms)
    bool getRealtimeData() {
      sendRealtimeRequest();
      bool newData = readRealtimeResponse();
      return newData;
    }

    // Passive listening for any messages
    void listenForMessages() {

      // Read ALL pending messages to avoid buffer overflow
      for (int i = 0; i < 10; i++) {  // Read up to 10 messages per call
        
        if (mcp2515.readMessage(&rxFrame) != MCP2515::ERROR_OK) { 
          break;  // No more messages
        }
      
        // Non-blocking read — only parses known message types
        uint32_t id = rxFrame.can_id;
        if (id == (0x80000000 + ((uint16_t)CAN_PACKET_FILL_RX_BUFFER << 8) + NODE_CAN_ID)) {
          if (rxFrame.data[0] + rxFrame.can_dlc - 1 < sizeof(rxData)) {
            memcpy(&rxData[rxFrame.data[0]], &rxFrame.data[1], rxFrame.can_dlc - 1);
            rxLen += rxFrame.can_dlc - 1;
          }
        } 
        else if (id == (0x80000000 + ((uint16_t)CAN_PACKET_PROCESS_RX_BUFFER << 8) + NODE_CAN_ID)) {
          // Check if this is a realtime data response
          if (rxLen >= 17 && rxData[0] == 0x32) {
            parseRealtimeData();
          }
          rxLen = 0;    
        }
          // Handle STATUS_6 messages with ADC data
        else if (id == (0x80000000 + ((uint16_t)CAN_PACKET_STATUS_6 << 8) + ESC_CAN_ID)) {
          parseStatus6();
        }
      }
    }

  private:
    void sendRealtimeRequest() {
      struct can_frame msg;
      msg.can_id  = (uint32_t(0x8000) << 16) | (uint16_t(CAN_PACKET_PROCESS_SHORT_BUFFER) << 8) | ESC_CAN_ID;
      msg.can_dlc = 7;
      msg.data[0] = NODE_CAN_ID;
      msg.data[1] = 0x00;
      msg.data[2] = 0x32; // Realtime data command
      msg.data[3] = 0x00;
      msg.data[4] = 0x00;
      msg.data[5] = B10000001;
      msg.data[6] = B11000011;
      mcp2515.sendMessage(&msg);
    }

    bool readRealtimeResponse() {
      bool dataReady = false;

      // Small loop to catch frames for this transaction
      unsigned long startTime = millis();
      while (millis() - startTime < 5) { // very short window
        if (mcp2515.readMessage(&rxFrame) == MCP2515::ERROR_OK) {
          uint32_t id = rxFrame.can_id;

          if (id == (0x80000000 + ((uint16_t)CAN_PACKET_FILL_RX_BUFFER << 8) + NODE_CAN_ID)) {
            if (rxFrame.data[0] + rxFrame.can_dlc - 1 < sizeof(rxData)) {
              memcpy(&rxData[rxFrame.data[0]], &rxFrame.data[1], rxFrame.can_dlc - 1);
              rxLen += rxFrame.can_dlc - 1;
            }
          } else if (id == (0x80000000 + ((uint16_t)CAN_PACKET_PROCESS_RX_BUFFER << 8) + NODE_CAN_ID)) {
            if (rxLen >= 17 && rxData[0] == 0x32) {
              parseRealtimeData();
              dataReady = true;
            }
            rxLen = 0;
          }
        }
      }

      return dataReady;
    }

    void parseRealtimeData() {
      dutyCycle = ((int16_t(rxData[9]) << 8) | int16_t(rxData[10])) / 1000.0;
      erpm      = ((int32_t(rxData[11]) << 24) | (int32_t(rxData[12]) << 16) |
                   (int32_t(rxData[13]) << 8)  | (int32_t(rxData[14])));
      voltage   = ((int16_t(rxData[15]) << 8) | int16_t(rxData[16])) / 10.0;
    }

    // Parse STATUS_6 (periodic ADC broadcast)
    void parseStatus6() {
      if (rxFrame.can_dlc < 8) {
        return;
      }

      // STATUS_6 format: [adc1][adc2][adc3][ppm]
      // Each value is int16 * 1000 (normalized 0.0-1.0)
      int16_t adc1_raw = ((int16_t)rxFrame.data[0] << 8) | rxFrame.data[1];
      int16_t adc2_raw = ((int16_t)rxFrame.data[2] << 8) | rxFrame.data[3];

      adc1 = adc1_raw / 1000.0;  // Normalized 0.0-1.0
      adc2 = adc2_raw / 1000.0;

      // Check current footpad state based on threshold
      bool currentState = (adc1 > footpadThreshold || adc2 > footpadThreshold);
      
      // Update immediately - let the application handle debouncing if needed
      footpadTriggered = currentState;
      
      adcDataAvailable = true;
    }
};

