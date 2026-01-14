#include <Arduino.h>
#include <SPI.h>
#include "mcp2515.h"

#define ESC_CAN_ID 107
#define NODE_CAN_ID 36 // Your device's CAN ID

// Relevant CAN command IDs
typedef enum {
  CAN_PACKET_FILL_RX_BUFFER = 5,
  CAN_PACKET_PROCESS_RX_BUFFER = 7,
  CAN_PACKET_PROCESS_SHORT_BUFFER = 8,
  CAN_PACKET_CONF_CURRENT_LIMITS = 21,
  CAN_PACKET_CONF_CURRENT_LIMITS_IN = 23,
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
    double adc3 = 0.0;
    double ppm = 0.0;
    bool adcDataAvailable = false;

    // Footpad detection
    bool footpadTriggered = false;
    double footpadThreshold = 0.15;  // Adjust based on your sensor (0.0-1.0)

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

    // Set battery current limits temporarily (does not persist after reboot)
    // maxCurrent: maximum battery current in amps (discharge)
    // maxRegenCurrent: maximum regenerative braking current in amps (optional, defaults to maxCurrent)
    // Returns true if command was sent successfully
    bool setBatteryCurrentLimit(float maxCurrent, float maxRegenCurrent = -1.0) {
      // Safety validation
      if (maxCurrent < 0 || maxCurrent > 500) {
        return false;  // Invalid discharge current
      }
      
      // If regen current not specified, use same as max current
      if (maxRegenCurrent < 0) {
        maxRegenCurrent = maxCurrent;
      }
      
      if (maxRegenCurrent < 0 || maxRegenCurrent > 500) {
        return false;  // Invalid regen current
      }

      // Build the payload (8 bytes: two int32 values)
      uint8_t buffer[8];
      int32_t send_index = 0;
      
      // Convert to milliamps and pack as int32 (big endian)
      int32_t maxCurrentMilliamps = (int32_t)(maxCurrent * 1000.0);
      int32_t maxRegenCurrentMilliamps = (int32_t)(maxRegenCurrent * 1000.0);
      
      // Pack max current (big endian)
      buffer[send_index++] = (maxCurrentMilliamps >> 24) & 0xFF;
      buffer[send_index++] = (maxCurrentMilliamps >> 16) & 0xFF;
      buffer[send_index++] = (maxCurrentMilliamps >> 8) & 0xFF;
      buffer[send_index++] = maxCurrentMilliamps & 0xFF;
      
      // Pack max regen current (big endian)
      buffer[send_index++] = (maxRegenCurrentMilliamps >> 24) & 0xFF;
      buffer[send_index++] = (maxRegenCurrentMilliamps >> 16) & 0xFF;
      buffer[send_index++] = (maxRegenCurrentMilliamps >> 8) & 0xFF;
      buffer[send_index++] = maxRegenCurrentMilliamps & 0xFF;

      // Send with Extended ID format
      struct can_frame msg;
      msg.can_id = ESC_CAN_ID | ((uint32_t)CAN_PACKET_CONF_CURRENT_LIMITS_IN << 8);
      msg.can_id |= CAN_EFF_FLAG;  // Mark as extended frame
      msg.can_dlc = 8;
      memcpy(msg.data, buffer, 8);
      
      return mcp2515.sendMessage(&msg) == MCP2515::ERROR_OK;
    }

    // Set motor current limits temporarily (for controlling acceleration)
    // minCurrent: minimum motor current in amps (braking/regen, should be negative or 0)
    // maxCurrent: maximum motor current in amps (acceleration)
    // Returns true if command was sent successfully
    bool setMotorCurrentLimit(float minCurrent, float maxCurrent) {
      // Safety validation
      if (maxCurrent < 0 || maxCurrent > 500) {
        return false;  // Invalid max motor current
      }
      if (minCurrent > 0 || minCurrent < -500) {
        return false;  // Invalid min motor current (should be negative or 0)
      }

      // Build the payload (8 bytes: two int32 values)
      uint8_t buffer[8];
      int32_t send_index = 0;
      
      // Convert to milliamps and pack as int32 (big endian)
      int32_t minCurrentMilliamps = (int32_t)(minCurrent * 1000.0);
      int32_t maxCurrentMilliamps = (int32_t)(maxCurrent * 1000.0);
      
      // Pack min current (big endian)
      buffer[send_index++] = (minCurrentMilliamps >> 24) & 0xFF;
      buffer[send_index++] = (minCurrentMilliamps >> 16) & 0xFF;
      buffer[send_index++] = (minCurrentMilliamps >> 8) & 0xFF;
      buffer[send_index++] = minCurrentMilliamps & 0xFF;
      
      // Pack max current (big endian)
      buffer[send_index++] = (maxCurrentMilliamps >> 24) & 0xFF;
      buffer[send_index++] = (maxCurrentMilliamps >> 16) & 0xFF;
      buffer[send_index++] = (maxCurrentMilliamps >> 8) & 0xFF;
      buffer[send_index++] = maxCurrentMilliamps & 0xFF;

      // Send with Extended ID format
      struct can_frame msg;
      msg.can_id = ESC_CAN_ID | ((uint32_t)CAN_PACKET_CONF_CURRENT_LIMITS << 8);
      msg.can_id |= CAN_EFF_FLAG;  // Mark as extended frame
      msg.can_dlc = 8;
      memcpy(msg.data, buffer, 8);

      return mcp2515.sendMessage(&msg) == MCP2515::ERROR_OK;
    }

    // Optional: passive listening for any messages (not required for realtime)
    void listenForMessages() {

      // Read ALL pending messages to avoid buffer overflow
      for (int i = 0; i < 10; i++) {  // Read up to 10 messages per call
        
        if (mcp2515.readMessage(&rxFrame) == MCP2515::ERROR_OK) {
          break;  // No more messages
        }
      
        // Non-blocking read – only parses known message types
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

    // Send a command using the FILL_RX_BUFFER + PROCESS_RX_BUFFER mechanism
    // This is required for commands longer than 6 bytes
    bool sendBufferedCommand(uint8_t* data, uint8_t dataLen) {
      struct can_frame msg;
      uint8_t sendIndex = 0;
      
      // Send data in chunks using FILL_RX_BUFFER
      while (sendIndex < dataLen) {
        msg.can_id = (uint32_t(0x8000) << 16) | (uint16_t(CAN_PACKET_FILL_RX_BUFFER) << 8) | ESC_CAN_ID;
        
        // First byte is the buffer index
        msg.data[0] = sendIndex;
        
        // Copy up to 7 bytes of data
        uint8_t bytesToSend = min(7, dataLen - sendIndex);
        memcpy(&msg.data[1], &data[sendIndex], bytesToSend);
        msg.can_dlc = bytesToSend + 1;
        
        if (mcp2515.sendMessage(&msg) != MCP2515::ERROR_OK) {
          return false;
        }
        
        sendIndex += bytesToSend;
        delay(1);  // Small delay between frames
      }
      
      // Send PROCESS_RX_BUFFER command to execute
      msg.can_id = (uint32_t(0x8000) << 16) | (uint16_t(CAN_PACKET_PROCESS_RX_BUFFER) << 8) | ESC_CAN_ID;
      msg.can_dlc = 3;
      msg.data[0] = NODE_CAN_ID;
      msg.data[1] = 0x00;
      msg.data[2] = dataLen;
      
      return mcp2515.sendMessage(&msg) == MCP2515::ERROR_OK;
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
      int16_t adc3_raw = ((int16_t)rxFrame.data[4] << 8) | rxFrame.data[5];
      int16_t ppm_raw = ((int16_t)rxFrame.data[6] << 8) | rxFrame.data[7];

      adc1 = adc1_raw / 1000.0;  // Normalized 0.0-1.0
      adc2 = adc2_raw / 1000.0;
      adc3 = adc3_raw / 1000.0;
      ppm = ppm_raw / 1000.0;

      // Check current footpad state based on threshold
      bool currentState = (adc1 > footpadThreshold || adc2 > footpadThreshold);
      
      // Update immediately - let the application handle debouncing if needed
      footpadTriggered = currentState;
      
      adcDataAvailable = true;
    }
};