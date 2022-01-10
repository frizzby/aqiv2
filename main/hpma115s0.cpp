#include "HPMA115S0.h"
#include "esp_log.h"
#include <string>

extern "C" {
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
}

static char *TAG = "hpma115s0";


char calc_checksum(int head, int len, int cmd, int data) {
  //CS = MOD((65536-(HEAD+LEN+CMD+DATA)), 256)
  return (65536 - (head + len + cmd + data)) % 256;
}


/**
 * @brief Constructor for HPMA115S0 class
 * @param  a Stream ({Software/Hardware}Serial) object.
 * @note The serial stream should be already initialized
 * @return  void
 */
HPMA115S0::HPMA115S0(uart_port_t serial) : _serial(serial) {
//   _serial.setTimeout(100);
}

/**
 * @brief Function that initializes sensor
 * @return  a String containing sensor response
 */
void HPMA115S0::Init() {
  StartParticleMeasurement();
  vTaskDelay(pdMS_TO_TICKS(100));
  DisableAutoSend();
}

/**
 * @brief Function that sends serial command to sensor
 * @param  a unsigned char * containing the command
 * @param size of buffer
 * @return  void
 */
void HPMA115S0::SendCmd(const char * cmdBuf, const int cmdSize) {
  //Clear RX
  uart_flush(_serial);
  auto c = uart_write_bytes(_serial, cmdBuf, cmdSize);
  // ESP_LOGI(TAG, "sent %d bytes.", c);
  
}

/**
 * @brief Function that reads command response from sensor
 * @param Buffer to store data in
 * @param Buffer size
 * @param Expected command type
 * @return  returns number of bytes read from sensor
 */
int HPMA115S0::ReadCmdResp(unsigned char * dataBuf, unsigned int dataBufSize, unsigned int cmdType) {
  static unsigned char respBuf[HPM_MAX_RESP_SIZE];
  static unsigned int respIdx = 0;
  static unsigned int calChecksum = 0;

  //Read response
  respIdx = 0;
  calChecksum = 0;
  memset(respBuf, 0, sizeof(respBuf));
//   _serial.setTimeout(100);
  
  if (skipUntilAfter(HPM_CMD_RESP_HEAD, 10)) {
    vTaskDelay(pdMS_TO_TICKS(1)); //wait for the rest of the bytes to arrive
    respBuf[HPM_HEAD_IDX] = HPM_CMD_RESP_HEAD;
    readBytes(&respBuf[HPM_LEN_IDX], 1, 100, 10); //Read the command length

    //Ensure buffers are big enough
    if (respBuf[HPM_LEN_IDX] && ((respBuf[HPM_LEN_IDX] + 1) <=  sizeof(respBuf) - 2) && (respBuf[HPM_LEN_IDX] - 1) <= dataBufSize ) {
      if (readBytes(&respBuf[HPM_CMD_IDX], respBuf[HPM_LEN_IDX] + 1, 100, 10) == (respBuf[HPM_LEN_IDX] + 1)) { //read respBuf[HPM_LEN_IDX] num of bytes + calChecksum byte
        if (respBuf[HPM_CMD_IDX] == cmdType) { //check if CMD type matches

          //Calculate and validate checksum
          for (respIdx = 0; respIdx < (2 + respBuf[HPM_LEN_IDX]); respIdx++) {
            calChecksum += respBuf[respIdx];
          }
          calChecksum = (65536 - calChecksum) % 256;
          if (calChecksum == respBuf[2 + respBuf[HPM_LEN_IDX]]) {
            memset(dataBuf, 0, dataBufSize);
            memcpy(dataBuf, &respBuf[HPM_DATA_START_IDX], respBuf[HPM_LEN_IDX] - 1);
            return (respBuf[HPM_LEN_IDX] - 1);
          }
        }
      }
    }
  }
  return 0;
}


/**
 * @brief Function that sends a read command to sensor
 * @return  returns true if valid measurements were read from sensor
 */
bool HPMA115S0::ReadParticleMeasurement(int * pm2_5, int * pm10) {
  // ESP_LOGI(TAG, "ReadParticleMeasurement");
  const char cmdBuf[] = {0x68, 0x01, 0x04, 0x93};
  static unsigned char dataBuf[HPM_READ_PARTICLE_MEASURMENT_LEN - 1];

  //Send command
  SendCmd(cmdBuf, 4);

  //Read response
  if (ReadCmdResp(dataBuf, sizeof(dataBuf), READ_PARTICLE_MEASURMENT) == (HPM_READ_PARTICLE_MEASURMENT_LEN - 1)) {
    _pm2_5 = dataBuf[0] * 256 + dataBuf[1];
    _pm10 = dataBuf[2] * 256 + dataBuf[3];
    // ESP_LOGI(TAG, "pm2.5=%d pm10=%d", _pm2_5, _pm10);
    
    *pm2_5 = _pm2_5;
    *pm10 = _pm10;


  // ReadCustomerAdjustmentCoefficient();

    return true;
  }
  return false;
}

/**
 * @brief Function that starts sensor measurement
 * @return  void
 */
void HPMA115S0::StartParticleMeasurement() {
  ESP_LOGI(TAG, "StartParticleMeasurement");
  const char cmd[] = {0x68, 0x01, 0x01, 0x96};
  SendCmd(cmd, 4);

  static unsigned char dataBuf[10];
  ESP_LOGI(TAG, "Waiting for ACK");
  auto rx_bytes = uart_read_bytes(_serial, dataBuf, 2, pdMS_TO_TICKS(10000));
  ESP_LOGI(TAG, "Received %d bytes: %02X:%02X", dataBuf[0], dataBuf[1], rx_bytes);

}


/**
 * @brief Read and log customer adjustment coefficient.
 * @return  void
 */
void HPMA115S0::ReadCustomerAdjustmentCoefficient() {
  ESP_LOGI(TAG, "ReadCustomerAdjustmentCoefficient");
  const char cmd[] = {0x68, 0x01, 0x10, calc_checksum(0x68, 0x01, 0x10, 0)};
  SendCmd(cmd, 4);

  static unsigned char dataBuf[10];
  auto rx_bytes = uart_read_bytes(_serial, dataBuf, 4, pdMS_TO_TICKS(10000));
  // ESP_LOGI(TAG, "Received %d bytes: %02X:%02X:%02X:%02X", dataBuf[0], dataBuf[1], dataBuf[3], dataBuf[4], rx_bytes);
  ESP_LOGI(TAG, "ReadCustomerAdjustmentCoefficient: Received %d bytes:", rx_bytes);
  ESP_LOG_BUFFER_HEX(TAG, dataBuf, rx_bytes);
}

/**
 * @brief Read and log customer adjustment coefficient.
 * @return  void
 */
void HPMA115S0::SetCustomerAdjustmentCoefficient(char coeff) {
  ESP_LOGI(TAG, "SetCustomerAdjustmentCoefficient: %02x", coeff);
  const char cmd[] = {0x68, 0x02, 0x08, coeff, calc_checksum(0x68, 0x02, 0x08, coeff)};
  SendCmd(cmd, 5);

  static unsigned char dataBuf[10];
  auto rx_bytes = uart_read_bytes(_serial, dataBuf, 2, pdMS_TO_TICKS(10000));
  // ESP_LOGI(TAG, "Received %d bytes: %02X:%02X:%02X:%02X", dataBuf[0], dataBuf[1], dataBuf[3], dataBuf[4], rx_bytes);
  ESP_LOGI(TAG, "SetCustomerAdjustmentCoefficient: Received %d bytes:", rx_bytes);
  ESP_LOG_BUFFER_HEX(TAG, dataBuf, rx_bytes);
}

/**
 * @brief Function that stops sensor measurement
 * @return  void
 */
void HPMA115S0::StopParticleMeasurement() {
  ESP_LOGI(TAG, "StopParticleMeasurement");
  const char cmd[] = {0x68, 0x01, 0x02, 0x95};
  SendCmd(cmd, 4);
}

/**
 * @brief Function that enables auto send
 * @return  void
 */
void HPMA115S0::EnableAutoSend() {
  const char cmd[] = {0x68, 0x01, 0x40, 0x57};
  SendCmd(cmd, 4);
}

/**
 * @brief Function that stops auto send
 * @return  void
 */
void HPMA115S0::DisableAutoSend() {
  ESP_LOGI(TAG, "DisableAutoSend");
  const char cmd[] = {0x68, 0x01, 0x20, 0x77};
  SendCmd(cmd, 4);
}

/**
* @brief Function that returns the latest PM 2.5 reading
* @note Sensor reports new reading ~ every 1 sec.
* @return  PM 2.5 reading (unsigned int)
*/
unsigned int HPMA115S0::GetPM2_5() {
  return _pm2_5;
}

/**
* @brief Function that returns the latest PM 10 reading
* @note Sensor reports new reading ~ every 1 sec.
* @return  PM 10 reading (unsigned int)
*/
unsigned int HPMA115S0::GetPM10() {
  return _pm10;
}

size_t HPMA115S0::readBytes(uint8_t* buf, uint32_t length, TickType_t ticks_to_wait, uint8_t retries) {
  // ESP_LOGI(TAG, "readBytes length=%d", length);

  uint8_t try_num=0;
  int rx_bytes, total_rx_bytes = 0;
  
  do {
    rx_bytes = uart_read_bytes(_serial, buf, length, pdMS_TO_TICKS(1500));
    
    // printf("uart_read_bytes: %d\n", rx_bytes);
    // for (int i = 0; i < rx_bytes; i++) {
        // if (i > 0) printf(":");
        // printf("%02X", buf[i]);
    // }
    // printf("\n");
    // const char cmdBuf[] = {0x68, 0x01, 0x04, 0x93};
    // SendCmd(cmdBuf, 4);

    if (rx_bytes == -1) {
        ++try_num;
        ESP_LOGE(TAG, "Error reading from UART after %d/%d tries.", try_num, retries);
        if (try_num == retries) {
            return total_rx_bytes;
        }
    } else {
        total_rx_bytes += rx_bytes;
    }
  } while (total_rx_bytes < length);

  return total_rx_bytes;
}

bool HPMA115S0::skipUntilAfter(char terminator, uint8_t retries) {
//   bool res;
  uint8_t byte;
  do {
    readBytes(&byte, 1, 100, retries);
  } while (byte != terminator);
  return true;
}