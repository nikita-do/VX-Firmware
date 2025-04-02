
/*
 * Header file for ADPD144RI PPG sensor
 */

#ifndef ADPD144_H__
#define ADPD144_H__

#include "driver/i2c_master.h"
#include "esp_err.h"

/* ------------------------- Defines  --------------------------------------- */
#define I2C_SCL_IO_PIN 36
#define I2C_SDA_IO_PIN 35
#define PORT_NUMBER I2C_NUM_0
#define MASTER_FREQUENCY 400000 // Set the master frequency to 400 kHz
#define DEVICE_ADDRESS 0x64 // ADPD144 I2C address

#define REG_VALUE_NUM_BYTES (2)
#define REG_ADDR_NUM_BYTES (1)

// Define register addresses
#define REG_STATUS            0x00  // Status register
#define REG_INT_MASK          0x01  // Interrupt mask register
#define REG_INT_IO_CTL        0x02  // Interrupt control register
#define REG_FIFO_THRESH       0x06  // FIFO threshold register
#define REG_CHIP_ID           0x08  // FIFO threshold register
#define REG_MODE              0x10  // Mode register
#define REG_SLOT_EN           0x11  // 
#define REG_F_SAMPLE          0x12  // 
#define REG_PD_LED_SELECT     0x14  // 
#define REG_NUM_AVG           0x15  // 
#define REG_SLOTA_CH1_OFFSET  0x18  // 
#define REG_SLOTA_CH2_OFFSET  0x19  // 
#define REG_SLOTA_CH3_OFFSET  0x1A  // 
#define REG_SLOTA_CH4_OFFSET  0x1B  // 
#define REG_SLOTB_CH1_OFFSET  0x1E  // 
#define REG_SLOTB_CH2_OFFSET  0x1F  // 
#define REG_SLOTB_CH3_OFFSET  0x20  // 
#define REG_SLOTB_CH4_OFFSET  0x21  // 
#define REG_ILED1_COARSE      0x23  // 
#define REG_ILED2_COARSE      0x24  // 
#define REG_ILED_FINE         0x25  // 
#define REG_SLOTA_LEDMODE     0x30  // 
#define REG_SLOTA_NUMPULSES   0x31  // Slot A number of pulses register
#define REG_SLOTB_LEDMODE     0x35  // 
#define REG_SLOTB_NUMPULSES   0x36  // Slot B number of pulses register
#define REG_SLOTA_AFEMODE     0x39  // Slot A Analog Front-End (AFE) configuration register
#define REG_SLOTB_AFEMODE     0x3B  // Slot B Analog Front-End (AFE) configuration register
#define REG_SLOTA_GAIN        0x42  // 
#define REG_SLOTB_GAIN        0x44  // 
#define REG_SAMPLE_CLK        0x4B  // 
#define REG_ADC_TIMING        0x4E  // 
#define REG_DATA_ACCESS_CTL   0x5F  // 
#define REG_SLOTA_PD1_16_BIT  0x64  // 
#define REG_SLOTA_PD2_16_BIT  0x65  // 
#define REG_SLOTA_PD3_16_BIT  0x66  // 
#define REG_SLOTA_PD4_16_BIT  0x67  // 
#define REG_SLOTB_PD1_16_BIT  0x68  // 
#define REG_SLOTB_PD2_16_BIT  0x69  // 
#define REG_SLOTB_PD3_16_BIT  0x6A  // 
#define REG_SLOTB_PD4_16_BIT  0x6B  // 
#define REG_SLOTA_PD1_LOW     0x70  // 
#define REG_SLOTA_PD2_LOW     0x71  // 
#define REG_SLOTA_PD3_LOW     0x72  // 
#define REG_SLOTA_PD4_LOW     0x73  // 
#define REG_SLOTA_PD1_HIGH    0x74  // 
#define REG_SLOTA_PD2_HIGH    0x75  // 
#define REG_SLOTA_PD3_HIGH    0x76  // 
#define REG_SLOTA_PD4_HIGH    0x77  // 
#define REG_SLOTB_PD1_LOW     0x78  // 
#define REG_SLOTB_PD2_LOW     0x79  // 
#define REG_SLOTB_PD3_LOW     0x7A  // 
#define REG_SLOTB_PD4_LOW     0x7B  // 
#define REG_SLOTB_PD1_HIGH    0x7C  // 
#define REG_SLOTB_PD2_HIGH    0x7D  // 
#define REG_SLOTB_PD3_HIGH    0x7E  // 
#define REG_SLOTB_PD4_HIGH    0x7F  // 

typedef struct
{
    i2c_device_config_t i2c_dev_conf;       /*!< Configuration for adpd device */
    i2c_master_dev_handle_t i2c_dev_handle; /*!< I2C device handle */
    uint8_t buffer[REG_VALUE_NUM_BYTES + REG_ADDR_NUM_BYTES]; /*!< I2C transaction buffer */
} i2c_adpd144_t, *i2c_adpd144_handle_t;

typedef struct {
    uint8_t address;
    uint16_t value;
} adpd144_register_t;

esp_err_t adpd144_init();
void adpd144_start(void);
void adpd144_stop(void);
void adpd144_readReg(uint8_t nAddr, uint16_t *pnData);
void adpd144_writeReg(uint8_t nAddr, uint16_t nRegValue);
esp_err_t adpd144_readRedValue(uint32_t *data, uint8_t len);
esp_err_t adpd144_readIRValue(uint32_t *data, uint8_t len);

#endif /* ADPD144_H__ */