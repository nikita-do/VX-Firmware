// Written for only one device on the i2c bus
/* ------------------------- Includes -------------------------------------- */
#include <stdint.h>
#include <stdlib.h>
#include "adpd144.h"
#include "esp_check.h"

/* ------------------------- Defines  --------------------------------------- */
#define REG_VALUE_NUM_BYTES (2)
#define REG_ADDR_NUM_BYTES (1)

i2c_adpd144_handle_t dev_handle = NULL;

static const char LOG_TAG[] = "i2c-adpd";

// Define all registers in an array
const adpd144_register_t register_config[] = {
    {REG_MODE, 0x0001},
    {REG_SAMPLE_CLK, 0x0080},
    {REG_INT_IO_CTL, 0x0005},
    {REG_SLOT_EN, 0x30A9},
    {REG_F_SAMPLE, 0x000A},
    {REG_PD_LED_SELECT, 0x0116},
    {REG_NUM_AVG, 0x0330},
    {REG_SLOTA_CH1_OFFSET, 0x3FFF},
    {REG_SLOTA_CH2_OFFSET, 0x3FFF},
    {REG_SLOTA_CH3_OFFSET, 0x1FF0},
    {REG_SLOTA_CH4_OFFSET, 0x1FF0},
    {REG_SLOTB_CH1_OFFSET, 0x3FFF},
    {REG_SLOTB_CH2_OFFSET, 0x3FFF},
    {REG_SLOTB_CH3_OFFSET, 0x1FF0},
    {REG_SLOTB_CH4_OFFSET, 0x1FF0},
    {REG_ILED1_COARSE, 0x3005},
    {REG_ILED2_COARSE, 0x3007},
    {REG_ILED_FINE, 0x0207},
    {REG_SLOTA_LEDMODE, 0x0319},
    {REG_SLOTA_NUMPULSES, 0x0813},
    {REG_SLOTB_LEDMODE, 0x0319},
    {REG_SLOTB_NUMPULSES, 0x0813},
    {REG_SLOTA_AFEMODE, 0x21F3},
    {REG_SLOTB_AFEMODE, 0x21F3},
    {REG_SLOTA_GAIN, 0x1C36},
    {REG_SLOTB_GAIN, 0x1C36},
    {REG_ADC_TIMING, 0x0040},
    {REG_MODE, 0x0002},
    {0xFF, 0xFFFF} // signal the end of list
};

/**
 *  @brief    Load ADPD default configuration
 *  @param    adpd144_register_t *cfg
 *  @retval   None
 */
void adpd144_loadConfig(const adpd144_register_t *cfg)
{
    uint8_t regAddr, i;
    uint16_t regData;
    if (cfg == 0)
        return;
    /* Clear the FIFO */
    adpd144_writeReg(REG_MODE, 0);      // Program mode
    adpd144_writeReg(REG_DATA_ACCESS_CTL, 1);      // Set the FIFO_ACCESS_ENA to 1
    adpd144_writeReg(REG_STATUS, 0x80FF); // Set the FIFO_FLUSH to 0x80FF
    adpd144_writeReg(REG_DATA_ACCESS_CTL, 0);      // Set the FIFO_ACCESS_ENA to 0
    i = 0;
    while (1)
    {
        /* Read the address and data from the config */
        regAddr = (uint8_t)(cfg[i].address);
        regData = (uint16_t)(cfg[i].value);
        i++;
        if (regAddr == 0xFF)
            break;
        /* Load the data into the ADPD registers */
        adpd144_writeReg(regAddr, regData);
    }
}


esp_err_t adpd144_readIRValue(uint32_t *data, uint8_t len)
{
    uint16_t anRxData[2] = {0};
    uint8_t regAddr[2];

    /* Time slot requiring access */
    adpd144_writeReg(REG_DATA_ACCESS_CTL, 1); // Prevent slot A data from updating

    if (len == 1)
    {
        regAddr[0] = REG_SLOTA_PD3_16_BIT; // Low data-word for channel 3 slot A
    }
    else if (len == 2)
    {
        regAddr[0] = REG_SLOTA_PD3_LOW; // Low data-word for channel 3 slot A
        regAddr[1] = REG_SLOTA_PD3_HIGH; // High data-word for channel 3 slot A
    }
    else
    {
        ESP_LOGE(LOG_TAG, "Invalid length for adpd144_ReadReg_SlotA");
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < len; i++)
    {
        adpd144_readReg(regAddr[i], &anRxData[i]);
    }

    if (len == 1)
    {
        *data = anRxData[0];
    }
    else if (len == 2)
    {
        *data = (anRxData[1] << 16) + anRxData[0];
    }

    adpd144_writeReg(REG_DATA_ACCESS_CTL, 0); // Allow slot A data to update

    return ESP_OK;
}


esp_err_t adpd144_readRedValue(uint32_t *data, uint8_t len)
{
    uint16_t anRxData[2] = {0};
    uint8_t regAddr[2];

    /* Time slot requiring access */
    adpd144_writeReg(REG_DATA_ACCESS_CTL, 3); // Prevent slot B data from updating

    if (len == 1)
    {
        regAddr[0] = REG_SLOTB_PD3_16_BIT; // Low data-word for channel 3 slot B
    }
    else if (len == 2)
    {
        regAddr[0] = REG_SLOTB_PD3_LOW; // Low data-word for channel 3 slot B
        regAddr[1] = REG_SLOTB_PD3_HIGH; // High data-word for channel 3 slot B
    }
    else
    {
        ESP_LOGE(LOG_TAG, "Invalid length for adpd144_ReadReg_SlotB");
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < len; i++)
    {
        adpd144_readReg(regAddr[i], &anRxData[i]);
    }

    if (len == 1)
    {
        *data = anRxData[0];
    }
    else if (len == 2)
    {
        *data = (anRxData[1] << 16) + anRxData[0];
    }

    adpd144_writeReg(REG_DATA_ACCESS_CTL, 0); // Allow slot B data to update

    return ESP_OK;
}

esp_err_t adpd144_init(void)
{
    esp_err_t ret = ESP_OK;

    ESP_LOGI(LOG_TAG, "Configuring i2c bus");
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = PORT_NUMBER,
        .scl_io_num = I2C_SCL_IO_PIN,
        .sda_io_num = I2C_SDA_IO_PIN,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    /* Init i2c adpd device */
    if (dev_handle)
    {
        ESP_LOGE(LOG_TAG, "i2c adpd device already initialized");
        return ESP_FAIL;
    }

    dev_handle = (i2c_adpd144_handle_t)calloc(1, sizeof(*dev_handle));
    ESP_GOTO_ON_FALSE(dev_handle, ESP_ERR_NO_MEM, cleanup, LOG_TAG, "Failed to allocate memory for i2c adpd handle");

    dev_handle->buffer = (uint8_t *)calloc(1, REG_ADDR_NUM_BYTES + REG_VALUE_NUM_BYTES);
    ESP_GOTO_ON_FALSE(dev_handle->buffer, ESP_ERR_NO_MEM, cleanup, LOG_TAG, "Failed to allocate memory for i2c adpd device buffer");

    dev_handle->i2c_dev_conf.scl_speed_hz = MASTER_FREQUENCY;
    dev_handle->i2c_dev_conf.device_address = DEVICE_ADDRESS;

    ret = i2c_master_bus_add_device(bus_handle, &dev_handle->i2c_dev_conf, &dev_handle->i2c_dev_handle);
    ESP_GOTO_ON_ERROR(ret, cleanup, LOG_TAG, "i2c new bus failed");

    /* Start reading the IC */
    uint16_t chip_id; // Buffer to store the received data

    adpd144_readReg(REG_CHIP_ID, &chip_id);
    ESP_LOGI(LOG_TAG, "ADPD144 REG_CHIP_ID: %x", chip_id);

    adpd144_loadConfig(register_config);

    return ESP_OK;

cleanup:
    if (dev_handle && dev_handle->i2c_dev_handle)
    {
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle->i2c_dev_handle));
    }
    free(dev_handle);
    dev_handle = NULL;
    return ret;
}

/** @brief  Synchronous register read from the ADPD
 *
 * @param  nAddr 8-bit register address
 * @param  *pnData Pointer to 16-bit register data value
 */
void adpd144_readReg(uint8_t nAddr, uint16_t *pnData)
{
    uint8_t anRxData[2];
    // ESP_RETURN_ON_FALSE(dev_handle, ESP_ERR_INVALID_STATE, LOG_TAG, "i2c device not initialized");

    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle->i2c_dev_handle, &nAddr, REG_ADDR_NUM_BYTES, anRxData, REG_VALUE_NUM_BYTES, -1));

    *pnData = ((uint16_t)anRxData[0] << 8) + anRxData[1];
}

/** @brief  Synchronous register write to the ADPD
 *
 * @param  nAddr 8-bit register address
 * @param  nRegValue 16-bit register data value
 */
void adpd144_writeReg(uint8_t nAddr, uint16_t nRegValue)
{
    // ESP_RETURN_ON_FALSE(dev_handle, ESP_ERR_INVALID_STATE, LOG_TAG, "i2c device not initialized");

    dev_handle->buffer[0] = nAddr;
    dev_handle->buffer[1] = (uint8_t)(nRegValue >> 8);
    dev_handle->buffer[2] = (uint8_t)(nRegValue);

    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle->i2c_dev_handle, dev_handle->buffer,REG_ADDR_NUM_BYTES + sizeof(uint16_t), -1));
}

/* ------------------------- End of file ----------------------------------- */