
#ifndef DAC80501_H
#define DAC80501_H

#ifdef __cplusplus
extern "C"{
#endif
    
#include "Arduino.h"

/*!
 * @addtogroup dac DAC  Click Driver
 * @brief API for configuring and manipulating DAC  Click driver.
 * @{
 */

/**
 * @defgroup dac_reg DAC  Registers List
 * @brief List of registers of DAC  Click driver.
 */

/**
 * @addtogroup dac_reg
 * @{
 */

/**
 * @brief DAC  description register.
 * @details Specified register for description of DAC  Click driver.
 */
#define DAC_REG_NOOP                      0x00
#define DAC_REG_DEVID                     0x01
#define DAC_REG_SYNC                      0x02
#define DAC_REG_CONFIG                    0x03
#define DAC_REG_GAIN                      0x04
#define DAC_REG_TRIGGER                   0x05
#define DAC_REG_STATUS                    0x07
#define DAC_REG_DAC                       0x08

/*! @} */ // dac_reg

/**
 * @defgroup dac_set DAC  Registers Settings
 * @brief Settings for registers of DAC  Click driver.
 */

/**
 * @addtogroup dac_set
 * @{
 */

/**
 * @brief DAC  description setting.
 * @details Specified setting for description of DAC  Click driver.
 */
#define DAC_SYNC_DISABLE                0x0000
#define DAC_SYNC_ENABLE                 0x0001

#define DAC_CONFIG_REF_PWDWN_ENABLE     0x0000
#define DAC_CONFIG_REF_PWDWN_DISABLE    0x0100
#define DAC_CONFIG_REF_PWDWN_BIT_MASK   0x0100
#define DAC_CONFIG_DAC_PWDWN_DISABLE    0x0000
#define DAC_CONFIG_DAC_PWDWN_ENABLE     0x0001
#define DAC_CONFIG_DAC_PWDWN_BIT_MASK   0x0001
    
#define DAC_GAIN_REF_DIV_DISABLE        0x0000
#define DAC_GAIN_REF_DIV_2              0x0100
#define DAC_GAIN_REF_DIV_BIT_MASK       0x0100
#define DAC_GAIN_BUFF_GAIN_1            0x0000
#define DAC_GAIN_BUFF_GAIN_2            0x0001
#define DAC_GAIN_BUFF_GAIN_BIT_MASK     0x0001

#define DAC_TRIGGER_LDAC                0x0010
#define DAC_TRIGGER_SOFT_RESET          0x000A

#define DAC_STATUS_REF_ALARM_BIT_MASK   0x0001

/**
 * @brief DAC  device address setting.
 * @details Specified setting for device slave address selection of
 * DAC  Click driver.
 */
#define DAC_I2C_ADR_AGND                0x48
#define DAC_I2C_ADR_VDD                 0x49
#define DAC_I2C_ADR_SDA                 0x4A
#define DAC_I2C_ADR_SCL                 0x4B

/**
 * @brief Data sample selection.
 * @details This macro sets data samples for SPI modules.
 * @note Available only on Microchip PIC family devices.
 * This macro will set data sampling for all SPI modules on MCU. 
 * Can be overwritten with @b dac_init which will set
 * @b SET_SPI_DATA_SAMPLE_MIDDLE by default on the mapped mikrobus.
 */
#define DAC_SET_DATA_SAMPLE_EDGE      SET_SPI_DATA_SAMPLE_EDGE
#define DAC_SET_DATA_SAMPLE_MIDDLE    SET_SPI_DATA_SAMPLE_MIDDLE

/*! @} */ // dac_set


/**
 * @brief DAC  Click driver selector.
 * @details Selects target driver interface of DAC  Click driver.
 */
typedef enum
{
   DAC_DRV_SEL_SPI,      /**< SPI driver descriptor. */
   DAC_DRV_SEL_I2C       /**< I2C driver descriptor. */

} dac_drv_t;

/**
 * @brief DAC  Click driver interface.
 * @details Definition of driver interface of DAC  Click driver.
 */
typedef bool ( *dac_master_io_t )( struct dac_s*, uint8_t, uint8_t*, uint8_t ); /**< Driver serial interface. */

/**
 * @brief DAC  Click context object.
 * @details Context object definition of DAC  Click driver.
 */
typedef struct dac_s
{
    uint16_t  scl;        /**< Clock pin descriptor for I2C driver. */
    uint16_t  sda;        /**< Bidirectional data pin descriptor for I2C driver. */
    uint16_t  miso;       /**< Master input - slave output pin descriptor for SPI driver. */
    uint16_t  mosi;       /**< Master output - slave input pin descriptor for SPI driver. */
    uint16_t  sck;        /**< Clock pin descriptor for SPI driver. */
    uint16_t  cs;         /**< Chip select pin descriptor for SPI driver. */
    
    uint32_t  i2c_speed;       /**< I2C serial speed. */
    uint8_t   i2c_address;     /**< I2C slave address. */

    uint32_t  spi_speed;       /**< SPI serial speed. */

    uint8_t     slave_address;                      /**< Device slave address (used for I2C driver). */
    dac_drv_t  drv_sel;               /**< Master driver interface selector. */

    dac_master_io_t  write_f;         /**< Master write function. */
    dac_master_io_t  read_f;          /**< Master read function. */

} dac_t;

/**
 * @brief DAC  Click return value data.
 * @details Predefined enum values for driver return values.
 */
typedef enum
{
   DAC_OK = 1,
   DAC_ERROR = 0

} dac_return_value_t;

/*!
 * @addtogroup dac DAC  Click Driver
 * @brief API for configuring and manipulating DAC  Click driver.
 * @{
 */


/**
 * @brief DAC  driver interface setup function.
 * @details This function sets a serial driver interface which will be used
 * further in the click driver.
 * @param[out] cfg : Click configuration structure.
 * See #dac_t object definition for detailed explanation.
 * @param[in] drv_sel : Driver interface selection.
 * See #dac_drv_t object definition for detailed explanation.
 * @return Nothing.
 * @note This driver selection should be call before init function to configure
 * the driver to work with the serial interface which is consistent with the
 * real state of the hardware. If this function is not called, the default
 * driver interface will be set.
 */
void dac_drv_interface_selection ( dac_t *cfg, dac_drv_t drv_sel );

/**
 * @brief DAC  initialization function.
 * @details This function initializes all necessary pins and peripherals used
 * for this click board.
 * @param[out] ctx : Click context object.
 * See #dac_t object definition for detailed explanation.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #bool definition for detailed explanation.
 * @note None.
 */
bool dac_init ( dac_t *ctx );

/**
 * @brief DAC  data writing function.
 * @details This function writes a desired number of data bytes starting from
 * the selected register.
 * @param[in] ctx : Click context object.
 * See #dac_t object definition for detailed explanation.
 * @param[in] reg : Start register address.
 * @param[in] data_in : Data to be written.
 * @param[in] len : Number of bytes to be written.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #bool definition for detailed explanation.
 * @note None.
 */
bool dac_generic_write ( dac_t *ctx, uint8_t reg, uint8_t *data_in, uint8_t len );

/**
 * @brief DAC  data reading function.
 * @details This function reads a desired number of data bytes starting from
 * the selected register.
 * @param[in] ctx : Click context object.
 * See #dac_t object definition for detailed explanation.
 * @param[in] reg : Start register address.
 * @param[out] data_out : Output read data.
 * @param[in] len : Number of bytes to be read.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #bool definition for detailed explanation.
 * @note None.
 */
bool dac_generic_read ( dac_t *ctx, uint8_t reg, uint8_t *data_out, uint8_t len );

/**
 * @brief Enable synchronous function.
 * @details The function is used to set synchronous or asynchronous mode.
 * @param[in] ctx : Click context object.
 * See #dac_t object definition for detailed explanation.
 * @param[in] en_sync : DAC output mode
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #bool definition for detailed explanation.
**/
bool dac_enable_sync ( dac_t *ctx, uint8_t en_sync );

/**
 * @brief Set config function.
 * @details The function is used to set the devices configuration.
 * @param[in] ctx : Click context object.
 * See #dac_t object definition for detailed explanation.
 * @param[in] en_ref_pwdwn : Internal reference mode.
 * @param[in] en_dac_pwdwn : DAC in power-up mode.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #bool definition for detailed explanation.
**/
bool dac_set_config ( dac_t *ctx, uint16_t en_ref_pwdwn, uint16_t en_dac_pwdwn );

/**
 * @brief Set gain function.
 * @details The function is used to set gain and internal voltage reference.
 * @param[in] ctx : Click context object.
 * See #dac_t object definition for detailed explanation.
 * @param[in] en_ref_div : Reference voltage mode.
 * @param[in] en_buff_gain : DAC gain mode.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #bool definition for detailed explanation.
**/
bool dac_set_gain ( dac_t *ctx, uint16_t en_ref_div, uint16_t en_buff_gain );

/**
 * @brief Set synchronously load function.
 * @details The function is used to set 'LDAC' bit in order to synchronously
 * load the DAC in synchronous mode, This bit is self resetting.
 * @param[in] ctx : Click context object.
 * See #dac_t object definition for detailed explanation.
 * @return Nothing.
**/
void dac_set_synchronously_load ( dac_t *ctx );

/**
 * @brief Set reset function.
 * @details The function is used to perform software reset.
 * @param[in] ctx : Click context object.
 * See #dac_t object definition for detailed explanation.
 * @return Nothing.
**/
void dac_soft_reset ( dac_t *ctx );

/**
 * @brief Get ref alarm function.
 * @details The function is used to get reference alarm
 * of the device.
 * @param[in] ctx : Click context object.
 * See #dac_t object definition for detailed explanation.
 * @return @li @c  0x00 - Ok.,
 *         @li @c  0x01 - Difference between the reference and supply pins
 *                        is below a minimum analog threshold.
**/
uint8_t dac_get_ref_alarm ( dac_t *ctx );

/**
 * @brief Set Vout function.
 * @details The function is used set Vout ( mV ) by calculating input data 
 * and writing it to the DAC data register.
 * @param[in] ctx : Click context object.
 * See #dac_t object definition for detailed explanation.
 * @param[in] vout_mv : Vout in mV.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #bool definition for detailed explanation.
**/
bool dac_set_vout ( dac_t *ctx, uint16_t vout_mv );


uint8_t dac_spiSendReceiveByte(uint8_t txBuf);


#ifdef __cplusplus
}
#endif
#endif // DAC80501_H

/*! @} */ // dac


// ------------------------------------------------------------------------ END
