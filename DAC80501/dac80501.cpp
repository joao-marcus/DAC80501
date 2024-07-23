#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>
#include "dac80501.h"

/**
 * @brief Dummy data.
 * @details Definition of dummy data.
 */
#define DUMMY             0x00
#define SPI_READ_MASK     0x80

static uint8_t gain_chk;
static uint8_t ref_div_chk;

// -------------------------------------------- PRIVATE FUNCTION DECLARATIONS 

/**
 * @brief DAC  dev writing function.
 * @details This function writes a desired number of data bytes starting from
 * the selected register.
 * @param[in] ctx : Click context object.
 * See #dac_t object definition for detailed explanation.
 * @param[in] reg : Start register address.
 * @param[in] data_in : Data to be written.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #bool definition for detailed explanation.
 */
bool dev_write ( dac_t *ctx, uint8_t reg, uint16_t data_in );

// --------------------------------------------------------- PUBLIC FUNCTIONS 

/**
 * @brief DAC  I2C writing function.
 * @details This function writes a desired number of data bytes starting from
 * the selected register by using I2C serial interface.
 * @param[in] ctx : Click context object.
 * See #dac_t object definition for detailed explanation.
 * @param[in] reg : Start register address.
 * @param[in] data_in : Data to be written.
 * @param[in] len : Number of bytes to be written.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #bool definition for detailed explanation.
 */
static bool dac_i2c_write ( dac_t *ctx, uint8_t reg, uint8_t *data_in, uint8_t len );

/**
 * @brief DAC  I2C reading function.
 * @details This function reads a desired number of data bytes starting from
 * the selected register by using I2C serial interface.
 * @param[in] ctx : Click context object.
 * See #dac_t object definition for detailed explanation.
 * @param[in] reg : Start register address.
 * @param[out] data_out : Output read data.
 * @param[in] len : Number of bytes to be read.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #bool definition for detailed explanation.
 */
static bool dac_i2c_read ( dac_t *ctx, uint8_t reg, uint8_t *data_out, uint8_t len );

/**
 * @brief DAC  SPI writing function.
 * @details This function writes a desired number of data bytes starting from
 * the selected register by using SPI serial interface.
 * @param[in] ctx : Click context object.
 * See #dac_t object definition for detailed explanation.
 * @param[in] reg : Start register address.
 * @param[in] data_in : Data to be written.
 * @param[in] len : Number of bytes to be written.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #bool definition for detailed explanation.
 */
static bool dac_spi_write ( dac_t *ctx, uint8_t reg, uint8_t *data_in, uint8_t len );

/**
 * @brief DAC  SPI reading function.
 * @details This function reads a desired number of data bytes starting from
 * the selected register by using SPI serial interface.
 * @param[in] ctx : Click context object.
 * See #dac_t object definition for detailed explanation.
 * @param[in] reg : Start register address.
 * @param[out] data_out : Output read data.
 * @param[in] len : Number of bytes to be read.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #bool definition for detailed explanation.
 */
static bool dac_spi_read ( dac_t *ctx, uint8_t reg, uint8_t *data_out, uint8_t len );

void dac_drv_interface_selection ( dac_t *cfg, dac_drv_t drv_sel ) {
    cfg->drv_sel = drv_sel;
}

bool dac_init ( dac_t *ctx  ) {
  
      pinMode(ctx->cs, OUTPUT);     // Set SPI slave select pin as output
  
    if ( DAC_DRV_SEL_I2C == ctx->drv_sel ) {
         
      Wire.begin();
      
      Wire.beginTransmission(ctx->slave_address);

      Wire.setClock(ctx->i2c_speed);
      
      digitalWrite(ctx->cs, LOW);  // Make sure ctx->cs is held high
      
      ctx->read_f  = dac_i2c_read;
      ctx->write_f = dac_i2c_write;
    } else {
      
      SPI.begin();                 // begin SPI
//      SPI.beginTransaction(SPISettings(ctx->spi_speed , MSBFIRST, SPI_MODE1));
      SPI.setBitOrder(MSBFIRST); // LSBFIRST or MSBFIRST
      SPI.setDataMode(SPI_MODE1);    //SPI_MODE0    SPI_MODE1   SPI_MODE2   SPI_MODE3
      
      digitalWrite(ctx->cs, HIGH);  // Make sure ctx->cs is held high
      
      ctx->read_f  = dac_spi_read;
      ctx->write_f = dac_spi_write;
    }

    return 1;
}

bool dac_generic_write ( dac_t *ctx, uint8_t reg, uint8_t *data_in, uint8_t len ) {
    return ctx->write_f( ctx, reg, data_in, len );
}

bool dac_generic_read ( dac_t *ctx, uint8_t reg, uint8_t *data_out, uint8_t len ) {
    return ctx->read_f( ctx, reg, data_out, len );
}

static bool dac_i2c_write ( dac_t *ctx, uint8_t reg, uint8_t *data_in, uint8_t len ) {
    uint8_t tx_buf[ 257 ] = { 0 };

    tx_buf[ 0 ] = reg;

    for ( uint8_t cnt = 1; cnt <= len; cnt++ ) {
        tx_buf[ cnt ] = data_in[ cnt - 1 ];
    }

    for (int x = 0; x < len+1; x++) {
      Wire.write(tx_buf[x]);
    }
}

static bool dac_i2c_read ( dac_t *ctx, uint8_t reg, uint8_t *data_out, uint8_t len ) {    
    uint8_t tx_buf[ 257 ] = { 0 };
      
    tx_buf[ 0 ] = reg;
    for ( uint8_t cnt = 1; cnt <= len; cnt++ ) {
        tx_buf[ cnt ] = data_out[ cnt - 1 ];
    }
    
    for (int x = 0; x < len+1; x++) {
      data_out[x] = Wire.write(tx_buf[x]);
    }
}

static bool dac_spi_write ( dac_t *ctx, uint8_t reg, uint8_t *data_in, uint8_t len ) {
    uint8_t tx_buf[ 257 ] = { 0 };

    tx_buf[ 0 ] = reg;
    for ( uint8_t cnt = 1; cnt <= len; cnt++ ) {
        tx_buf[ cnt ] = data_in[ cnt - 1 ];
    }

    digitalWrite(ctx->cs, LOW);
    for (int x = 0; x < len+1; x++) {
      dac_spiSendReceiveByte(tx_buf[x]);
    }
    digitalWrite(ctx->cs, HIGH);

    return 1;
}

static bool dac_spi_read ( dac_t *ctx, uint8_t reg, uint8_t *data_out, uint8_t len ) {
    uint8_t tx_buf[ 257 ] = { 0 };
    uint8_t reg_adr = reg | SPI_READ_MASK;    
    tx_buf[ 0 ] = reg_adr;
    for ( uint8_t cnt = 1; cnt <= len; cnt++ ) {
        tx_buf[ cnt ] = data_out[ cnt - 1 ];
    }
    digitalWrite(ctx->cs, LOW);
    for (int x = 0; x < len+1; x++) {
      data_out[x] = dac_spiSendReceiveByte(tx_buf[x]);
    }
    digitalWrite(ctx->cs, HIGH);

    return 1;
}

bool dac_enable_sync ( dac_t *ctx, uint8_t en_sync ) {
    bool error_flag;

    if ( en_sync == DAC_SYNC_ENABLE ) {
        error_flag = dev_write( ctx, DAC_REG_SYNC, DAC_SYNC_ENABLE );
    } else if ( en_sync == DAC_SYNC_DISABLE ) {
        error_flag =  dev_write( ctx, DAC_REG_SYNC, DAC_SYNC_DISABLE );
    }

    return error_flag;
}

bool dac_set_config ( dac_t *ctx, uint16_t en_ref_pwdwn, uint16_t en_dac_pwdwn ) {
    bool error_flag;
    uint8_t rx_buf[ 2 ];
    uint16_t tmp;

    en_ref_pwdwn &= DAC_CONFIG_REF_PWDWN_BIT_MASK;
    en_dac_pwdwn &= DAC_CONFIG_DAC_PWDWN_BIT_MASK;

    if ( ctx->drv_sel == DAC_DRV_SEL_I2C) { 
        dac_i2c_read( ctx, DAC_REG_CONFIG, rx_buf, 2 );
        tmp = ( uint16_t ) ( rx_buf[ 0 ] << 8 );
        tmp |= ( uint16_t ) rx_buf[ 1 ];
    }

    if ( ( en_ref_pwdwn & en_dac_pwdwn ) >
         ( DAC_CONFIG_REF_PWDWN_BIT_MASK | DAC_CONFIG_DAC_PWDWN_BIT_MASK )
       ) {
        error_flag = DAC_ERROR;
    } else {
        tmp |= ( uint16_t ) en_ref_pwdwn;
        tmp |= ( uint16_t ) en_dac_pwdwn;
        error_flag = dev_write( ctx, DAC_REG_CONFIG, tmp );
    }

    return error_flag;
}

bool dac_set_gain ( dac_t *ctx, uint16_t en_ref_div, uint16_t en_buf_gain ) {
    bool error_flag;
    uint8_t rx_buf[ 2 ];
    uint16_t tmp;

    gain_chk = en_buf_gain;
	ref_div_chk = en_ref_div;

    if ( ctx->drv_sel == DAC_DRV_SEL_I2C) { 
        dac_i2c_read( ctx, DAC_REG_GAIN, rx_buf, 2 );
        tmp = ( uint16_t ) ( rx_buf[ 0 ] << 8 );
        tmp |= ( uint16_t ) rx_buf[ 1 ];
    }

    if ( ( en_ref_div & en_buf_gain ) >
         ( DAC_GAIN_REF_DIV_BIT_MASK | DAC_GAIN_BUFF_GAIN_BIT_MASK )
       ) {
        error_flag = DAC_ERROR;
    } else {
        tmp |= ( uint16_t ) en_ref_div;
        tmp |= ( uint16_t ) en_buf_gain;
        error_flag = dev_write( ctx, DAC_REG_GAIN, tmp );
    }

    return error_flag;
}

void dac_set_synchronously_load ( dac_t *ctx ) {
    dev_write( ctx, DAC_REG_TRIGGER, DAC_TRIGGER_LDAC );
}

void dac_soft_reset ( dac_t *ctx ) {
    dev_write( ctx, DAC_REG_TRIGGER, DAC_TRIGGER_SOFT_RESET );
}

uint8_t dac_get_ref_alarm ( dac_t *ctx ) {
    uint8_t ref_alarm;
    uint8_t rx_buf[ 2 ];
    uint16_t status_val;

    if ( ctx->drv_sel == DAC_DRV_SEL_I2C) { 
        dac_i2c_read( ctx, DAC_REG_STATUS, rx_buf, 2 );
        status_val = ( uint16_t ) ( rx_buf[ 0 ] << 8 );
        status_val |= ( uint16_t ) rx_buf[ 1 ];
     }
    status_val &= DAC_STATUS_REF_ALARM_BIT_MASK;

    ref_alarm = ( uint8_t ) status_val;

    return ref_alarm;
}

bool dac_set_vout ( dac_t *ctx, uint16_t vout_mv ) {
    bool error_flag;
    float v_data;
    uint16_t vout_data;

 //   if ( ( gain_chk == DAC_GAIN_BUFF_GAIN_2 ) && ( vout_mv > 2500 ) 
 //      || ( gain_chk == DAC_GAIN_BUFF_GAIN_1 ) && ( vout_mv > 1250 ) ) {
 //       error_flag = DAC_ERROR;
 //   } else {
        v_data = ( float ) vout_mv;
        
        if ( gain_chk == DAC_GAIN_BUFF_GAIN_2 ) {
//            v_data *= 26.2144;
            v_data *= 13.1072;
        } else {
//            v_data *= 52.4288;
            v_data *= 26.2144;
        }
        vout_data = ( uint16_t ) v_data;

        error_flag = dev_write( ctx, DAC_REG_DAC, vout_data );
  //  }

    return error_flag;
}

// --------------------------------------------- PRIVATE FUNCTION DEFINITIONS 

bool dev_write ( dac_t *ctx, uint8_t reg, uint16_t data_in ) {
    uint8_t tx_buf[ 2 ];
    tx_buf[ 0 ] = ( uint8_t ) ( data_in >> 8 );
    tx_buf[ 1 ] = ( uint8_t ) data_in;
    bool error_flag = dac_generic_write( ctx, reg, tx_buf, 2 );
    return error_flag;
}

// ------------------------------------------------------------------------ END

uint8_t dac_spiSendReceiveByte(uint8_t txBuf) {

  uint8_t dataRx = 0;

  dataRx = SPI.transfer(txBuf); 
 
  return dataRx;
}
