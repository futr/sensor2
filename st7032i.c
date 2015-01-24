#include "st7032i.h"

void st7032i_init( uint8_t contrast )
{
    /* 液晶を初期化 */
    uint8_t data;

    /* Function set */
    data = 0x38;
    i2c_write_register( ST7032IAddress, 0x00, &data, 1, I2CPolling );

    _delay_ms( 1 );

    /* Function set */
    data = 0x39;
    i2c_write_register( ST7032IAddress, 0x00, &data, 1, I2CPolling );

    _delay_ms( 1 );

    /* Internal OSC Freq */
    /* 表示が安定しない場合フレーム周波数を上げると安定する場合がある */
    // data = 0x14;
    data = 0x16;
    i2c_write_register( ST7032IAddress, 0x00, &data, 1, I2CPolling );

    _delay_ms( 1 );

    /* contrast set */
    data = 0x70 | ( contrast & 0x0F );
    i2c_write_register( ST7032IAddress, 0x00, &data, 1, I2CPolling );

    _delay_ms( 1 );

    /* Power-Icon-Contrast control */
    data = 0x5C | ( ( contrast >> 4 ) & 0x03 );
    i2c_write_register( ST7032IAddress, 0x00, &data, 1, I2CPolling );

    _delay_ms( 1 );

    /* Follower control */
    data = 0x6C;
    i2c_write_register( ST7032IAddress, 0x00, &data, 1, I2CPolling );

    _delay_ms( 300 );

    /* Function set */
    data = 0x38;
    i2c_write_register( ST7032IAddress, 0x00, &data, 1, I2CPolling );

    _delay_ms( 1 );

    /* Display on-off control */
    data = 0x0C;
    i2c_write_register( ST7032IAddress, 0x00, &data, 1, I2CPolling );

    _delay_ms( 1 );

    /* Clear */
    data = 0x01;
    i2c_write_register( ST7032IAddress, 0x00, &data, 1, I2CPolling );

    _delay_ms( 2 );
}

void st7032i_set_char( char line, char pos, char character )
{
    /* 指定位置に文字を表示 */
    uint8_t data;
    uint8_t addr;

    /* アドレス指定 */
    addr = line * 40 + pos;

    data = 0x80 | addr;
    i2c_write_register( ST7032IAddress, 0x00, &data, 1, I2CPolling );

    /* 書き込み */
    data = character;
    i2c_write_register( ST7032IAddress, 0x40, &data, 1, I2CPolling );
}

void st7032i_puts( char line, char pos, char *str )
{
    /* 1行出力 */
    uint8_t data;
    uint8_t addr;

    /* アドレス指定 */
    addr = line * 40 + pos;

    data = 0x80 | addr;
    i2c_write_register( ST7032IAddress, 0x00, &data, 1, I2CPolling );

    /* Wait */
    _delay_us( 50 );

    /* 書き込み */
    i2c_write_register( ST7032IAddress, 0x40, (uint8_t *)str, strlen( str ), I2CPolling );
    /*
    for ( i = 0; str[i] != '\0'; i++ ) {
        data = str[i];

        i2c_write_register( ST7032IAddress, 0x40, &data, 1, I2CPolling );
    }
    */

    /* Wait */
    _delay_us( 50 );
}

void st7032i_clear( void )
{
    /* 画面クリア */
    uint8_t data;

    data = 0x01;
    i2c_write_register( ST7032IAddress, 0x00, &data, 1, I2CPolling );

    _delay_ms( 2 );
}

void st7032i_set_icon( ST7032IIconAddr address, ST7032IIcon on )
{
    /* 指定ICONをOnOff */
    uint8_t data;

    /* Function set */
    data = 0x39;
    i2c_write_register( ST7032IAddress, 0x00, &data, 1, I2CPolling );

    /* Set icon address */
    data = 0x40 | ( ( address >> 4 ) & 0x0F );
    i2c_write_register( ST7032IAddress, 0x00, &data, 1, I2CPolling );

    /* Write Data to RAM */
    data = 0x1F & ( on << 1 );
    i2c_write_register( ST7032IAddress, 0x40, &data, 1, I2CPolling );

    /* Function set */
    data = 0x38;
    i2c_write_register( ST7032IAddress, 0x00, &data, 1, I2CPolling );
}
