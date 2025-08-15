
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "esp_system.h"
#include "esp_log.h"

// direct ROM delay (microseconds)
#include "rom/ets_sys.h"


#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"   
#include "soc/sens_reg.h"
#include "soc/io_mux_reg.h"

static const char *TAG = "baremetal";

#define I2C_SDA_GPIO    21
#define I2C_SCL_GPIO    22
#define TAS2563_SDZ_GPIO 27  


#define SPI_SCLK_GPIO   18
#define SPI_MOSI_GPIO   23
#define SPI_MISO_GPIO   19
#define SPI_CS_GPIO     5
#define TRF_EN_GPIO     17  
#define TRF_IRQ_GPIO    16    

#define TAS2563_I2C_ADDR  (0x4C)  

#define TAS_REG_BOOK      0x7F
#define TAS_REG_PAGE      0x00
#define TAS_REG_REV       0x7D
#define TAS_REG_PWRCTL    0x02
#define TAS_REG_PB_CFG1   0x03

#define TRF_REG_IRQ_STATUS    0x0C
#define TRF_REG_FIFO_STATUS   0x1C
#define TRF_REG_TX_LEN1       0x1D
#define TRF_REG_TX_LEN2       0x1E
#define TRF_REG_FIFO          0x1F
// direct commands (datasheet)
#define TRF_CMD_RESET_FIFO        0x0F
#define TRF_CMD_TX_WITH_CRC       0x11
#define TRF_CMD_ENABLE_RX         0x17
#define TRF_CMD_RUN_DECODERS      0x14
#define TRF_CMD_SOFT_INIT         0x03
#define TRF_CMD_IDLE              0x00

#define PIN_FUNC_SELECT_REG(pin_reg, func)   (REG_WRITE(pin_reg, (func)))

static inline void gpio_out_set(int gpio_num)
{
    REG_WRITE(GPIO_OUT_W1TS_REG, (1u << gpio_num));
}
static inline void gpio_out_clr(int gpio_num)
{
    REG_WRITE(GPIO_OUT_W1TC_REG, (1u << gpio_num));
}
static inline uint32_t gpio_in_level(int gpio_num)
{
    return (REG_READ(GPIO_IN_REG) >> gpio_num) & 0x1;
}


static void pin_output(int gpio_num)
{
    gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio_num], PIN_FUNC_GPIO);
}

static void pin_input(int gpio_num)
{
    gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio_num], PIN_FUNC_GPIO);
}

static void enable_pullup(int gpio_num)
{
    gpio_set_pull_mode(gpio_num, GPIO_PULLUP_ONLY);
}

static inline void udelay(uint32_t us)
{
    ets_delay_us(us);
}

static void i2c_init_pins(void)
{
    pin_output(I2C_SCL_GPIO);
    pin_output(I2C_SDA_GPIO);
    // set lines high by releasing them (set as input with pull-up)
    pin_input(I2C_SCL_GPIO);
    pin_input(I2C_SDA_GPIO);
    enable_pullup(I2C_SCL_GPIO);
    enable_pullup(I2C_SDA_GPIO);
    udelay(10);
}

static void i2c_scl_drive_low(void) { pin_output(I2C_SCL_GPIO); gpio_out_clr(I2C_SCL_GPIO); }
static void i2c_scl_release(void)   { pin_input(I2C_SCL_GPIO); }
static void i2c_sda_drive_low(void) { pin_output(I2C_SDA_GPIO); gpio_out_clr(I2C_SDA_GPIO); }
static void i2c_sda_release(void)   { pin_input(I2C_SDA_GPIO); }

static int i2c_read_sda(void) { return gpio_in_level(I2C_SDA_GPIO); }
static int i2c_read_scl(void) { return gpio_in_level(I2C_SCL_GPIO); }

#define I2C_DELAY_US 5   // ~5us low/high -> ~100 kHz;

static void i2c_start(void)
{
    // SDA high->low while SCL high
    i2c_sda_release();
    i2c_scl_release();
    udelay(I2C_DELAY_US);
    i2c_sda_drive_low();
    udelay(I2C_DELAY_US);
    i2c_scl_drive_low();
    udelay(I2C_DELAY_US);
}

static void i2c_stop(void)
{
    // SDA low->high while SCL high
    i2c_sda_drive_low();
    udelay(I2C_DELAY_US);
    i2c_scl_release();
    udelay(I2C_DELAY_US);
    i2c_sda_release();
    udelay(I2C_DELAY_US);
}

static int i2c_write_bit(int bit)
{
    if (bit) i2c_sda_release(); else i2c_sda_drive_low();
    udelay(I2C_DELAY_US);
    i2c_scl_release();
    udelay(I2C_DELAY_US);
    // optional clock stretching check could go here
    i2c_scl_drive_low();
    udelay(I2C_DELAY_US);
    return 0;
}

static int i2c_read_bit(void)
{
    i2c_sda_release();
    udelay(I2C_DELAY_US);
    i2c_scl_release();
    udelay(I2C_DELAY_US);
    int b = i2c_read_sda();
    i2c_scl_drive_low();
    udelay(I2C_DELAY_US);
    return b;
}

static int i2c_write_byte(uint8_t b)
{
    for (int i = 7; i >= 0; --i) {
        i2c_write_bit((b >> i) & 1);
    }
    // read ACK bit
    int ack = i2c_read_bit();
    return ack;
}

static uint8_t i2c_read_byte(int ack)
{
    uint8_t b = 0;
    for (int i = 7; i >= 0; --i) {
        int bit = i2c_read_bit();
        b |= (bit << i);
    }
    i2c_write_bit(ack ? 0 : 1); 
    return b;
}

static int i2c_write_reg(uint8_t dev7, uint8_t reg, uint8_t val)
{
    i2c_start();
    int nak = i2c_write_byte((dev7 << 1) | 0);
    if (nak) { i2c_stop(); return -1; }
    if (i2c_write_byte(reg)) { i2c_stop(); return -2; }
    if (i2c_write_byte(val)) { i2c_stop(); return -3; }
    i2c_stop();
    return 0;
}

static int i2c_read_reg(uint8_t dev7, uint8_t reg, uint8_t *out)
{
    i2c_start();
    if (i2c_write_byte((dev7 << 1) | 0)) { i2c_stop(); return -1; }
    if (i2c_write_byte(reg)) { i2c_stop(); return -2; }
    i2c_start();
    if (i2c_write_byte((dev7 << 1) | 1)) { i2c_stop(); return -3; }
    *out = i2c_read_byte(0); // NACK after read
    i2c_stop();
    return 0;
}

// Mode 0: CPOL=0 CPHA=0
static void spi_init_pins(void)
{
    pin_output(SPI_SCLK_GPIO);
    pin_output(SPI_MOSI_GPIO);
    pin_input(SPI_MISO_GPIO);
    pin_output(SPI_CS_GPIO);
    // idle states
    gpio_out_clr(SPI_SCLK_GPIO);
    gpio_out_set(SPI_CS_GPIO);   
}

static void spi_cs_low(void)  { gpio_out_clr(SPI_CS_GPIO); udelay(1); }
static void spi_cs_high(void) { gpio_out_set(SPI_CS_GPIO); udelay(1); }

static void spi_sclk_pulse(void)
{
    gpio_out_set(SPI_SCLK_GPIO);
    udelay(2);
    gpio_out_clr(SPI_SCLK_GPIO);
    udelay(2);
}

static void spi_write_byte(uint8_t b)
{
    for (int i = 7; i >= 0; --i) {
        if ((b >> i) & 1) gpio_out_set(SPI_MOSI_GPIO);
        else gpio_out_clr(SPI_MOSI_GPIO);
        udelay(1);
        spi_sclk_pulse();
    }
}

static uint8_t spi_read_byte(void)
{
    uint8_t b = 0;
    for (int i = 7; i >= 0; --i) {
        spi_sclk_pulse();
        int bit = gpio_in_level(SPI_MISO_GPIO);
        b |= (bit << i);
    }
    return b;
}

// SPI register header format: MSB=1 (register access), bit6=R/W (1=read), bit5=continuous, bits4..0=reg addr
static uint8_t trf_reg_hdr_rw(uint8_t reg, int read, int cont)
{
    uint8_t hdr = 0x80; // B7=1 to indicate register access
    if (read) hdr |= 0x40;
    if (cont) hdr |= 0x20;
    hdr |= (reg & 0x1F);
    return hdr;
}

static int trf_write_reg(uint8_t reg, uint8_t val)
{
    spi_cs_low();
    spi_write_byte(trf_reg_hdr_rw(reg, 0, 0));
    spi_write_byte(val);
    spi_cs_high();
    return 0;
}

static int trf_read_reg(uint8_t reg, uint8_t *val)
{
    spi_cs_low();
    spi_write_byte(trf_reg_hdr_rw(reg, 1, 0));
    *val = spi_read_byte();
    spi_cs_high();
    return 0;
}

static int trf_write_reg_cont(uint8_t start_reg, const uint8_t *buf, int n)
{
    spi_cs_low();
    spi_write_byte(trf_reg_hdr_rw(start_reg, 0, 1)); 
    for (int i = 0; i < n; ++i) spi_write_byte(buf[i]);
    spi_cs_high();
    return 0;
}

static int trf_read_reg_cont(uint8_t start_reg, uint8_t *buf, int n)
{
    spi_cs_low();
    spi_write_byte(trf_reg_hdr_rw(start_reg, 1, 1));
    for (int i = 0; i < n; ++i) buf[i] = spi_read_byte();
    spi_cs_high();
    return 0;
}

static int trf_direct_cmd(uint8_t cmd)
{
    spi_cs_low();
    spi_write_byte(cmd);
    spi_cs_high();
    return 0;
}

static int tas_select_book_page(uint8_t book, uint8_t page)
{
    // write Book then Page
    if (i2c_write_reg(TAS2563_I2C_ADDR, TAS_REG_BOOK, book) < 0) return -1;
    if (i2c_write_reg(TAS2563_I2C_ADDR, TAS_REG_PAGE, page) < 0) return -2;
    return 0;
}

static int tas_read_rev(uint8_t *rev)
{
    // ensure book/page 0
    tas_select_book_page(0x00, 0x00);
    if (i2c_read_reg(TAS2563_I2C_ADDR, TAS_REG_REV, rev) < 0) return -1;
    return 0;
}

static int tas_set_mode_active(void)
{
    // set PWRCTL MODE bits to 00 (Active)
    uint8_t v;
    if (i2c_read_reg(TAS2563_I2C_ADDR, TAS_REG_PWRCTL, &v) < 0) return -1;
    v &= ~0x03;
    v |= 0x00;
    if (i2c_write_reg(TAS2563_I2C_ADDR, TAS_REG_PWRCTL, v) < 0) return -2;
    return 0;
}

static int tas_get_amp_level(uint8_t *lvl)
{
    uint8_t v;
    if (i2c_read_reg(TAS2563_I2C_ADDR, TAS_REG_PB_CFG1, &v) < 0) return -1;
    *lvl = (v >> 1) & 0x1F;
    return 0;
}

static int tas_set_amp_level(uint8_t lvl)
{
    if (lvl > 0x1C) lvl = 0x1C;
    uint8_t v;
    if (i2c_read_reg(TAS2563_I2C_ADDR, TAS_REG_PB_CFG1, &v) < 0) return -1;
    v &= ~(0x1F << 1);
    v |= ((lvl & 0x1F) << 1);
    if (i2c_write_reg(TAS2563_I2C_ADDR, TAS_REG_PB_CFG1, v) < 0) return -2;
    return 0;
}

static void tas_volume_step(int delta)
{
    uint8_t cur=0;
    if (tas_get_amp_level(&cur) == 0) {
        int n = (int)cur + delta;
        if (n < 0) n = 0;
        if (n > 0x1C) n = 0x1C;
        tas_set_amp_level((uint8_t)n);
        ESP_LOGI(TAG, "TAS amp level -> 0x%02X", n);
    } else {
        ESP_LOGE(TAG, "Failed to get current amp level");
    }
}

static void trf_wait_irq_ms(int ms)
{
    int waited = 0;
    while (gpio_in_level(TRF_IRQ_GPIO) == 1 && waited < ms) {
        udelay(1000);
        waited += 1;
    }
}

static int rfid_inventory_uid(uint8_t uid[8])
{
    trf_direct_cmd(TRF_CMD_RESET_FIFO);
    udelay(100);

    
    trf_direct_cmd(TRF_CMD_RUN_DECODERS);
    trf_direct_cmd(TRF_CMD_ENABLE_RX);
    udelay(100);

    uint8_t frame[] = { 0x26, 0x01, 0x00 };
    uint8_t n = sizeof(frame);
    uint8_t txlen1 = (n >> 4) & 0xFF;
    uint8_t txlen2 = (uint8_t)((n & 0x0F) << 4);
    trf_write_reg(TRF_REG_TX_LEN1, txlen1);
    trf_write_reg(TRF_REG_TX_LEN2, txlen2);
    trf_write_reg_cont(TRF_REG_FIFO, frame, n);

   
    trf_direct_cmd(TRF_CMD_TX_WITH_CRC);

   
    trf_wait_irq_ms(100);
   
    uint8_t irq = 0;
    trf_read_reg(TRF_REG_IRQ_STATUS, &irq);
    ESP_LOGI(TAG, "TRF IRQ=0x%02X", irq);

    uint8_t resp[32]; memset(resp, 0, sizeof(resp));
    trf_read_reg_cont(TRF_REG_FIFO, resp, sizeof(resp));
    int last = 31;
    while (last >= 0 && resp[last] == 0) --last;
    if (last < 9) return -1;
    int crc_idx = last;
    int uid_end = crc_idx - 2;
    int uid_start = uid_end - 7;
    if (uid_start < 0) return -2;
    for (int i = 0; i < 8; ++i) uid[i] = resp[uid_start + i];
    ESP_LOGI(TAG, "UID LSB->MSB: %02X %02X %02X %02X %02X %02X %02X %02X",
             uid[0],uid[1],uid[2],uid[3],uid[4],uid[5],uid[6],uid[7]);
    return 0;
}

static void gpio_lowlevel_init(void)
{
    
    gpio_reset_pin((gpio_num_t)I2C_SDA_GPIO);
    gpio_reset_pin((gpio_num_t)I2C_SCL_GPIO);
    gpio_reset_pin((gpio_num_t)TAS2563_SDZ_GPIO);
    gpio_reset_pin((gpio_num_t)SPI_SCLK_GPIO);
    gpio_reset_pin((gpio_num_t)SPI_MOSI_GPIO);
    gpio_reset_pin((gpio_num_t)SPI_MISO_GPIO);
    gpio_reset_pin((gpio_num_t)SPI_CS_GPIO);
    gpio_reset_pin((gpio_num_t)TRF_EN_GPIO);
    gpio_reset_pin((gpio_num_t)TRF_IRQ_GPIO);
    gpio_set_pull_mode((gpio_num_t)I2C_SDA_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)I2C_SCL_GPIO, GPIO_PULLUP_ONLY);

    gpio_set_direction((gpio_num_t)TRF_EN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)TRF_EN_GPIO, 1);
    gpio_set_direction((gpio_num_t)TAS2563_SDZ_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)TAS2563_SDZ_GPIO, 1);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Bare-metal bit-bang demo start");
    gpio_lowlevel_init();
    i2c_init_pins();
    spi_init_pins();
    udelay(5000);

    uint8_t rev = 0;
    if (tas_select_book_page(0x00, 0x00) == 0 && tas_read_rev(&rev) == 0) {
        ESP_LOGI(TAG, "TAS2563 REV=0x%02X", rev);
    } else {
        ESP_LOGW(TAG, "TAS2563 not responding at I2C 0x%02X", TAS2563_I2C_ADDR);
    }

    if (tas_set_mode_active() == 0) {
        ESP_LOGI(TAG, "TAS set active");
    }

    uint8_t cur;
    if (tas_get_amp_level(&cur) == 0) {
        ESP_LOGI(TAG, "Current AMP_LEVEL = 0x%02X", cur);
    }

    tas_volume_step(+2);
    udelay(300000);
    tas_volume_step(-1);
    udelay(200000);
    trf_direct_cmd(TRF_CMD_SOFT_INIT);
    udelay(5000);
    trf_direct_cmd(TRF_CMD_IDLE);
    udelay(1000);
    trf_write_reg(TRF_REG_IRQ_STATUS, 0xFF); 
    uint8_t uid[8];
    int r = rfid_inventory_uid(uid);
    if (r == 0) {
        ESP_LOGI(TAG, "Tag found (UID printed)");
    } else {
        ESP_LOGW(TAG, "No tag or parse failed (err=%d)", r);
    }

    while (1) {
        udelay(1000000);
    }
}
