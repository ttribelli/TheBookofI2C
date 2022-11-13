// Listing 13-2 (main.cpp)
// Espressif ESP32
// FreeRTOS
// Espressif IoT Development Framework (ESP-IDF)
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/index.html
//
// Demonstrate reading a rotary encoder
// using an MCP23008 with interrupts.
//
// Assumptions:
//
//  *   MCP23008 INT line connected to
//      digital I/O line 2.
//
//  *   Sparkfun illuminated R/G rotary
//      encoder output A connected to
//      GPA1 on MCP23008.
//
//  *   Sparkfun illuminated R/G rotary
//      encoder output B connected to
//      GPA0 on MCP23008.
//
//  *   MCP23008 wired to use address
//      0x20 (A0, A1, A2 = 0, 0 0).
//
// Tested devices:
//                                      ADDRS   SDA     SCL     INT
//     Espressif ESP32 DevKitC V4                21      22      23
//     Espressif ESP32-C3 DevKitC 02             18      19       3
//     Sparkfun ESP32 Thing Plus                 23      22      14
//     Adafruit Huzzah32 ESP32 Feather           23      22      14
//     MCP23008                          0x20

#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

extern "C"
{
    void app_main( void );
}

// MCP23008 registers

#define IODIR       (0)
#define IOPOL       (1)
#define GPINTEN     (2)
#define DEFVAL      (3)
#define INTCON      (4)
#define IOCON       (5)
#define GPPU        (6)
#define INTF        (7)
#define INTCAP      (8)
#define GPIO        (9)
#define OLAT        (10)

// Mutex protecting console I/O

SemaphoreHandle_t ioMutex = NULL;

// Queue used by ISR and main loop

QueueHandle_t gpioQueue = NULL;

// Interrupt service routine that gets
// called whenever the INT pin on the
// MCP23008 goes from high to low.
// This function needs to be *fast*.
// That means just signal blocked
// code that will do the real work.
//
// FreeRTOS also limits what system
// calls may be used in an ISR. Look
// for functions that end with "FromISR".

static void IRAM_ATTR gpioIsrHandler( void * )
{
    // Signal that a change occurred

    xQueueSendFromISR( gpioQueue, NULL, NULL );
}

// Initialize GPIO interrupt.
// Various parameters are defined using the
// Espressif IDF menuconfig utility.

esp_err_t gpioInterruptInit( void )
{
    // Configure a gpio pin

    gpio_num_t    gpioPin = gpio_num_t( CONFIG_GPIO_MCP23008_INTERRUPT );
    gpio_config_t config;
    memset( &config, 0, sizeof( config ) );

    config.pin_bit_mask = (1ULL << gpioPin);
    config.mode         = GPIO_MODE_INPUT;
    config.intr_type    = GPIO_INTR_NEGEDGE;
    config.pull_up_en   = GPIO_PULLUP_ENABLE;
    
    auto err = gpio_config( &config );
    
    // Configure an ISR

    if ( err == ESP_OK )
    {
        err = gpio_install_isr_service( ESP_INTR_FLAG_LEVEL1 );
    }
    if ( err == ESP_OK )
    {
        err = gpio_isr_handler_add( gpioPin, gpioIsrHandler, NULL );
    }
    
    return err;
}

// Initialize the I2C controller.
// Various parameters are defined using the
// Espressif IDF menuconfig utility.

esp_err_t i2cControllerInit( bool pullup = true )
{
    // Configure an i2c driver
    
    i2c_config_t config;
    memset( &config, 0, sizeof( config ) );
    
    config.mode             = I2C_MODE_MASTER;
    config.sda_io_num       = CONFIG_I2C_CONTROLLER_SDA;
    config.scl_io_num       = CONFIG_I2C_CONTROLLER_SCL;
    config.master.clk_speed = CONFIG_I2C_CONTROLLER_FREQUENCY;
    
    if ( pullup )
    {
        config.sda_pullup_en = GPIO_PULLUP_ENABLE;
        config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    }
    
    auto err = i2c_param_config( CONFIG_I2C_CONTROLLER_PORT_NUM,
                                 &config );

    // Install an i2c driver
    
    if ( err == ESP_OK )
    {
        err = i2c_driver_install( CONFIG_I2C_CONTROLLER_PORT_NUM,
                                  config.mode,
                                  0,    // Disable RX buffer
                                  0,    // Disable TX buffer
                                  0 );  // Interrupt flags
    }
    
    return err;
}

// Send a string of bytes to an I2C peripheral

esp_err_t i2cControllerWrite( i2c_port_t     port,
                              uint8_t        address,
                              const uint8_t *data,
                              size_t         size,
                              bool           ackEnabled )
{
    // Create a command link
    auto command = i2c_cmd_link_create();
    auto err     = ESP_OK;
    if ( command == NULL )
    {
        err = ESP_FAIL;
    }

    // Populate it
    if ( err == ESP_OK )
    {
        err = i2c_master_start( command );          // Start bit
    }
    
    if ( err == ESP_OK )
    {
        err = i2c_master_write_byte( command,       // Peripheral address
                                     (address << 1) | I2C_MASTER_WRITE,
                                     ackEnabled );
    }

    if ( err == ESP_OK )
    {
        err = i2c_master_write( command,            // Data bytes
                                data,
                                size,
                                ackEnabled );
    }

    if ( err == ESP_OK )
    {
        err = i2c_master_stop( command );           // Stop bit
    }
    
    if ( err == ESP_OK )
    {
        // Send it
        err = i2c_master_cmd_begin( port,
                                    command,
                                    1000 / portTICK_PERIOD_MS );
    }

    // Release the resources
    if ( command != NULL )
    {
        i2c_cmd_link_delete( command );
    }

    return err;
}

// Read a string of bytes from an I2C peripheral

esp_err_t i2cControllerReadReg( i2c_port_t  port,
                                uint8_t     address,
                                uint8_t     reg,
                                uint8_t    *data,
                                size_t      size,
                                bool        ackEnabled )
{
    // Create a command link
    auto command = i2c_cmd_link_create();
    auto err     = ESP_OK;
    if ( command == NULL )
    {
        err = ESP_FAIL;
    }

    // Populate it
    if ( err == ESP_OK )
    {
        err = i2c_master_start( command );          // Start bit
    }

    if ( err == ESP_OK )
    {
        err = i2c_master_write_byte( command,       // Peripheral address
                                     (address << 1) | I2C_MASTER_WRITE,
                                     ackEnabled );
    }

    if ( err == ESP_OK )
    {
        err = i2c_master_write_byte( command,       // Register
                                     reg,
                                     ackEnabled );
    }

    if ( err == ESP_OK )
    {
        err = i2c_master_start( command );          // Restart bit
    }

    if ( err == ESP_OK )
    {
        err = i2c_master_write_byte( command,       // Peripheral address
                                     (address << 1) | I2C_MASTER_READ,
                                     ackEnabled );
    }

    if ( err == ESP_OK )
    {
        err = i2c_master_read( command,             // Data bytes
                               data,
                               size,
                               I2C_MASTER_NACK );
    }

    if ( err == ESP_OK )
    {
        err = i2c_master_stop( command );           // Stop bit
    }

    if ( err == ESP_OK )
    {
        // Send it
        err = i2c_master_cmd_begin( port,
                                    command,
                                    1000 / portTICK_PERIOD_MS );
    }

    // Release the resources
    if ( command != NULL )
    {
        i2c_cmd_link_delete( command );
    }

    return err;
}

// Write a value to an MCP23008 register

void writeReg( uint8_t reg, uint8_t val )
{
    uint8_t buffer[] = { reg, val };

    auto err = i2cControllerWrite( CONFIG_I2C_CONTROLLER_PORT_NUM,
                                   CONFIG_I2C_MCP23008_ADDRESS,
                                   buffer,
                                   sizeof( buffer ),
                                   true );     // ACK check enabled
    ESP_ERROR_CHECK_WITHOUT_ABORT( err );
}

// Read a value from an MCP23008 register

int readReg( uint8_t reg )
{
    uint8_t buffer[] = { 0 };

    auto err = i2cControllerReadReg( CONFIG_I2C_CONTROLLER_PORT_NUM,
                                     CONFIG_I2C_MCP23008_ADDRESS,
                                     reg,
                                     buffer,
                                     sizeof( buffer ),
                                     true );     // ACK check enabled
    ESP_ERROR_CHECK_WITHOUT_ABORT( err );

    return int( buffer[0] );
}

// Reset the MCP23008 to a known state

void mcpReset( void )
{
    // I2C General Call is not mentioned in
    // the manual so do this the hard way
    
    // INTF   is read only
    // INTCAP is read only

    // Disable interrupts
    // and clear any pendng
    writeReg( GPINTEN, 0 );
    readReg( INTCAP );

    // Set remaining registers
    // to POR/RST values
    writeReg( IODIR,   0xFF );
    writeReg( IOPOL,   0 );
    writeReg( GPINTEN, 0 );
    writeReg( DEFVAL,  0 );
    writeReg( INTCON,  0 );
    writeReg( IOCON,   0 );
    writeReg( GPPU,    0 );
    writeReg( GPIO,    0 );
    writeReg( OLAT,    0 );
}

void app_main( void )
{
    // Output the current configuration
    
    printf( "Rotary Encoder Test\n"
            "Configuring Interrupt for pin %d\n"
            "Configuring I2C with SDA %d SCL %d Freq %'d\n"
            "MCP23008 at address 0x%02x\n",
            CONFIG_GPIO_MCP23008_INTERRUPT,
            CONFIG_I2C_CONTROLLER_SDA, CONFIG_I2C_CONTROLLER_SCL,
            CONFIG_I2C_CONTROLLER_FREQUENCY,
            CONFIG_I2C_MCP23008_ADDRESS );
    fflush( stdout );

    // Initialize the mutex used to protect
    // console I/O
    
    ioMutex = xSemaphoreCreateMutex();
    configASSERT( ioMutex );
    
    // Initialize the queue that enables
    // reading the MCP23008
    
    gpioQueue = xQueueCreate( 10, 0 );
    configASSERT( gpioQueue );

    // Initialize interrupt

    auto err = gpioInterruptInit();
    ESP_ERROR_CHECK_WITHOUT_ABORT( err );

    // Initialize I2C
    
    err = i2cControllerInit();
    ESP_ERROR_CHECK_WITHOUT_ABORT( err );

    // Reset the MCP23008 to a known state

    mcpReset();

    // Initialize the MCP23008 (this is the
    // default state, so just defensive coding).
    // Note that SEQOP=0 is autoincrementing registers
    // and INTPOL=0 yields active low interrupts.

    writeReg( IOCON, 0 );

    // Set data direction to input,
    // Turn pullups on for GPA0/GPA1.
    // Set polarity to inverting for GPA0/GPA1.

    writeReg( IODIR, 0xff );
    writeReg( IOPOL, 3 );
    writeReg( GPPU, 3 );

    // Initialize MCP23008 interrupts

    writeReg( INTCON, 0 );      // GPA1 int on change
    writeReg( GPINTEN, 0x2 );   // Enable GPA1 interrupt

    while( true )
    {
        
        // The following variable tracks
        // rotations on the rotary encoder.
        // This variable is negative if there
        // have been more counterclockwise
        // rotations than clockwise. Likewise,
        // it's positive if there have been
        // more clockwise rotations.

        static int rotaryPosn = 0;
        static int lastRP     = 0;

        // Block until the ISR signals a change
        
        xQueueReceive( gpioQueue, NULL, portMAX_DELAY );

        // Read the INTCAP register.
        // This reads the GPIO pins
        // at the time of the interrupt
        // and also clears the interrupt
        // flags.
        //
        // Note: A rotary encoder input in GPA1,
        // B rotary encoder input in GPA0

        int cur = readReg( INTCAP ) & 0x3;

        // We have CW rotation if:
        //
        //     A:0->1 && B==1  (cur=3)
        // or  A:1->0 && B==0  (cur=0)

        if( cur == 0 || cur == 3 )
        {
            ++rotaryPosn;
        }

        // We have CCW rotation if:
        //
        //     A:0->1 && B==0 (cur=2)
        // or  A:1->0 && B==1 (cur=1)

        else if( cur == 1 || cur == 2 )
        {
            --rotaryPosn;
        }
        // else illegal reading...

        // Report any change in position

        if( rotaryPosn != lastRP )
        {
            xSemaphoreTake( ioMutex, portMAX_DELAY );
            printf( "Posn = %d\n", rotaryPosn );
            xSemaphoreGive( ioMutex );
            lastRP = rotaryPosn;
        }
        
    } // endwhile

}
