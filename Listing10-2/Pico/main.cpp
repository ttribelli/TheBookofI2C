// Listing 10-2 (main.cpp)
// Raspberry Pi Pico
// C/C++ SDK
// https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html
//
// Multi-threaded I2C demonstration #2.
// This program writes to two separate
// MCP4725 devices on the same I2C bus
// using separate threads for each of the
// DACs, with a mutex to protect writes
// to the I2C port.
//
// Tested devices:
//                                          ADDRS   SDA     SCL
//     Raspberry Pi Pico Development Board            4       5
//     SparkFun MCP4725                      0x60
//                                           0x61
//     Adafruit MCP4725                      0x62
//                                           0x63

#include <cstdio>
#include <cstring>
#include <thread>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"

#define i2cInst  i2c0
#define i2cAddr1 0x60
#define i2cAddr2 0x61
#define i2cBaud  (400 * 1000)
#define i2cSCL   5
#define i2cSDA   4


// Mutex protecting console I/O and I2C
mutex_t ioMutex;
mutex_t i2cMutex;


// I2cControllerInit-
//
// Initialize the I2C controller
uint I2cControllerInit( i2c_inst_t *i2c )
{
    // Configure i2c

    auto baudrate = i2c_init( i2c, i2cBaud );
    gpio_set_function( i2cSDA, GPIO_FUNC_I2C );
    gpio_set_function( i2cSCL, GPIO_FUNC_I2C );
    gpio_pull_up( i2cSDA );
    gpio_pull_up( i2cSCL );
    
    return baudrate;
}


// I2cControllerWrite-
//
// Send a string of bytes to an I2C peripheral
int I2cControllerWrite( i2c_inst_t    *i2c,
                        uint8_t        address,
                        const uint8_t *data,
                        size_t         size )
{
    return i2c_write_blocking( i2c, address, data, size, false );
}


// WriteToDAC-
//
// Write a value to an MCP4725
// and use a mutex to protect
// I2C operations
int WriteToDAC( i2c_inst_t *i2c, uint8_t address, uint16_t value )
{
    // Note: MCP4725 requires that we write
    // the HO byte first and the LO byte second!
    uint8_t buffer[] = {
        uint8_t( ( value >> 8 ) & 0xf ),
        uint8_t( value & 0xff )
    };
    
    mutex_enter_blocking( &i2cMutex );
    auto err = I2cControllerWrite( i2c, address, buffer, sizeof(buffer) );
    mutex_exit( &i2cMutex );
    
    return err;
}


// DACout-
//
// Draws one cycle of a triangle waveform on
// the MCP4725 I2C DAC device (e.g., Adafruit
// MCP4725 breakout board).
//
// Arguments: I2C port and address for the DAC.
// For Adafruit MCP4725 breakout boards, the
// address is either 0x62 or 0x63. For Sparkfun
// boards, this is either 0x60 or 0x61.

void DACout( i2c_inst_t *i2c, uint8_t address )
{
    int err;

    // Send the rising edge of the triangle wave:
    for( uint16_t dacOut = 0; dacOut < 0xfff; ++dacOut )
    {
        err = WriteToDAC( i2c, address, dacOut );
        if (err == PICO_ERROR_GENERIC) {
            printf("WriteToDAC() failed 0x%02x\n", address);
        }
    }

    // Send the falling edge of the triangle wave:
    for( uint16_t dacOut = 0xffe; dacOut > 0; --dacOut )
    {
        err = WriteToDAC( i2c, address, dacOut );
        if (err == PICO_ERROR_GENERIC) {
            printf("WriteToDAC() failed 0x%02x\n", address);
        }
    }
}


void DACthread( void )
{
    int cntr = 0;

    while( true )
    {
        // Print a message each time the thread
        // completes one cycle of the triangle
        // wave. Note that printf must be
        // protected by a mutex.
        
        mutex_enter_blocking( &ioMutex );
        printf( "thread loop, cntr=%d\n", cntr++ );
        mutex_exit( &ioMutex );

        // Draw one cycle of the triangle waveform
        // on the second DAC

        DACout( i2cInst, i2cAddr2 );
        
    } // endwhile

}


int main( void )
{
    int cntr = 0;

    stdio_init_all();

    // Give minicom some time to establish a connection
    sleep_ms(2500);

    printf( "Configuring I2C with SDA %d SCL %d Freq %d\n"
            "MCP4725 at addresses 0x%02x 0x%02x\n",
            i2cSDA, i2cSCL,
            i2cBaud,
            i2cAddr1, i2cAddr2 );
    fflush( stdout );

    // Initialize I2C
    
    I2cControllerInit( i2cInst );

    // Initialize the mutex used to protect
    // console I/O and the main task
    
    mutex_init( &ioMutex );
    mutex_init( &i2cMutex );

    // Start the other task
    
    multicore_launch_core1( DACthread );

    while( true )
    {
        // Print a message each time the main thread
        // completes one cycle of the triangle
        // wave. Note that printf must be
        // protected by a mutex.
        
        mutex_enter_blocking( &ioMutex );
        printf( "main loop, cntr=%d\n", cntr++ );
        mutex_exit( &ioMutex );

        // Draw one cycle of the triangle waveform
        // on the first DAC

        DACout( i2cInst, i2cAddr1 );
        
    } // endwhile

} // app_main
