// Listing 13-2 (main.cpp)
// Raspberry Pi Pico
// C/C++ SDK
// https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html
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
//                                          ADDRS   SDA     SCL     INT
//     Raspberry Pi Pico Development Board            4       5      21
//     MCP23008                              0x20

#include <cstdio>
#include <cstring>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"

#define i2cInst     i2c0
#define i2cAddr     0x20
#define i2cBaud     (400 * 1000)
#define i2cSCL      5
#define i2cSDA      4
#define gpioInt     21

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

mutex_t ioMutex;

// Queue used by ISR and main loop

queue_t gpioQueue;

// Interrupt service routine that gets
// called whenever the INT pin on the
// MCP23008 goes from high to low.
// This function needs to be *fast*.
// That means just signal blocked
// code that will do the real work.

void gpioIrqCallback( uint gpio, uint32_t events )
{
    // Only consider our pin
    
    if ( gpio == gpioInt )
    {
        // Signal that a change occurred.
        // Make sure queue is not full
        // so that we do not block.

        if ( ! queue_is_full( &gpioQueue ) )
        {
            queue_add_blocking( &gpioQueue, NULL );
        }
    }
}

// Initialize GPIO interrupt

void gpioInterruptInit( void )
{
    gpio_set_irq_enabled_with_callback( gpioInt,
                                        GPIO_IRQ_EDGE_FALL,
                                        true,
                                        &gpioIrqCallback );
}

// Initialize the I2C controller

uint i2cControllerInit( i2c_inst_t *i2c )
{
    // Configure i2c

    auto baudrate = i2c_init( i2c, i2cBaud );
    gpio_set_function( i2cSDA, GPIO_FUNC_I2C );
    gpio_set_function( i2cSCL, GPIO_FUNC_I2C );
    gpio_pull_up( i2cSDA );
    gpio_pull_up( i2cSCL );
    
    return baudrate;
}

// Send a string of bytes to an I2C peripheral

int i2cControllerWrite( i2c_inst_t    *i2c,
                        uint8_t        address,
                        const uint8_t *data,
                        size_t         size )
{
    return i2c_write_blocking( i2c, address, data, size, false );
}

// Read a string of bytes from an I2C peripheral

int i2cControllerReadReg( i2c_inst_t *i2c,
                          uint8_t     address,
                          uint8_t     reg,
                          uint8_t    *data,
                          size_t      size )
{
    auto num = i2c_write_blocking( i2c, address, &reg, sizeof( reg ), true );
    if ( num != PICO_ERROR_GENERIC )
    {
        num = i2c_read_blocking( i2c, address, data, size, false);
    }

    return num;
}

// Write a value to an MCP23008 register

void writeReg( uint8_t reg, uint8_t val )
{
    uint8_t buffer[] = { reg, val };

    i2cControllerWrite( i2cInst,
                        i2cAddr,
                        buffer,
                        sizeof( buffer ) );
}

// Read a value from an MCP23008 register

int readReg( uint8_t reg )
{
    uint8_t buffer[] = { 0 };

    i2cControllerReadReg( i2cInst,
                          i2cAddr,
                          reg,
                          buffer,
                          sizeof( buffer ) );

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

int main( void )
{
    int cntr = 0;

    stdio_init_all();

    // Give minicom some time to establish a connection
    
    sleep_ms(2500);

    // Output the current configuration
    
    printf( "Rotary Encoder Test\n"
            "Configuring Interrupt for pin %d\n"
            "Configuring I2C with SDA %d SCL %d Freq %d\n"
            "MCP23008 at address 0x%02x\n",
            gpioInt, i2cSDA, i2cSCL, i2cBaud, i2cAddr );
    fflush( stdout );
    
    // Initialize the mutex used to protect
    // console I/O
    
    mutex_init( &ioMutex );

    // Initialize the queue that enables
    // reading the MCP23008
    
    queue_init( &gpioQueue, 0, 10 );
    
    // Initialize interrupt

    gpioInterruptInit();

    // Initialize I2C
    
    i2cControllerInit( i2c0 );

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
        
        queue_remove_blocking( &gpioQueue, NULL );

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
            mutex_enter_blocking( &ioMutex );
            printf( "Posn = %d\n", rotaryPosn );
            mutex_exit( &ioMutex );
            lastRP = rotaryPosn;
        }
        
    } // endwhile

} // app_main
