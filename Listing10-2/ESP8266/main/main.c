// Listing 10-2 (main.c)
// Espressif ESP8266
// FreeRTOS
// Espressif ESP8266 RTOS SDK
// https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/index.html
//
// Multi-threaded I2C demonstration #2.
// This program writes to two separate
// MCP4725 devices on the same I2C bus
// using separate threads for each of the
// DACs, with FreeRTOS Task Notifications
// to protect writes to the I2C port.
//
// Tested devices:
//                                          ADDRS   SDA     SCL
//     Espressif ESP8266 DevKitC V4                  14       2
//     Adafruit Huzzah32 ESP8266 Feather              4       5
//     SparkFun MCP4725                      0x60
//                                           0x61
//     Adafruit MCP4725                      0x62
//                                           0x63

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"
#include "driver/i2c.h"


// Mutex protecting console I/O
// and Task Notifications protecting I2C:
SemaphoreHandle_t ioMutex      = NULL;
TaskHandle_t      mainNotify   = NULL;
TaskHandle_t      threadNotify = NULL;


// I2cControllerInit-
//
// Initialize the I2C controller.
// Various parameters are defined using the
// Espressif RTOS SDK menuconfig utility.
esp_err_t I2cControllerInit( bool pullup )
{
    // Configure an i2c driver
    
    i2c_port_t   i2c_master_port = (i2c_port_t) CONFIG_I2C_CONTROLLER_PORT_NUM;
    i2c_config_t config;
    memset( &config, 0, sizeof( config ) );

    config.mode             = I2C_MODE_MASTER;
    config.sda_io_num       = (gpio_num_t) CONFIG_I2C_CONTROLLER_SDA;
    config.scl_io_num       = (gpio_num_t) CONFIG_I2C_CONTROLLER_SCL;
    config.clk_stretch_tick = 300;
    
    if ( pullup )
    {
        config.sda_pullup_en = GPIO_PULLUP_ENABLE;
        config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    }

    // Install an i2c driver
    
    esp_err_t err = i2c_driver_install( i2c_master_port, config.mode );
    if ( err == ESP_OK )
    {
        err = i2c_param_config( i2c_master_port, &config );
    }
    
    return err;
}


// I2cControllerWrite-
//
// Send a string of bytes to an I2C peripheral
esp_err_t I2cControllerWrite( i2c_port_t  port,
                              uint8_t     address,
                              uint8_t    *data,
                              size_t      size,
                              bool        ackEnabled )
{
   // Create a command link
   i2c_cmd_handle_t command = i2c_cmd_link_create();
   esp_err_t        err     = ESP_OK;
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


// WriteToDAC-
//
// Write a value to an MCP4725
// and use task notifications to protect
// I2C operations
esp_err_t WriteToDAC(uint8_t       port,
                     uint8_t       address,
                     uint16_t      value,
                     TaskHandle_t  take,
                     TaskHandle_t  give)
{
    esp_err_t err = ESP_FAIL;

    // Acquire the I2C bus in the current task,
    // this may cause the current task to block
    // for a while.
    ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

    // Note: MCP4725 requires that we write
    // the HO byte first and the LO byte second!
    uint8_t   buffer[] = {
        (uint8_t) ( ( value >> 8 ) & 0xf ),
        (uint8_t) ( value & 0xff )
    };
    
    err = I2cControllerWrite( port,
                              address,
                              buffer,
                              sizeof(buffer),
                              true);    // ACK check enabled

    // Give up the I2C bus in the OTHER task,
    // leaving this task with ownership. As a
    // result of retaining ownership this task
    // will block on the next call. The OTHER
    // task will do something similar and unblock
    // this task. The result of all this is that
    // the two tasks will alternate their
    // writes to the MCP4725.
    xTaskNotifyGive( give );

    return err;
}


// DACout-
//
// Draws one cycle of a triangle waveform on
// the MCP4725 I2C DAC device (e.g., Adafruit
// MCP4725 breakout board).
//
// Arguments: I2C port and address for the DAC,
// current (take) and other (give) task handles.
// For Adafruit MCP4725 breakout boards, the
// address is either 0x62 or 0x63. For Sparkfun
// boards, this is either 0x60 or 0x61.

void DACout( uint8_t       port,
             uint8_t       address,
             TaskHandle_t  take,
             TaskHandle_t  give )
{
    
    // Send the rising edge of a triangle wave:

    for( uint16_t dacOut = 0; dacOut < 0xfff; ++dacOut )
    {
        esp_err_t err = WriteToDAC( port, address, dacOut, take, give );
        ESP_ERROR_CHECK_WITHOUT_ABORT( err );
    }

    // Send the falling edge of the triangle wave

    for( uint16_t dacOut = 0xffe; dacOut > 0; --dacOut )
    {
        esp_err_t err = WriteToDAC( port, address, dacOut, take, give );
        ESP_ERROR_CHECK_WITHOUT_ABORT( err );
    }
}


void DACthread( void *parm )
{
    int cntr = 0;

    while( true )
    {
        // Print a message each time the thread
        // completes one cycle of the triangle
        // wave. Note that printf must be
        // protected by a mutex.
        
        xSemaphoreTake( ioMutex, portMAX_DELAY );
        printf( "thread loop, cntr=%d\n", cntr++ );
        xSemaphoreGive( ioMutex );

        // Draw one cycle of the triangle waveform
        // on the second DAC

        DACout( CONFIG_I2C_CONTROLLER_PORT_NUM,
                CONFIG_I2C_MCP4725_ADDRESS_2,
                threadNotify,
                mainNotify );

        // Let the watchdog timer know we are running

        esp_task_wdt_reset();
        
    } // endwhile

}


void app_main( void )
{
    int cntr = 0;

    // Get the priority of the current task
    UBaseType_t priority = uxTaskPriorityGet( NULL );
    
    printf( "Configuring I2C with SDA %d SCL %d\n"
            "MCP4725 at addresses 0x%02x 0x%02x\n"
            "main running at priority %d\n",
            CONFIG_I2C_CONTROLLER_SDA, CONFIG_I2C_CONTROLLER_SCL,
            CONFIG_I2C_MCP4725_ADDRESS_1, CONFIG_I2C_MCP4725_ADDRESS_2,
            priority );
    fflush( stdout );

    // Initialize I2C
    
    esp_err_t err = I2cControllerInit( true );
    ESP_ERROR_CHECK_WITHOUT_ABORT( err );

    // Initialize the mutex used to protect
    // console I/O and task handle that protects
    // the main task. The thread task handle
    // will be initialized when we create
    // the task.
    
    ioMutex = xSemaphoreCreateMutex();
    configASSERT( ioMutex );
    mainNotify = xTaskGetCurrentTaskHandle();
    configASSERT( mainNotify );
    
    // Start a thread running that will write to the
    // second DAC. Give the thread a higher
    // priority than the main thread.
    //
    // The parameters are the following:
    //
    //  1. address of function to invoke as the
    //     new thread ("task" in FreeRTOS terminology).
    //
    //  2. name of the thread, used for debugging.
    //
    //  3. size of the stack in bytes.
    //
    //  4. parameter to pass to the thread.
    //
    //  5. Thread priority (higher number is
    //     higher priority).
    //
    //  6. Address to store the handle for created task.
    
    BaseType_t result = xTaskCreate(DACthread,
                                    "Triangle 2",
                                    2048, // Stack depth
                                    NULL,
                                    priority,
                                    &threadNotify);
    assert( result == pdPASS );
    configASSERT( threadNotify );
    
    // Let the main task be unblocked
    
    xTaskNotifyGive( mainNotify );

    while( true )
    {
        // Print a message each time the main thread
        // completes one cycle of the triangle
        // wave. Note that printf must be
        // protected by a mutex.
        
        xSemaphoreTake( ioMutex, portMAX_DELAY );
        printf( "main loop, cntr=%d\n", cntr++ );
        xSemaphoreGive( ioMutex );

        // Draw one cycle of the triangle waveform
        // on the first DAC

        DACout( CONFIG_I2C_CONTROLLER_PORT_NUM,
                CONFIG_I2C_MCP4725_ADDRESS_1,
                mainNotify,
                threadNotify );

        // Let the watchdog timer know we are running

        esp_task_wdt_reset();

    } // endwhile

} // app_main
