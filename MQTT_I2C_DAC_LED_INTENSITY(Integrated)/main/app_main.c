
/************************LED_INTENSITY***********/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h> 

#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/igmp.h"

#include "esp_wifi.h"
#include "soc/rtc_periph.h"
#include "driver/spi_slave.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"

#include "driver/dac.h"
/*****************************************************conditional compilation for SPI and I2C**********************************/
//#define SPI_ENABLE
#define I2C_ENABLE    
/*********************************************************************************************/
#define MACRO(a) ((a>>4 &0x0f) | (a<<4 & 0xf0)) //nibble swapping
// Pins in use
/*****************************************************SPI PIN Configuration******************************/
#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS 16

/****************************************************************************/
#define I2C_SLAVE_SCL_IO 22  // GPIO number for I2C clock
#define I2C_SLAVE_SDA_IO 21  // GPIO number for I2C data
#define I2C_SLAVE_NUM I2C_NUM_1  // I2C port number 0
#define I2C_SLAVE_TX_BUF_LEN (1024)  // I2C slave TX buffer size
#define I2C_SLAVE_RX_BUF_LEN (1024)  // I2C slave RX buffer size
#define ESP_SLAVE_ADDR 28  // I2C slave address (7-bit)




static const char *ITAG = "i2c-example";
spi_slave_transaction_t t;

 esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };
    
esp_mqtt_client_handle_t client; 

char var[5];
uint8_t recvbuf[10] = {0};
char push_msg[25] = "";
int LED_INTENSITY = 0;
uint8_t I2c_data[10];
static const char *TAG = "mqtt_example";


void SPI_Init(void);
void i2c_slave_init(void);
static void log_error_if_nonzero(const char *message, int error_code);
//static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void mqtt_app_start(void);
void ftoa(float n, char* res, int afterpoint);
int intToStr(int x, char str[], int d);
void reverse(char* str, int len); 

void app_main(void)
{
    
    

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
	SPI_Init();
	i2c_slave_init();	
	dac_output_enable(DAC_CHAN_0);      //enabling channel0 as DAC output
    ESP_ERROR_CHECK(example_connect());
    client = esp_mqtt_client_init(&mqtt_cfg);
    mqtt_app_start();
   
    while (1)  
    {

        #ifdef I2C_ENABLE
                 // Read data from the I2C slave buffer
        int size = i2c_slave_read_buffer(I2C_SLAVE_NUM, recvbuf, sizeof(recvbuf), pdMS_TO_TICKS(1000));
        if (size > 0) { 		
			LED_INTENSITY = recvbuf[0];
   	        LED_INTENSITY=LED_INTENSITY << 8 | recvbuf[1];
   	        printf("Received data %d\n",LED_INTENSITY);
		    LED_INTENSITY = LED_INTENSITY / 252; 
		     sprintf(push_msg, "%d", LED_INTENSITY);           //2bytes to 8 bit conversion
   	     //int dac_value = (Led_inten * 255) / 255;  // Scale the 8-bit value to DAC range
        // Set the DAC output value on GPIO 25 (DAC2)
            dac_output_voltage(DAC_CHAN_0, LED_INTENSITY);
            if(LED_INTENSITY>150)
				strcat(push_msg," ,LED_ON");	
			else
			   strcat(push_msg," ,LED_OFF");
			float voltage=0.0117*LED_INTENSITY;      //   3/255=0.0117u
		 
			
			ftoa(voltage, var, 3);
			printf("volt=%s\n\r",var);
			strcat(push_msg,var);
			   
			   
			   
//	        printf(" the I2c msg %s\n", push_msg);
	        int msg_id = esp_mqtt_client_publish(client,"Led_Intensity", push_msg, 0, 1, 0);
	        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
	        
        }
        #endif
        
        
          #ifdef SPI_ENABLE
    t.length = 10 * 8;
    t.rx_buffer = recvbuf;
    if(spi_slave_transmit(HSPI_HOST, &t, portMAX_DELAY)==ESP_OK)
    {
	   Led_inten = recvbuf[0];
   	   Led_inten=Led_inten << 8 | recvbuf[1];
	   sprintf(push_msg, "%04x", Led_inten);
	   Led_inten = Led_inten / 257;
   	 //  int dac_value = (Led_inten * 255) / 255;  // Scale the 8-bit value to DAC range
        // Set the DAC output value on GPIO 25 (DAC2)
       dac_output_voltage(DAC_CHAN_0, Led_inten);
       if(Led_inten>150)
		   strcat(push_msg," ,LED_ON");	
	   else
		   strcat(push_msg," ,LED_OFF");
	   printf("%s\n", push_msg);
	   int msg_id = esp_mqtt_client_publish(client,"Led_Intensity", push_msg, 0, 1, 0);

	}
	#endif
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
void i2c_slave_init() {
    int i2c_slave_port = I2C_SLAVE_NUM;  // Assign the chosen I2C port to a variable

    i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,  // Specify the GPIO pin for SDA
        .scl_io_num = I2C_SLAVE_SCL_IO,  // Specify the GPIO pin for SCL
        .sda_pullup_en = GPIO_PULLUP_DISABLE,  // Enable pull-up on SDA pin
        .scl_pullup_en = GPIO_PULLUP_DISABLE,  // Enable pull-up on SCL pin
        .mode = I2C_MODE_SLAVE,  // Set mode to I2C slave
        .slave = {
            .addr_10bit_en = 0,  // Use 7-bit address mode
            .slave_addr = ESP_SLAVE_ADDR  // Set the I2C slave address
        }
    };

    // Configure the I2C parameters for the chosen port
    esp_err_t param_config_status = i2c_param_config(i2c_slave_port, &conf_slave);
    if (param_config_status != ESP_OK) {
		printf("initialised");
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(param_config_status));
    }

    // Install the I2C driver
    esp_err_t driver_install_status = i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
    if (driver_install_status != ESP_OK) {
		printf("not init");
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(driver_install_status));
    }
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_app_start(void)
{
	
#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0)
    {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128)
        {
            int c = fgetc(stdin);
            if (c == '\n')
            {
                line[count] = '\0';
                break;
            }
            else if (c > 0 && c < 127)
            {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.broker.address.uri = line;
        printf("Broker url: %s\n", line);
    }
    else
    {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */
  
   
    esp_mqtt_client_start(client);
}

void SPI_Init(void)
{
	spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    // Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
    };

    // Initialize SPI slave interface
    spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
}
void reverse(char* str, int len) 
{ 
	int i = 0, j = len - 1, temp; 
	while (i < j) { 
		temp = str[i]; 
		str[i] = str[j]; 
		str[j] = temp; 
		i++; 
		j--; 
	} 
} 
int intToStr(int x, char str[], int d) 
{ 
	int i = 0; 
	while (x) { 
		str[i++] = (x % 10) + '0'; 
		x = x / 10; 
	} 


	while (i < d) 
		str[i++] = '0'; 

	reverse(str, i); 
	str[i] = '\0'; 
	return i; 
} 

// Converts a floating-point/double number to a string. 
void ftoa(float n, char* res, int afterpoint) 
{ 

	int ipart = (int)n; 

 
	float fpart = n - (float)ipart; 

	int i = intToStr(ipart, res, 0); 


	if (afterpoint != 0) { 
		res[i] = '.'; // add dot 
		fpart = fpart * pow(10, afterpoint); 

		intToStr((int)fpart, res + i + 1, afterpoint); 
	} 
} 