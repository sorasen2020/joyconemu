#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "btdevice.h"
#include "pokecon.h"

#if _DEBUG
#include "esp_timer.h"
// for debug ESP32-DevkitC-32E ESP32-WROOM-32E
#define UART_TXD_PIN \
	(GPIO_NUM_17)
#define UART_RXD_PIN \
	(GPIO_NUM_16)
#define UART_NUM (UART_NUM_2)
static void periodic_timer_callback(void* arg);
static uint32_t prev_time = 0;
#else
#define UART_TXD_PIN \
	(UART_PIN_NO_CHANGE)	// When UART2, TX GPIO_NUM_19, RX GPIO_NUM_26
#define UART_RXD_PIN \
	(UART_PIN_NO_CHANGE)	// When UART0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE
#define UART_NUM (UART_NUM_0)
#endif

#define BUF_SIZE (256)
#define RD_BUF_SIZE (BUF_SIZE)

static QueueHandle_t uart_queue;

/* Function Prototype */
esp_err_t uart_init(void);
static void uart_event_task(void *pvParameters);

void app_main() {
	const char* TAG = "app_main";
	esp_err_t err;
	int ret;

#if _DEBUG
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        /* name is optional, but may help identify the timer when debugging */
        .name = "periodic"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    /* Start the timers */
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000));
#endif

	err = uart_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_init failed: %s\n", esp_err_to_name(err));
	}

	ret = btdevice_init();
	if (ret != BTDEVICE_OK) {
		ESP_LOGE(TAG, "btdevice_init failed\n");
	}

	xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}

esp_err_t uart_init(void) {
	const char* TAG = "uart_init";
	esp_err_t err;
	uart_config_t uart_config = {
		.baud_rate  = 9600,
		.data_bits  = UART_DATA_8_BITS,
		.parity 	= UART_PARITY_DISABLE,
		.stop_bits  = UART_STOP_BITS_1,
		.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	//Install UART driver, and get the queue.
	err = uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 20, &uart_queue, ESP_INTR_FLAG_IRAM);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_driver_install failed: %s\n", esp_err_to_name(err));
		return err;
	}

	err = uart_param_config(UART_NUM, &uart_config);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_param_config failed: %s\n", esp_err_to_name(err));
		return err;
	}

	//Set UART pins (using UART0 default pins ie no changes.)
	err = uart_set_pin(UART_NUM, UART_TXD_PIN, UART_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_param_config failed: %s\n", esp_err_to_name(err));
	}

	return err;
}

static void uart_event_task(void *pvParameters)
{
	static const char *TAG = "uart_events";
	uart_event_t event;
	pokecon_report_input_t pc_report;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
	int ret;
#if _DEBUG
    uint32_t current_time = 0;
    uint32_t delta = 0;
#endif
    char c;
    int idx;
    int counter;

	for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
#if _DEBUG
                    current_time = esp_timer_get_time();
                    delta = (uint32_t)((double)(current_time - prev_time) / 1000);
                    prev_time = current_time;
                    ESP_LOGI(TAG, "[UART DATA]: %d %ld", event.size, delta);
#else
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
#endif
                    idx = 0;
                    counter = 0;
                    while (counter < event.size) {
                        uart_read_bytes(UART_NUM, &c, 1, portMAX_DELAY);
                        // ESP_LOGI(TAG, "c=%c", c);
                        if ((c != '\n') && (idx < RD_BUF_SIZE)) {
                            dtmp[idx++] = c;
                        }

                        if (c == '\r') {
                            dtmp[idx++] = '\n';
					        char * line = (char *)dtmp;
                            ESP_LOGI(TAG, "%s", line);
                            ret = pokecon_parseline(line, &pc_report);
                            if (ret != EOF) {
                                btdevice_sendreport(pc_report);
                            }
                            idx = 0;
                            memset(dtmp, 0, sizeof(RD_BUF_SIZE));
                        }
                        counter++;
                    }
					//uart_flush_input(UART_NUM);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider increasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED]");
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

#if _DEBUG
static void periodic_timer_callback(void* arg)
{
    int64_t time_since_boot = esp_timer_get_time();
}
#endif