
#include <stdio.h>
#include <string.h>
#include <string>
#include <time.h>
#include <queue>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

extern "C" {
    void app_main(void);
}

class Problem
{
    public:
        int id;
        int time;
        int capacity;
        std::queue<int> weights; 
        std::queue<int> values;
        int ts_begin;
        int ts_end;
};

static void knapsack_worker(void *arg);
static Problem* parse_problem(std::string problem);
static void init_workers(char* cant);
static void knapsack_master(void *arg);
void app_main(void);

#define BUF_SIZE (1024)

std::queue<std::string> cola;

std::queue<std::string> colaSalida;

// SemaphoreHandle_t xSemaphoreInput;
// SemaphoreHandle_t xSemaphoreOutput;

uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .use_ref_tick = false
};

static Problem* parse_problem(std::string problem)
{
    char* token;
    std::string split = ",";
    char* problem_aux = new char[cola.front().size() + 1];
    strcpy(problem_aux, cola.front().c_str());
    Problem* p = new Problem();
    // extract the problem id, the first character is ignored because its a '#'
    token = strtok(problem_aux, split.c_str());
    p->id = atoi(token + 1);

    // extract the capacity of the bag
    token = strtok(NULL, split.c_str());
    p->capacity = atoi(token);

    while (token != NULL) {
        token = strtok(NULL, split.c_str());
        if (token != NULL) {
            p->weights.push(atoi(token));
            token = strtok(NULL, split.c_str());
            if (token != NULL) {
                p->values.push(atoi(token));
            }
        }
    }
    delete problem_aux;
    return p;
}

static void init_workers(char* cant_str)
{
    int cant = atoi(cant_str);
    int i = 0;
    for(i = 0; i < cant; i++) {
         xTaskCreate(knapsack_worker, "knapsack_worker", 1024, (void*)i, 10, NULL);
    }
}

static void knapsack_master(void *arg)
{
    // Configure a temporary buffer for the incoming data
    char *data = (char *) malloc(BUF_SIZE);
    int cont = 0;
    bool first = true;
    while (1) {
        // Read data from the UART
        uint32_t len = uart_read_bytes(UART_NUM_0, (uint8_t *)(data + cont), BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        uart_write_bytes(UART_NUM_0, (const char *) (data + cont), len);
        if(len > 0) {
            cont += len;
            if(data[cont - 1] == '*') {
                data[cont - 1] = '\0';
                if (first) {
                    init_workers(&data[1]);
                    first = false;
                } else {
                    std::string problem(data);
                    cola.push(problem);
                }
                cont =  0;
            }
            
        }
    }
}

static void knapsack_worker(void *arg)
{
    char* data = (char *) malloc(sizeof(char)*BUF_SIZE);
    uint32_t tid = (uint32_t)arg;
    const TickType_t xDelay = 10000 / portTICK_PERIOD_MS;
    std::string problem;
    uint32_t xTime = clock() / CLOCKS_PER_SEC;

    sprintf(data, "Worker Thread id:%d initialized\n", tid);
    uart_write_bytes(UART_NUM_0, (const char *)data, strlen(data));
    // Configure a temporary buffer for the incoming data
    while (1) {
        xTime = clock() / CLOCKS_PER_SEC;
        // Write data back to the UART
        sprintf(data, "%d, %s \n", xTime, "Hola ");
        uart_write_bytes(UART_NUM_0, (const char *)data, strlen(data));
        if(!cola.empty()) {
            // problem = new char[cola.front().size() + 1];
            // strcpy(problem, cola.front().c_str());
            problem = cola.front();
            cola.pop();
            // sprintf(data, "'%s'\n\n", problem);
            // uart_write_bytes(UART_NUM_0, (const char *)data, strlen(data));
            Problem* p = parse_problem(problem);

            
            // uart_write_bytes(UART_NUM_0, (const char *)data, strlen(data));
            sprintf(data, "%d %d %s\n", p->weights.size(), p->values.size(), ";");
            uart_write_bytes(UART_NUM_0, (const char *)data, strlen(data));
            delete p;
        }
        vTaskDelay(xDelay);
    }
}

void app_main(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_param_config(UART_NUM_0, &uart_config);

    // xSemaphoreInput = xSemaphoreCreateMutex();
    // xSemaphoreOutput = xSemaphoreCreateMutex();

    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
    xTaskCreate(knapsack_master, "knapsack_master", 1024, NULL, 10, NULL);
}
