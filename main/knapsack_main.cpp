
#include <stdio.h>
#include <string.h>
#include <string>
#include <time.h>
#include <queue>
#include <deque>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

extern "C"
{
    void app_main(void);
}

class Problem
{
    public:
        int id;
        int time;
        int tid;
        int capacity;
        int number_elements;
        std::deque<int> weights; 
        std::deque<int> values;
        int ts_begin;
        int ts_end;
        int solution;

        void print(char* str)
        {
            time = ts_end - ts_begin;
            sprintf(str, "#%d,%d,%d|%d|%d,%d,%d", tid, id, capacity, solution, ts_begin, ts_end, time);
        }
};

static uint32_t get_time(void);
static void print(char* data);
static void print(char* data, int length);
static void knapsack_worker(void *arg);
static Problem* parse_problem(std::string problem);
int solve_knapSack(Problem* problem);
int knapSack(int W, int wt[], int val[], int n);
static void init_workers(char* cant);
static void knapsack_master(void *arg);
void app_main(void);


// constants and global variables
#define BUF_SIZE (1024)
std::queue<std::string> cola;
std::queue<std::string> colaOut;

SemaphoreHandle_t xSemaphoreInput;
SemaphoreHandle_t xSemaphoreOutput;
SemaphoreHandle_t xSemaphorePrint;

uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .use_ref_tick = false
};

static uint32_t get_time(void)
{
    // return clock() / (CLOCKS_PER_SEC / 100);
    return xTaskGetTickCount();
    // return clock();
}

static void print(char* data)
{
    int length = strlen(data);
    print(data, length);
}

static void print(char* data, int length)
{
    while (xSemaphoreTake(xSemaphorePrint, ( TickType_t )100) != pdTRUE);
    uart_write_bytes(UART_NUM_0, (const char *)data, length);
    xSemaphoreGive(xSemaphorePrint);
}

static Problem* parse_problem(std::string problem)
{
    char* token;
    std::string split = ",";
    char* problem_aux = new char[problem.size() + 1];
    strcpy(problem_aux, problem.c_str());
    Problem* p = new Problem();
    p->ts_begin = get_time();
    // extract the problem id, the first character is ignored because its a '#'
    token = strtok(problem_aux, split.c_str());
    p->id = atoi(token + 1);

    // extract the capacity of the bag
    token = strtok(NULL, split.c_str());
    p->capacity = atoi(token);

    while (token != NULL)
    {
        token = strtok(NULL, split.c_str());
        if (token != NULL) {
            p->weights.push_back(atoi(token));
            token = strtok(NULL, split.c_str());
            if (token != NULL) {
                p->values.push_back(atoi(token));
            }
        }
    }
    p->number_elements = p->values.size();
    delete problem_aux;
    return p;
}

// A utility function that returns maximum of two integers 
int max(int a, int b) { return (a > b)? a : b; } 
  
// Returns the maximum value that can be put in a knapsack of capacity W 
int knapSack(int W, std::deque<int>wt, std::deque<int>val, int n) 
{ 
   int i, w; 
    int K[n+1][W+1];
  
    // Build table K[][] in bottom up manner 
    for (i = 0; i <= n; i++) 
    { 
        for (w = 0; w <= W; w++) 
        { 
            if (i==0 || w==0) 
                K[i][w] = 0; 
            else if (wt[i-1] <= w) 
                    K[i][w] = max(val[i-1] + K[i-1][w-wt[i-1]],  K[i-1][w]); 
            else
                    K[i][w] = K[i-1][w]; 
        } 
    } 
    return K[n][W]; 
}

int solve_knapSack(Problem* p)
{
    return knapSack(p->capacity, p->weights, p->values, p->number_elements);
}
  

static void init_workers(char* cant_str)
{
    int cant = atoi(cant_str);
    int i = 0;
    for(i = 0; i < cant; i++)
    {
         xTaskCreate(knapsack_worker, "knapsack_worker", 8192, (void*)i, 10, NULL);
    }
}

static void knapsack_master(void *arg)
{
    // Configure a temporary buffer for the incoming data
    char* pch;
    char* data = (char *) malloc(BUF_SIZE);
    char* dataOut = (char *) malloc(BUF_SIZE);
    std::string dataOutStr;
    int cont = 0;
    bool first = true;
    while (1)
    {
        // Read data from the UART
        uint32_t len = uart_read_bytes(UART_NUM_0, (uint8_t *)(data + cont), BUF_SIZE, 20 / portTICK_PERIOD_MS);
        // Write data back to the UART
        print((data + cont), len);
        if(len > 0)
        {
            cont += len;
            if(data[cont - 1] == '*')
            {
                // data[cont - 1] = '\0';
                data[cont] = '\0';
                if (first)
                {
                    init_workers(&data[1]);
                    first = false;
                } else {
                    pch = strtok(data, "*");
                    while (pch != NULL)
                    {
                        // std::string problem(data);
                        std::string problem(pch);
                        while (xSemaphoreTake(xSemaphoreInput, ( TickType_t )100) != pdTRUE);
                        cola.push(problem);
                        xSemaphoreGive(xSemaphoreInput);
                        pch = strtok (NULL, "*");
                    }
                    sprintf(data, "Problem added\n");
                    print(data);
                }
                cont =  0;
            }
        }

        // check if an output is available and send it through UART
        if(xSemaphoreTake(xSemaphoreOutput, ( TickType_t )100) == pdTRUE) {
            if (!colaOut.empty()) {
                dataOutStr = colaOut.front();
                colaOut.pop();
                xSemaphoreGive(xSemaphoreOutput);

                sprintf(dataOut, "\tSalida: %s\n", dataOutStr.c_str());
                print(dataOut);
            } else {
                xSemaphoreGive(xSemaphoreOutput);
            }
        }
    }
}

static void knapsack_worker(void *arg)
{
    char* data = (char *) malloc(sizeof(char)*BUF_SIZE);
    uint32_t tid = (uint32_t)arg;
    const TickType_t xDelay = 100 / portTICK_PERIOD_MS;
    std::string problem;

    sprintf(data, "Worker Thread id:%d initialized\n", tid);
    print(data);
    // Configure a temporary buffer for the incoming data
    while (1) {
        // sprintf(data, "Worker Thread id:%d loop, time: %d, core# %d \n", tid, get_time(), xPortGetCoreID());
        // print(data);
        if (xSemaphoreTake(xSemaphoreInput, ( TickType_t )100) == pdTRUE) {
            if (!cola.empty()) {
                problem = cola.front();
                cola.pop();
                xSemaphoreGive(xSemaphoreInput);

                Problem* p = parse_problem(problem);
                p->tid = tid;

                p->solution = solve_knapSack(p);
                // vTaskDelay(xDelay);
                p->ts_end = get_time();
                p->print(data);
                std::string dataString(data);
                while (xSemaphoreTake(xSemaphoreOutput, ( TickType_t )100) != pdTRUE);
                colaOut.push(dataString);
                xSemaphoreGive(xSemaphoreOutput);
                
                delete p;
            } else {
                xSemaphoreGive(xSemaphoreInput);
                vTaskDelay(xDelay);
            }
        }
    }
}

void app_main(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Create Mutex for the queues and the Output
    xSemaphoreInput = xSemaphoreCreateMutex();
    xSemaphoreOutput = xSemaphoreCreateMutex();
    xSemaphorePrint = xSemaphoreCreateMutex();

    // Create the Master Thread
    xTaskCreate(knapsack_master, "knapsack_master", 1024, NULL, 10, NULL);
    // vTaskStartScheduler();
}
