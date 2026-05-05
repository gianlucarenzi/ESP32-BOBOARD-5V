#include <Arduino.h>
#include <driver/gpio.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_struct.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <pbi-driver.h> // Include PBI driver memory space

/* ANSI Eye-Candy ;-) */
#define ANSI_RED     "\x1b[31m"
#define ANSI_GREEN   "\x1b[32m"
#define ANSI_YELLOW  "\x1b[1;33m"
#define ANSI_BLUE    "\x1b[1;34m"
#define ANSI_MAGENTA "\x1b[1;35m"
#define ANSI_CYAN    "\x1b[1;36m"
#define ANSI_WHITE   "\x1b[1;37m"
#define ANSI_RESET   "\x1b[0m"

#define SERIAL_QUEUE_LENGTH 32 // Length of the serial queue
#define SERIAL_MSG_SIZE 128 // Size of the serial message buffer

// --- HARDWARE MAPPING ---
// Data Bus D0-D7: GPIO 2, 4, 5, 12, 13, 14, 15, 18
// Address A0-A5: GPIO 32, 33, 34, 35, 36, 39
// Address A6-A7: GPIO 16, 17
// Address A8: GPIO 22
// PHI2: GPIO 23, R/W: GPIO 25, CCTL: GPIO 26, D1XX: GPIO 27
// Outputs: ~MPD: GPIO 19, ~EXTSEL: GPIO 21

constexpr gpio_num_t PIN_D0 = GPIO_NUM_2;
constexpr gpio_num_t PIN_D1 = GPIO_NUM_4;
constexpr gpio_num_t PIN_D2 = GPIO_NUM_5;
constexpr gpio_num_t PIN_D3 = GPIO_NUM_12;
constexpr gpio_num_t PIN_D4 = GPIO_NUM_13;
constexpr gpio_num_t PIN_D5 = GPIO_NUM_14;
constexpr gpio_num_t PIN_D6 = GPIO_NUM_15;
constexpr gpio_num_t PIN_D7 = GPIO_NUM_18;

constexpr gpio_num_t PIN_A0 = GPIO_NUM_32;
constexpr gpio_num_t PIN_A1 = GPIO_NUM_33;
constexpr gpio_num_t PIN_A2 = GPIO_NUM_34;
constexpr gpio_num_t PIN_A3 = GPIO_NUM_35;
constexpr gpio_num_t PIN_A4 = GPIO_NUM_36;
constexpr gpio_num_t PIN_A5 = GPIO_NUM_39;
constexpr gpio_num_t PIN_A6 = GPIO_NUM_16;
constexpr gpio_num_t PIN_A7 = GPIO_NUM_17;
constexpr gpio_num_t PIN_A8 = GPIO_NUM_22;

constexpr gpio_num_t PIN_PHI2 = GPIO_NUM_23;
constexpr gpio_num_t PIN_RW   = GPIO_NUM_25;
constexpr gpio_num_t PIN_CCTL = GPIO_NUM_26;
constexpr gpio_num_t PIN_D1XX = GPIO_NUM_27;

constexpr gpio_num_t PIN_MPD    = GPIO_NUM_19;
constexpr gpio_num_t PIN_EXTSEL = GPIO_NUM_21;
constexpr gpio_num_t PIN_CS     = GPIO_NUM_26; // Shared or used for VERA

#define MASK_PHI2   (1UL << 23)
#define MASK_RW     (1UL << 25)
#define MASK_CCTL   (1UL << 26)
#define MASK_D1XX   (1UL << 27)

#define MASK_DATA_LOW ( (1UL << 2) | (1UL << 4) | (1UL << 5) | (1UL << 12) | \
                        (1UL << 13) | (1UL << 14) | (1UL << 15) | (1UL << 18) )

// Shadow RAM PBI Driver Memory Space
static char ram_d800[2048] = {0};
static char ram_d600[512] = {0};
static uint8_t d500[256] = {0}; // CCTL memory space
static uint8_t cardselected = 0; // Flag to indicate if a card is selected
static uint8_t DEVICE_ID = 0x01; // Example device ID for PBI I/O
extern const uint8_t pbi_driver[]; // PBI driver memory space

#define DBG_ERROR   0
#define DBG_INFO    1
#define DBG_VERBOSE 2
#define DBG_NOISY   3

static int debuglevel = DBG_INFO;

#define CARTRIDGE_CONTROL (!(gpio_low & MASK_CCTL))
#define PBI_IO            (!(gpio_low & MASK_D1XX))

QueueHandle_t serialQueue;

// Prototypes for functions
void SerialTask(void *pvParameters);
void MonitorTask(void *pvParameters);

void SerialTask(void *pvParameters)
{
	char msg[SERIAL_MSG_SIZE];
	Serial.println(ANSI_BLUE "Serial Task Started on Core 0" ANSI_RESET);
	while (true) {
		if (xQueueReceive(serialQueue, msg, portMAX_DELAY) == pdTRUE) {
			Serial.println(msg);
		}
	}
}

void serialPrintQueue(const char* fmt, ...)
{
	char buf[SERIAL_MSG_SIZE];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, SERIAL_MSG_SIZE, fmt, args);
	va_end(args);
	if (serialQueue != NULL) {
		xQueueSend(serialQueue, buf, 0);
	}
}

inline uint32_t read_gpio_low(void)  { return REG_READ(GPIO_IN_REG); }
inline uint32_t read_gpio_high(void) { return REG_READ(GPIO_IN1_REG); }

static inline uint16_t read_address_bus(uint32_t gpio_low, uint32_t gpio_high)
{
	// A0-A5 from gpio_high: 32, 33, 34, 35, 36, 39
	uint16_t addr = (gpio_high & 0x0F);          // A0-A3 (bits 0,1,2,3 -> GPIO 32,33,34,35)
	addr |= (gpio_high & 0x10);                 // A4 (bit 4 -> GPIO 36)
	addr |= (gpio_high & 0x80) >> 2;            // A5 (bit 7 -> bit 5 -> GPIO 39)
	
	// A6-A7 from gpio_low: 16, 17
	addr |= (gpio_low & 0x00030000) >> 10;      // A6-A7 (bits 16,17 -> 6,7)
	
	// A8 from gpio_low: 22
	addr |= (gpio_low & 0x00400000) >> 14;      // A8 (bit 22 -> 8)
	return addr;
}

static inline uint8_t read_data_bus(uint32_t gpio_low)
{
	return (((gpio_low >> 2) & 1) << 0) |
	       (((gpio_low >> 4) & 1) << 1) |
	       (((gpio_low >> 5) & 1) << 2) |
	       (((gpio_low >> 12) & 1) << 3) |
	       (((gpio_low >> 13) & 1) << 4) |
	       (((gpio_low >> 14) & 1) << 5) |
	       (((gpio_low >> 15) & 1) << 6) |
	       (((gpio_low >> 18) & 1) << 7);
}

static inline void set_data_bus_direction(gpio_mode_t mode)
{
	if (mode == GPIO_MODE_OUTPUT) {
		GPIO.enable_w1ts = MASK_DATA_LOW;
	} else {
		GPIO.enable_w1tc = MASK_DATA_LOW;
	}
}

static uint32_t data_set_lut[256];

static void precompute_data_lut(void)
{
	const uint32_t pin_bits[8] = {
		(1U << PIN_D0), (1U << PIN_D1), (1U << PIN_D2), (1U << PIN_D3),
		(1U << PIN_D4), (1U << PIN_D5), (1U << PIN_D6), (1U << PIN_D7)
	};
	for (int v = 0; v < 256; v++) {
		uint32_t m = 0;
		for (int b = 0; b < 8; b++) {
			if ((v >> b) & 1) m |= pin_bits[b];
		}
		data_set_lut[v] = m;
	}
}

static inline void write_data_bus(uint8_t value)
{
	uint32_t set_mask = data_set_lut[value];
	GPIO.out_w1ts = set_mask;
	GPIO.out_w1tc = MASK_DATA_LOW & ~set_mask;
}

static inline void fast_gpio_set_level(gpio_num_t pin, uint32_t level)
{
	if (level) {
		if (pin < 32) GPIO.out_w1ts = (1U << pin);
		else GPIO.out1_w1ts.val = (1U << (pin - 32));
	} else {
		if (pin < 32) GPIO.out_w1tc = (1U << pin);
		else GPIO.out1_w1tc.val = (1U << (pin - 32));
	}
}

void setup(void)
{
	Serial.begin(115200); 
	precompute_data_lut();
	serialQueue = xQueueCreate(SERIAL_QUEUE_LENGTH, SERIAL_MSG_SIZE);
	xTaskCreatePinnedToCore(SerialTask, "SerialTask", 2048, NULL, 1, NULL, 0); 
	xTaskCreatePinnedToCore(MonitorTask, "MonitorTask", 4096, NULL, 10, NULL, 1); 

	gpio_config_t io_conf = {};
	// Inputs (Low Reg): PHI2, RW, CCTL, D1XX, A8, A6, A7
	io_conf.pin_bit_mask = MASK_PHI2 | MASK_RW | MASK_CCTL | MASK_D1XX | \
	                       (1ULL << PIN_A6) | (1ULL << PIN_A7) | (1ULL << PIN_A8);
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpio_config(&io_conf);

	// Inputs (High Reg): A0-A5 (32, 33, 34, 35, 36, 39)
	io_conf.pin_bit_mask = (1ULL << 32) | (1ULL << 33) | (1ULL << 34) | \
	                       (1ULL << 35) | (1ULL << 36) | (1ULL << 39);
	gpio_config(&io_conf);

	// Data Bus (Input initially)
	io_conf.pin_bit_mask = MASK_DATA_LOW;
	gpio_config(&io_conf);

	// Outputs: MPD, EXTSEL
	gpio_set_direction(PIN_MPD, GPIO_MODE_OUTPUT);
	gpio_set_direction(PIN_EXTSEL, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_MPD, 1);
	gpio_set_level(PIN_EXTSEL, 1);

	// Configurazione CS come output
	gpio_set_direction(PIN_CS, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_CS, 1);
}

void MonitorTask(void *pvParameters)
{
	serialPrintQueue(ANSI_BLUE "High-Speed 6502 Bus Monitor Ready (XL/XE)\n" ANSI_RESET);
	for (;;) {
		uint32_t gpio_low;
		do { gpio_low = REG_READ(GPIO_IN_REG); } while (!(gpio_low & MASK_PHI2));

		if (!(gpio_low & (MASK_CCTL | MASK_D1XX))) {
			uint32_t gpio_high = REG_READ(GPIO_IN1_REG);
			uint16_t address = read_address_bus(gpio_low, gpio_high);
			bool rw = (gpio_low & MASK_RW) != 0;

			if (CARTRIDGE_CONTROL) {
				if (rw) {
					uint8_t data = d500[address & 0xFF];
					write_data_bus(data);
					set_data_bus_direction(GPIO_MODE_OUTPUT);
					while (REG_READ(GPIO_IN_REG) & MASK_PHI2) ;;
					set_data_bus_direction(GPIO_MODE_INPUT);
					if (debuglevel >= DBG_VERBOSE) serialPrintQueue("CCTL R: %04X -> %02X\n", address, data);
				} else {
					while (REG_READ(GPIO_IN_REG) & MASK_PHI2) ;;
					d500[address & 0xFF] = read_data_bus(REG_READ(GPIO_IN_REG));
					if (debuglevel >= DBG_VERBOSE) serialPrintQueue("CCTL W: %04X <- %02X\n", address, d500[address & 0xFF]);
				}
			} else if (PBI_IO) {
				uint8_t addressLSB = address & 0xFF;
				if (addressLSB == 0xFF) {
					if (!rw) {
						while (REG_READ(GPIO_IN_REG) & MASK_PHI2) ;;
						uint8_t device = read_data_bus(REG_READ(GPIO_IN_REG));
						cardselected = (device == DEVICE_ID);
						fast_gpio_set_level(PIN_CS, !cardselected);
						if (debuglevel >= DBG_INFO) serialPrintQueue("PBI SEL: %02X (%s)\n", device, cardselected ? "OK" : "NO");
					} else {
						uint8_t data = cardselected ? DEVICE_ID : 0;
						write_data_bus(data);
						set_data_bus_direction(GPIO_MODE_OUTPUT);
						while (REG_READ(GPIO_IN_REG) & MASK_PHI2) ;;
						set_data_bus_direction(GPIO_MODE_INPUT);
					}
				}
			}
		}
		while (REG_READ(GPIO_IN_REG) & MASK_PHI2) ;;
	}
}

void loop(void) { vTaskDelay(pdMS_TO_TICKS(1)); }
