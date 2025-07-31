#include <Arduino.h>
#include <driver/gpio.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_struct.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <pbi-driver.h> // Include PBI driver memory space

/* ANSI Eye-Candy ;-) */
#define ANSI_RED    "\x1b[31m"
#define ANSI_GREEN  "\x1b[32m"
#define ANSI_YELLOW "\x1b[1;33m"
#define ANSI_BLUE   "\x1b[1;34m"
#define ANSI_RESET  "\x1b[0m"

#define TEST

#ifdef TEST
// Definizione delle coppie di pin da testare: { Pin_Output, Pin_Input }
// Assicurati di collegare fisicamente questi pin tra loro con un cavo jumper.
// I pin sul lato destro (Input) verranno testati leggendo il valore scritto sul pin di sinistra (Output).
// Attenzione: i pin 34, 35, 36, 39 sono solo INPUT. Il loro partner sul lato sinistro DEVE essere un pin configurabile come OUTPUT.
const int testPairs[][2] = {
	{2, 4},   // D2, D4
	{5, 12},  // D5, D12
	{13, 14}, // D13, D14
	{15, 18}, // D15, D18
	{19, 21}, // D19, D21
	{22, 23}, // D22, D23
	{25, 26}, // D25, D26
	{27, 32}, // D27, D32
	{33, 34}, // D33, D34 (D34 è solo INPUT)
	{17, 35}, // TX2, D35 (D35 è solo INPUT)
	{16, 39}  // RX2, VN (VN/GPIO39 è solo INPUT)
};

const int numPairs = sizeof(testPairs) / sizeof(testPairs[0]);

// Funzione per verificare se un pin è solo input
bool isOnlyInputPin(int pin)
{
	return (pin == 34 || pin == 35 || pin == 36 || pin == 39);
}

void setup(void)
{
	Serial.begin(115200); // Inizializza la comunicazione seriale
	while (!Serial);      // Attendi che la porta seriale sia disponibile

	Serial.println(ANSI_BLUE "--- Avvio Test Automatico GPIO ESP32 ---" ANSI_RESET);
	Serial.println(ANSI_YELLOW "Assicurati che i pin siano fisicamente collegati a coppie come definito nel codice.");
	Serial.println("-----------------------------------------------------------------------------------" ANSI_RESET);
	Serial.println("Configurazione delle coppie:");
	for (int i = 0; i < numPairs; i++)
	{
		Serial.print(ANSI_YELLOW "  Pair "); Serial.print(i + 1); Serial.print(": Output GPIO"); Serial.print(testPairs[i][0]);
		Serial.print(" <-> Input GPIO" ANSI_RESET); Serial.println(testPairs[i][1]);
	}
	Serial.println("---------------------------------------");
	delay(2000); // Breve pausa prima di iniziare i test
}

void loop(void)
{
	static unsigned long lastTestTime = 0;
	const unsigned long testInterval = 5000; // Intervallo tra cicli di test completi (5 secondi)

	if (millis() - lastTestTime >= testInterval)
	{
		lastTestTime = millis();
		Serial.println(ANSI_BLUE "\n--- Inizio un nuovo ciclo di test ---" ANSI_RESET);
		int failedTests = 0;

		for (int i = 0; i < numPairs; i++)
		{
			int outputPin = testPairs[i][0];
			int inputPin = testPairs[i][1];

			Serial.print(ANSI_YELLOW "Testing Pair: Output GPIO");
			Serial.print(outputPin);
			Serial.print(" <-> Input GPIO");
			Serial.print(inputPin);
			Serial.print(" -- " ANSI_RESET);

			// Salta il test di output se il pin di output è un pin solo input (errore di configurazione)
			if (isOnlyInputPin(outputPin))
			{
				Serial.println(ANSI_RED "ERRORE: Pin " + String(outputPin) + " (Output) è un pin SOLO INPUT. Questa coppia non può funzionare come previsto." ANSI_RESET);
				failedTests++;
				continue; // Passa alla prossima coppia
			}

			// Configura i pin
			pinMode(outputPin, OUTPUT);
			pinMode(inputPin, INPUT); // Non usiamo PULLUP qui perché il pin è pilotato attivamente

			bool pairFailed = false;

			// --- Test HIGH ---
			digitalWrite(outputPin, HIGH);
			delay(10); // Breve ritardo per stabilizzare il segnale
			int readStateHigh = digitalRead(inputPin);

			if (readStateHigh == HIGH)
			{
				//Serial.println(ANSI_GREEN "HIGH Test: PASS" ANSI_RESET); // Troppo verboso, solo in caso di fallimento
			}
			else
			{
				Serial.println(ANSI_RED "FAIL (HIGH Test): Expected HIGH on GPIO" + String(inputPin) + ", but got LOW." ANSI_RESET);
				pairFailed = true;
			}

			// --- Test LOW ---
			digitalWrite(outputPin, LOW);
			delay(10); // Breve ritardo per stabilizzare il segnale
			int readStateLow = digitalRead(inputPin);

			if (readStateLow == LOW)
			{
				//Serial.println(ANSI_GREEN "LOW Test: PASS" ANSI_RESET); // Troppo verboso, solo in caso di fallimento
			}
			else
			{
				Serial.println(ANSI_RED "FAIL (LOW Test): Expected LOW on GPIO" + String(inputPin) + ", but got HIGH." ANSI_RESET);
				pairFailed = true;
			}

			if (!pairFailed)
			{
				Serial.println(ANSI_GREEN "PASS." ANSI_RESET);
			}
			else
			{
				failedTests++;
			}

			// Disabilita i pin per la prossima iterazione, o mettili in uno stato neutro
			// pinMode(outputPin, INPUT); // Rimettere come input previene consumi o conflitti futuri
			// pinMode(inputPin, INPUT);
		}

		Serial.println(ANSI_BLUE "--- Ciclo di test completato ---" ANSI_RESET);
		if (failedTests == 0)
		{
			Serial.println(ANSI_GREEN "Tutti i test delle coppie sono PASSATI!" ANSI_RESET);
		}
		else
		{
			Serial.print(ANSI_RED "Numero di test FALLITI: ");
			Serial.println(failedTests);
			Serial.println("Verifica i collegamenti o i pin indicati come falliti." ANSI_RESET);
		}

		Serial.println(ANSI_YELLOW "Prossimo test tra " + String(testInterval / 1000) + " secondi." ANSI_RESET);
	}
}
#else
// NON TEST
#define SERIAL_QUEUE_LENGTH 32 // Length of the serial queue
#define SERIAL_MSG_SIZE 128 // Size of the serial message buffer

// Pin Configuration 

// Pin Definitions for 6502 Bus Monitor
// Address Bus A0-A15

#define PROTOTYPE 1 // Define PROTOTYPE (some lines are not available)

#if PROTOTYPE == 0

// A0-A15 are only on the final revision board

// LSB Address Pins

constexpr gpio_num_t PIN_A0 = GPIO_NUM_6;	// D6
constexpr gpio_num_t PIN_A1 = GPIO_NUM_8;	// D8
constexpr gpio_num_t PIN_A6 = GPIO_NUM_38;	// D38
constexpr gpio_num_t PIN_A7 = GPIO_NUM_37;	// D37

// Decoded selection signals are only on the final revision board
constexpr gpio_num_t PIN_EXSEL = GPIO_NUM_11;	// When low external memory is selected, when high internal memory is selected
constexpr gpio_num_t PIN_D1XX = GPIO_NUM_10;	// When accessing PBI I/O memory space
constexpr gpio_num_t PIN_CCTL = GPIO_NUM_20;	// When accessing Cartridge Control $D500 CCTL
constexpr gpio_num_t PIN_MPD = GPIO_NUM_7;		// Math Pack ROM Disable

#endif

// Common LSB Address Pins
constexpr gpio_num_t PIN_A2 = GPIO_NUM_21;	// D21
constexpr gpio_num_t PIN_A3 = GPIO_NUM_27;	// D27
constexpr gpio_num_t PIN_A4 = GPIO_NUM_33;	// D33
constexpr gpio_num_t PIN_A5 = GPIO_NUM_32;	// D32

// MSB Address Pins
constexpr gpio_num_t PIN_A8  = GPIO_NUM_2;	// D2
constexpr gpio_num_t PIN_A9  = GPIO_NUM_5;	// D5
constexpr gpio_num_t PIN_A10 = GPIO_NUM_12;	// D12
constexpr gpio_num_t PIN_A11 = GPIO_NUM_15;	// D15
constexpr gpio_num_t PIN_A12 = GPIO_NUM_34;	// D34
constexpr gpio_num_t PIN_A13 = GPIO_NUM_35;	// D35
constexpr gpio_num_t PIN_A14 = GPIO_NUM_36;	// D36 VP
constexpr gpio_num_t PIN_A15 = GPIO_NUM_39;	// D39 VN

// PHI2 and RW Pins
// PHI2 is used for clocking the 6502 bus
// RW is used to indicate read/write operations
constexpr gpio_num_t PIN_PHI2 = GPIO_NUM_23;	// D23
constexpr gpio_num_t PIN_RW   = GPIO_NUM_25;	// D25

// Data Bus D0-D7
// These pins are used for data transfer on the 6502 bus
constexpr gpio_num_t PIN_D0 = GPIO_NUM_4;	// D4
constexpr gpio_num_t PIN_D1 = GPIO_NUM_13;	// D13
constexpr gpio_num_t PIN_D2 = GPIO_NUM_14;	// D14
constexpr gpio_num_t PIN_D3 = GPIO_NUM_16;	// D16 RX2
constexpr gpio_num_t PIN_D4 = GPIO_NUM_17;	// D17 TX2
constexpr gpio_num_t PIN_D5 = GPIO_NUM_18;	// D18
constexpr gpio_num_t PIN_D6 = GPIO_NUM_19;	// D19
constexpr gpio_num_t PIN_D7 = GPIO_NUM_22;	// D22

constexpr gpio_num_t PIN_CS = GPIO_NUM_26;	// D26 Chip Select Pin for FPGA VERA VIDEO CARD

// Stringa e buffer
const char MEMORY_STR[] = "6502 MEMORY";
const size_t MEMORY_STR_LEN = sizeof(MEMORY_STR) - 1;

#define BUFFER_SIZE 32
uint8_t writeBuffer[BUFFER_SIZE] = {0};
size_t writeIndex = 0;
uint8_t strIndex = 0;

static uint8_t d500[256] = {0}; // CCTL memory space
static uint8_t cardselected = 0; // Flag to indicate if a card is selected
static uint8_t DEVICE_ID = 0x01; // Example device ID for PBI I/O
extern const uint8_t pbi_driver[]; // PBI driver memory space

#define DBG_ERROR   0
#define DBG_INFO    1
#define DBG_VERBOSE 2
#define DBG_NOISY   3

static int debuglevel = DBG_INFO;

QueueHandle_t serialQueue;

// This task will run on the second (1) core of the ESP32
void SerialTask(void *pvParameters)
{
	char msg[SERIAL_MSG_SIZE];
	while (true)
	{
		if (xQueueReceive(serialQueue, msg, portMAX_DELAY) == pdTRUE)
		{
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

	if (serialQueue != NULL)
	{
		xQueueSend(serialQueue, buf, portMAX_DELAY);
	}
	else
	{
		Serial.println("Serial queue not initialized!");
	}
}

// Funzioni ottimizzate per lettura GPIO
inline uint32_t read_gpio_low(void)
{
	return REG_READ(GPIO_IN1_REG);  // GPIO 0-31
}

inline uint32_t read_gpio_high(void)
{
	return REG_READ(GPIO_IN_REG);   // GPIO 32-39
}

inline bool read_gpio_level(gpio_num_t pin)
{
	if(pin < 32) {
		return (read_gpio_low() >> pin) & 1;
	} else {
		return (read_gpio_high() >> (pin - 32)) & 1;
	}
}

uint16_t read_address_bus(void)
{
#if PROTOTYPE == 1
	// Read as much Address Lines as possible
	return (read_gpio_level(PIN_A2)  << 2) |
		   (read_gpio_level(PIN_A3)  << 3) |
		   (read_gpio_level(PIN_A4)  << 4) |
		   (read_gpio_level(PIN_A5)  << 5) |
		   (read_gpio_level(PIN_A8)  << 8) |
		   (read_gpio_level(PIN_A9)  << 9) |
		   (read_gpio_level(PIN_A10) << 10) |
		   (read_gpio_level(PIN_A11) << 11) |
		   (read_gpio_level(PIN_A12) << 12) |
		   (read_gpio_level(PIN_A13) << 13) |
		   (read_gpio_level(PIN_A14) << 14) |
		   (read_gpio_level(PIN_A15) << 15);
#else
	// Read all 16 Address Lines at once
	return (read_gpio_level(PIN_A0)  << 0) |
		   (read_gpio_level(PIN_A1)  << 1) |
		   (read_gpio_level(PIN_A2)  << 2) |
		   (read_gpio_level(PIN_A3)  << 3) |
		   (read_gpio_level(PIN_A4)  << 4) |
		   (read_gpio_level(PIN_A5)  << 5) |
		   (read_gpio_level(PIN_A6)  << 6) |
		   (read_gpio_level(PIN_A7)  << 7) |
		   (read_gpio_level(PIN_A8)  << 8) |
		   (read_gpio_level(PIN_A9)  << 9) |
		   (read_gpio_level(PIN_A10) << 10) |
		   (read_gpio_level(PIN_A11) << 11) |
		   (read_gpio_level(PIN_A12) << 12) |
		   (read_gpio_level(PIN_A13) << 13) |
		   (read_gpio_level(PIN_A14) << 14) |
		   (read_gpio_level(PIN_A15) << 15);
#endif
}

uint8_t read_data_bus(void)
{
	// Read all Data Bus Lines at once
	return (read_gpio_level(PIN_D0) << 0) |
		   (read_gpio_level(PIN_D1) << 1) |
		   (read_gpio_level(PIN_D2) << 2) |
		   (read_gpio_level(PIN_D3) << 3) |
		   (read_gpio_level(PIN_D4) << 4) |
		   (read_gpio_level(PIN_D5) << 5) |
		   (read_gpio_level(PIN_D6) << 6) |
		   (read_gpio_level(PIN_D7) << 7);
}

void set_data_bus_output(void)
{
	for(int i = 0; i < 8; i++)
	{
		gpio_set_direction(static_cast<gpio_num_t>(PIN_D0 + i), GPIO_MODE_OUTPUT);
	}
}

void set_data_bus_input(void)
{
	for(int i = 0; i < 8; i++)
	{
		gpio_set_direction(static_cast<gpio_num_t>(PIN_D0 + i), GPIO_MODE_INPUT);
	}
}

void write_data_bus(uint8_t value)
{
	for(int i = 0; i < 8; i++)
	{
		gpio_set_level(static_cast<gpio_num_t>(PIN_D0 + i), (value >> i) & 1);
	}
}

void dump_buffer(void)
{
	Serial.println("\nBuffer Dump:");
	for(size_t i = 0; i < BUFFER_SIZE; i++)
	{
		Serial.printf("%02X%s", writeBuffer[i], (i % 16 == 15) ? "\n" : " ");
	}
}

void setup(void)
{
	Serial.begin(115200);

    serialQueue = xQueueCreate(SERIAL_QUEUE_LENGTH, SERIAL_MSG_SIZE);
    xTaskCreatePinnedToCore(SerialTask, "SerialTask", 2048, NULL, 1, NULL, 1);

	// Configurazione pin
	gpio_config_t io_conf = {};
	// ADDRESS BUS A0-A15

#if PROTOTYPE == 0
	// ADDRESS BUS LSB A0-A7
	io_conf.pin_bit_mask = (1ULL << PIN_A0) | (1ULL << PIN_A1) | (1ULL << PIN_A2) | 
						   (1ULL << PIN_A3) | (1ULL << PIN_A4) | (1ULL << PIN_A5) |
						   (1ULL << PIN_A6) | (1ULL << PIN_A7);
#endif
	// ADDRESS BUS MSB A8-A15
	io_conf.pin_bit_mask |= (1ULL << PIN_A8) | (1ULL << PIN_A9) | (1ULL << PIN_A10) | 
						   (1ULL << PIN_A11) | (1ULL << PIN_A12) | (1ULL << PIN_A13) |
						   (1ULL << PIN_A14) | (1ULL << PIN_A15);
	// PHI2 & RW
	io_conf.pin_bit_mask |= (1ULL << PIN_PHI2) | (1ULL << PIN_RW);
	io_conf.mode = GPIO_MODE_INPUT;
	gpio_config(&io_conf);

	// Configurazione pin dati come input
	io_conf.pin_bit_mask = (1ULL << PIN_D0) | (1ULL << PIN_D1) | (1ULL << PIN_D2) |
						   (1ULL << PIN_D3) | (1ULL << PIN_D4) | (1ULL << PIN_D5) |
						   (1ULL << PIN_D6) | (1ULL << PIN_D7);
	gpio_config(&io_conf);

#if PROTOTYPE == 0
	// Configurazione pin segnali aggiuntivi come output
	io_conf.pin_bit_mask = (1ULL << PIN_CCTL) | (1ULL << PIN_D1XX) |
						   (1ULL << PIN_EXSEL) | (1ULL << PIN_MPD);
	io_conf.mode = GPIO_MODE_OUTPUT;
	gpio_config(&io_conf);
#endif

	// Configurazione CS come output
	gpio_set_direction(PIN_CS, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_CS, 1);

	serialPrintQueue(ANSI_BLUE ">> 6502 Bus Monitor Ready\n" ANSI_RESET);
}

void loop(void)
{
	static bool last_phi2 = false;
	bool current_phi2 = read_gpio_level(PIN_PHI2);

	// Rising edge of PHI2
	if(current_phi2 && !last_phi2)
	{
		uint16_t address = read_address_bus();
		bool rw = read_gpio_level(PIN_RW);
		
		// CCTL
		if(address >= 0xD500 && address <= 0xD5FF)
		{
			#if PROTOTYPE == 0
				gpio_set_level(PIN_CCTL, 0); // Set CCTL low to indicate access to CCTL memory space
			#endif
			if (rw)
			{
				// 6502 CPU is LD[X/Y/A] from memory @ $D5xx
				uint8_t data = d500[address & 0x00FF];
				set_data_bus_output();
				write_data_bus(data);
				serialPrintQueue(ANSI_YELLOW "CCTL: Send %02X from %04X to CPU\n" ANSI_RESET, data, address);
			}
			else
			{
				// 6502 CPU is asking to write to memory @ $D5xx ST[X/Y/A]
				set_data_bus_input();
				uint8_t data = read_data_bus();
				d500[address & 0x00FF] = data;
				serialPrintQueue(ANSI_YELLOW "DBG_VERBOSE, CCTL: Received %02X to %04X from CPU\n" ANSI_RESET, data, address);
			}
		}
		else
		{
			#if PROTOTYPE == 0
				gpio_set_level(PIN_CCTL, 1); // Set CCTL high for normal operation
			#endif
		}

		// Gestione PBI I/O
		if(address >= 0xD100 && address <= 0xD1FF)
		{
			#if PROTOTYPE == 0
				gpio_set_level(PIN_D1XX, 0); // Set D1XX low to indicate access to PBI I/O memory space
			#endif 
			if (address == 0xD1FF)
			{
				// 6502 CPU writes to PBI I/O to select device
				if (!rw)
				{
					uint8_t device = read_data_bus();
					if (device == DEVICE_ID)
					{
						serialPrintQueue(ANSI_BLUE "PBI I/O: Device %02X selected\n" ANSI_RESET, device);
						cardselected = 1;
						gpio_set_level(PIN_CS, 0); // Set CS low for PBI I/O
					}
					else
					{
						serialPrintQueue(ANSI_RED "PBI I/O: Invalid device %02X selected\n" ANSI_RESET, device);
						cardselected = 0;
						gpio_set_level(PIN_CS, 1); // Set CS high for no device selected
					}
				}
				else
				{
					// CPU reads from PBI I/O
					uint8_t data = cardselected ? DEVICE_ID : 0; // Return DEVICE_ID if device is selected, otherwise 0
					set_data_bus_output();
					write_data_bus(data);
					serialPrintQueue(ANSI_GREEN "PBI I/O: Sent %02X to CPU\n" ANSI_RESET, data);
				}
			}
			else
			{
				if (cardselected)
				{
					if (address >= 0xD100 && address <= 0xD1F0)
					{
						// Sniffing I/O Space registers
						serialPrintQueue(ANSI_YELLOW "PBI Registers: Read or Write @ %04X address\n" ANSI_RESET, address);
					}
					else
					{
						// Someone is trying to access to $D1F1 up to $D1FE
						serialPrintQueue(ANSI_RED "PBI Unknown Registers: Read or Write @ %04X address\n" ANSI_RESET, address);
					}
				}
				else
				{
					// Someone is trying to access to $D1XX without selecting device first!
					serialPrintQueue(ANSI_RED "PBI Read or Write @ %04X address. Device must be selected first!\n" ANSI_RESET, address);
				}
			}
		}
		else
		{
			#if PROTOTYPE == 0
				gpio_set_level(PIN_D1XX, 1); // Set D1XX high for normal operation
			#endif
		}

		// Gestione EXSEL e MPD
		if(address >= 0xD800 && address <= 0xDFFF)
		{
			if (cardselected)
			{
				// ROM (READ ONLY)
				if (rw)
				{
					// D800-DFFF: CPU read from PBI Driver and lower MPD
					uint8_t data = pbi_driver[ address - 0xD800 ];
					#if PROTOTYPE == 0 
						gpio_set_level(PIN_MPD, 0); // Set MPD low to indicate Math Pack ROM Disable
						gpio_set_level(PIN_EXSEL, 0); // Set EXSEL low for external memory
					#endif
					serialPrintQueue(ANSI_YELLOW "EXSEL: Set to external memory, MPD: Disabled" ANSI_RESET);
					set_data_bus_output();
					write_data_bus(data);
					serialPrintQueue(ANSI_YELLOW "PBI ROM Driver: Sent %02X from %04X to CPU\n" ANSI_RESET, data, address);
				}
				else
				{
					// D800-DFFF: CPU try to writes to PBI Driver?
					serialPrintQueue(ANSI_RED "OSROM Driver: Why Write to ROM? %04X\n" ANSI_RESET, address);
				}
			}
			else
			{
				serialPrintQueue(ANSI_BLUE "OSROM: Accessing Math Pack %04X\n" ANSI_RESET, address);
			}
		}
		else
		{
			#if PROTOTYPE == 0
				gpio_set_level(PIN_MPD, 1); // Set MPD high for normal operation
				gpio_set_level(PIN_EXSEL, 1); // Set EXSEL high for internal memory
			#endif
		}
	}

	last_phi2 = current_phi2;
	delayMicroseconds(5);
}
#endif
