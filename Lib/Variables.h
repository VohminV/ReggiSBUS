// LORA
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SCK 18
#define LORA_NSS 5
#define LORA_DIO0 26
#define LORA_DIO1 25
#define LORA_RST 14
#define LORA_BUSY 26
#define LORA_TXEN -1
#define LORA_RXEN -1

// BIND
#define BIND_PHRASE "reggi.0"

const uint32_t bindingTimeout = 180000; // 3 minutes
unsigned long lastBindTransmitTime = 0;
bool bindingCompleted = false;
bool bindingRequested = true;
uint32_t lastPingTime = 0;
uint32_t bindStartTime = 0;
const uint32_t pingInterval = 1000;
unsigned long timeout = 0;

// EEPROM Addresses
#define EEPROM_FREQ_ADDR 0
#define EEPROM_POWER_ADDR 4

// Default radio values
float frequency = 450.0; // Default frequency in MHz
int power = 10;          // Default power in dBm

// Web Server
bool webServerStarted = false;

// FRAME
#define FRAME_LEN 70             
#define SBUS_FRAME_LEN 25       
#define SBUS_CHANNEL_COUNT 16   

uint8_t sbusFrame[SBUS_FRAME_LEN] = {0}; 

typedef struct sbus_data_s {
    uint16_t channels[SBUS_CHANNEL_COUNT]; 
    bool failsafe;                        
    bool frameLost;                       
    char bind_elements[3];                 
} sbus_data_t;


//DEBUG
unsigned long packetCount = 0;       
unsigned long lastTime = 0;        
unsigned int packetsPerMinute = 0; 