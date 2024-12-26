#include <Arduino.h>
#include <RadioLib.h>
#include <EEPROM.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <../../lib/Variables.h>

// SPI setup
SPIClass spi(VSPI);
// Radio setup
SX1278 radio = new Module(LORA_NSS, LORA_DIO0, LORA_RST, LORA_DIO1, spi, SPISettings(8000000, MSBFIRST, SPI_MODE0));
// CRSF Serial setup
HardwareSerial SBUSSerial(1);

#define SBUS_PIN 13

AsyncWebServer server(80);
// Captive Portal
DNSServer dnsServer;

volatile bool operationDone = false;

void ICACHE_RAM_ATTR setFlag(void)
{
  // we sent or received  packet, set the flag
  operationDone = true;
}

// Radio initialization
void ICACHE_RAM_ATTR initRadio()
{

  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("Radio initialized successfully!"));
  }
  else
  {
    Serial.print(F("Radio initialization failed, error: "));
    Serial.println(state);
    while (true)
      delay(10);
  }

  if (radio.setFrequency(frequency) != RADIOLIB_ERR_NONE)
  {
    Serial.println(F("Failed to set frequency"));
    while (true)
      delay(10);
  }

  if (radio.setBandwidth(125.0) != RADIOLIB_ERR_NONE ||
      radio.setSpreadingFactor(6) != RADIOLIB_ERR_NONE ||
      radio.setCodingRate(5) != RADIOLIB_ERR_NONE ||
      radio.setOutputPower(power) != RADIOLIB_ERR_NONE)
  {
    Serial.println(F("Radio configuration failed"));
    while (true)
      delay(10);
  }

  radio.setDio0Action(setFlag, RISING);
}

void ICACHE_RAM_ATTR setupWebServer()
{

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->redirect("/config"); });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    String html = "<!DOCTYPE html><html><head><style>";
    html += "body { font-family: Arial, sans-serif; text-align: center; background-color: #f4f4f4; }";
    html += "form { display: inline-block; margin-top: 50px; }";
    html += "input[type='text'] { font-size: 18px; padding: 10px; margin: 10px; width: 200px; }";
    html += "input[type='submit'] { font-size: 18px; padding: 10px 20px; background-color: #007BFF; color: white; border: none; cursor: pointer; }";
    html += "input[type='submit']:hover { background-color: #0056b3; }";
    html += "</style></head><body>";
    html += "<h1>Reggi TX Settings</h1>";
    html += "<form action='/set' method='GET'>";
    html += "Frequency (MHz):<br><input type='text' name='freq' value='" + String(frequency) + "'><br>";
    html += "Power (dBm):<br><input type='text' name='power' value='" + String(power) + "'><br>";
    html += "<input type='submit' value='Set'>";
    html += "</form>";
    html += "</body></html>";
    request->send(200, "text/html", html); });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    if (request->hasParam("freq")) {
      frequency = request->getParam("freq")->value().toFloat();
      EEPROM.put(EEPROM_FREQ_ADDR, frequency);
    }
    if (request->hasParam("power")) {
      power = request->getParam("power")->value().toInt();
      EEPROM.put(EEPROM_POWER_ADDR, power);
    }
    EEPROM.commit();
    request->redirect("/config");
    delay(1000);
    ESP.restart(); });
  server.begin();
  webServerStarted = true;
}

static ICACHE_RAM_ATTR String toStringIp(IPAddress ip)
{
  String res = "";
  for (int i = 0; i < 3; i++)
  {
    res += String((ip >> (8 * i)) & 0xFF) + ".";
  }
  res += String(((ip >> 8 * 3)) & 0xFF);
  return res;
}

void bind_do_receive()
{
  uint8_t receivedData[FRAME_LEN];
  int state = radio.receive(receivedData, sizeof(receivedData));

  if (state == RADIOLIB_ERR_NONE)
  {
    if (receivedData[0] == BIND_PHRASE[0] && receivedData[1] == 0x01)
    {
      Serial.println("Binding data received successfully!");
      bindingCompleted = true;
    }
  }
}

unsigned long lastReceiveTime = 0;
const unsigned long receiveInterval = 3000;

void ICACHE_RAM_ATTR bind_do_transmit()
{
  uint8_t bindData[FRAME_LEN];
  memset(bindData, 0, FRAME_LEN);

  memcpy(bindData, BIND_PHRASE, strlen(BIND_PHRASE));

  if (millis() - lastBindTransmitTime > 3000)
  {
    int state = radio.transmit(bindData, FRAME_LEN);
    if (state == RADIOLIB_ERR_NONE)
    {
      Serial.println("Binding data transmitted successfully!");
      lastBindTransmitTime = millis();
    }
    else
    {
      Serial.print("Binding transmission failed, error: ");
      Serial.println(state);
    }
  }

  if (millis() - lastReceiveTime > receiveInterval)
  {
    bind_do_receive();
    lastReceiveTime = millis();
  }
}

void ICACHE_RAM_ATTR leftShift(uint8_t arr[], size_t size)
{
  memmove(arr, arr + 1, (size - 1));
  arr[size - 1] = 0xFF;
}

void setup()
{
  Serial.begin(115200);
  EEPROM.begin(512);
  spi.begin();

  SBUSSerial.begin(100000, SERIAL_8E2, SBUS_PIN, SBUS_PIN, true);
  EEPROM.get(EEPROM_FREQ_ADDR, frequency);
  EEPROM.get(EEPROM_POWER_ADDR, power);

  if (frequency < 100.0 || frequency > 1000.0)
    frequency = 450.0;
  if (power < 2 || power > 20)
    power = 10;

  WiFi.softAP("Reggi TX", "12345678");
  IPAddress apIP(10, 0, 0, 1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  dnsServer.start(53, "*", apIP);

  initRadio();
  bindStartTime = millis();
  Serial.println("Setup complete.");
}

void loop()
{
  dnsServer.processNextRequest();

  if (!bindingRequested && !bindingCompleted)
  {
    Serial.println("Starting binding process...");
    bindingRequested = true;
    bindStartTime = millis();
  }

  if (bindingRequested && !bindingCompleted)
  {
    Serial.println("Processing binding...");
    bind_do_transmit();
    if (millis() - bindStartTime > bindingTimeout)
    {
      Serial.println("Binding timeout. Binding process failed.");
      bindingRequested = false;
      bindingCompleted = false;
      if (!webServerStarted)
      {
        setupWebServer();
        webServerStarted = true;
      }
    }
  }

  if (bindingCompleted)
  {
    sbus_data_t txData;
    uint8_t size = SBUS_FRAME_LEN;

    // Проверяем, есть ли доступные данные в порту
    while (SBUSSerial.available() >= SBUS_FRAME_LEN)
    {
      // Чтение одного полного S.Bus фрейма
      SBUSSerial.readBytes(sbusFrame, SBUS_FRAME_LEN);

      // Проверка валидности фрейма
      if (sbusFrame[0] == 0xF0 && sbusFrame[24] == 0x00)
      {
        // Распаковка каналов
        for (int i = 0; i < SBUS_CHANNEL_COUNT; i++)
        {
          int bitIndex = i * 11;
          int byteIndex = bitIndex / 8;
          int bitOffset = bitIndex % 8;

          txData.channels[i] =
              ((sbusFrame[1 + byteIndex] | (sbusFrame[2 + byteIndex] << 8)) >> bitOffset) & 0x07FF;
        }

        // Чтение флагов
        txData.failsafe = sbusFrame[23] & 0x08;
        txData.frameLost = sbusFrame[23] & 0x04;

        // Обработка данных (например, передача через другой интерфейс)
        txData.bind_elements[0] = BIND_PHRASE[0];
        txData.bind_elements[1] = BIND_PHRASE[3];
        txData.bind_elements[2] = BIND_PHRASE[6];

        // Пример вывода данных каналов
        for (int i = 0; i < SBUS_CHANNEL_COUNT; i++)
        {
          Serial.print("Channel ");
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.println(txData.channels[i]);
        }

        if (txData.failsafe)
        {
          Serial.println("Failsafe activated!");
        }
        if (txData.frameLost)
        {
          Serial.println("Frame lost!");
        }
      }
      else
      {
        Serial.println("Invalid S.Bus frame!");
      }
    }

    // Имитация задержки, если требуется передача данных обратно
    delay(14); // 14 мс для работы S.Bus
  }
}
