#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <TFT_eSPI.h>
#include <freertos/FreeRTOS.h>
#include "UbidotsConnection.h"

#define NUM_READINGS 100
#define AVERAGE_READINGS 10

/* Ubidots Config */
#define UBIDOTS_TOKEN " "
#define UBIDOTS_DEVICE "esp32_device"
#define UBIDOTS_URL "industrial.api.ubidots.com"
#define UBIDOTS_PORT "80"

#define EXAMPLE_ESP_WIFI_SSID "UPBWiFi"
#define EXAMPLE_ESP_WIFI_PASS ""

// P21 - SDA
// P22 - SCL

const int buttonPin = 35; // gpio integrado
const int ledPin = 27;    // Pin del LED

Adafruit_SHT31 sht31 = Adafruit_SHT31();
TFT_eSPI tft = TFT_eSPI();

float temperatureReadings[NUM_READINGS];
float humidityReadings[NUM_READINGS];
float globalTemperatureavg = 0.0;
float globalHumidityavg = 0.0;
int readingIndex = 0;

SemaphoreHandle_t xMutex;
bool displayUpdating = true;
bool ledState = false;             // Estado del LED
TaskHandle_t ledTaskHandle = NULL; // Manejador para la tarea del LED

int blinkPeriod = 2000; // Periodo de parpadeo incial en milisegundos

UbidotsConnection ubidots("API_KEY"); // coloco mi api de ubidots

// Prototipos de las tareas
void readSensorTask(void *pvParameters);
void updateDisplayTask(void *pvParameters);
void commandProcessingTask(void *pvParameters);
void ledBlinkTask(void *pvParameters);
void sendUbidotsDataTask(void *pvParameters);

// Inicializar tareas
void initializeTasks()
{
  xTaskCreate(readSensorTask, "ReadSensorTask", 2048, NULL, 1, NULL);
  xTaskCreate(updateDisplayTask, "UpdateDisplayTask", 2048, NULL, 2, NULL);
  xTaskCreate(commandProcessingTask, "CommandProcessingTask", 2048, NULL, 3, NULL);
  xTaskCreate(sendUbidotsDataTask, "SendUbidotsDataTask", 2048, NULL, 3, NULL);
}

void readSensorTask(void *pvParameters)
{
  while (true)
  {
    float temperature = sht31.readTemperature();
    float humidity = sht31.readHumidity();

    if (isnan(temperature) || isnan(humidity))
    {
      Serial.println("Error leyendo del sensor.");
    }
    else
    {
      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
      {
        temperatureReadings[readingIndex] = temperature;
        humidityReadings[readingIndex] = humidity;

        readingIndex = (readingIndex + 1) % NUM_READINGS;

        Serial.print("Temp: ");
        Serial.print(temperature);
        Serial.print(" oC, Hum: ");
        Serial.print(humidity);
        Serial.println(" %");

        xSemaphoreGive(xMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void updateDisplayTask(void *pvParameters)
{
  while (true)
  {
    if (displayUpdating)
    {
      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
      {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextSize(2);
        tft.setCursor(10, 10);
        tft.print("Sensor SHT30");
        tft.setCursor(0, 35);
        tft.print("Temp: ");
        tft.print(temperatureReadings[readingIndex == 0 ? NUM_READINGS - 1 : readingIndex - 1]);
        tft.print(" oC");
        tft.setCursor(0, 65);
        tft.print("Hum: ");
        tft.print(humidityReadings[readingIndex == 0 ? NUM_READINGS - 1 : readingIndex - 1]);
        tft.print(" %");
        xSemaphoreGive(xMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void commandProcessingTask(void *pvParameters)
{
  while (true)
  {
    if (Serial.available() > 0)
    {
      String command = Serial.readStringUntil('\n');

      if (command.equals("avg"))
      {
        float avgTemperature = 0.0;
        float avgHumidity = 0.0;
        int count = 0;

        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
        {
          for (int i = 0; i < AVERAGE_READINGS; i++)
          {
            int index = (readingIndex - 1 - i + NUM_READINGS) % NUM_READINGS;
            avgTemperature += temperatureReadings[index];
            avgHumidity += humidityReadings[index];
            globalTemperatureavg = avgTemperature;
            globalHumidityavg = avgHumidity;
            count++;
          }
          avgTemperature /= count;
          avgHumidity /= count;
          xSemaphoreGive(xMutex);
        }

        displayUpdating = false;

        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
        {
          tft.fillScreen(TFT_BLACK);
          tft.setTextColor(TFT_WHITE, TFT_BLACK);
          tft.setTextSize(2);
          tft.setCursor(10, 10);
          tft.print("Comando: avg");
          tft.setCursor(10, 35);
          tft.print("Prom Temp: ");
          tft.print(avgTemperature);
          tft.print(" oC");
          tft.setCursor(10, 65);
          tft.print("Prom Hum: ");
          tft.print(avgHumidity);
          tft.print(" %");
          xSemaphoreGive(xMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));

        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
        {
          tft.fillScreen(TFT_BLACK);
          xSemaphoreGive(xMutex);
        }

        displayUpdating = true;
      }
      else if (command.startsWith("pro"))
      {
        // Cambiar el periodo de parpadeo
        char value = command.charAt(3); // Obtener el caracter despues de "pro"
        int period = value - '0';       // Convertir a numero

        if (period >= 1 && period <= 9) // Que no pase de 9 o se enloquece
        {
          blinkPeriod = period * 1000; // Convertir a milisegundos

          displayUpdating = false;

          if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
          {
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setTextSize(2);
            tft.setCursor(10, 10);
            tft.print("Comando: pro");
            tft.setCursor(10, 35);
            tft.print("Nuevo periodo:");
            tft.setCursor(10, 65);
            tft.print(period);
            tft.print(" segundos");
            xSemaphoreGive(xMutex);
          }

          vTaskDelay(pdMS_TO_TICKS(2000));

          if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
          {
            tft.fillScreen(TFT_BLACK); // Limpiar la pantalla despues de mostrar el resultado
            xSemaphoreGive(xMutex);
          }

          displayUpdating = true;
        }
        else
        {
          Serial.println("Periodo no valido");
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void ledBlinkTask(void *pvParameters)
{
  while (true)
  {
    if (ledState)
    {
      digitalWrite(ledPin, HIGH);
      vTaskDelay(pdMS_TO_TICKS(blinkPeriod / 2));
      digitalWrite(ledPin, LOW);
      vTaskDelay(pdMS_TO_TICKS(blinkPeriod / 2));
    }
    else
    {
      digitalWrite(ledPin, LOW); // Asegurarse de que el LED este apagado
      vTaskDelete(NULL);         // Terminar la tarea si el LED no debe parpadear
    }
  }
}
void sendUbidotsDataTask(void *pvParameters)
{
  while (true)
  {

    if (ubidots.sendData(UBIDOTS_DEVICE, "temperature", globalTemperatureavg))
    {
      Serial.println("Temperature data sent successfully");
    }
    else
    {
      Serial.println("Failed to send temperature data");
    }

    if (ubidots.sendData(UBIDOTS_DEVICE, "humidity", globalHumidityavg))
    {
      Serial.println("Humidity data sent successfully");
    }
    else
    {
      Serial.println("Failed to send humidity data");
    }

    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

// Pulsador
void IRAM_ATTR handleButtonPress()
{
  ledState = !ledState;

  if (ledState && ledTaskHandle == NULL)
  {
    xTaskCreate(ledBlinkTask, "LedBlinkTask", 2048, NULL, 1, &ledTaskHandle);
  }
  else if (!ledState && ledTaskHandle != NULL)
  {
    vTaskDelete(ledTaskHandle);
    ledTaskHandle = NULL;      // Reiniciar el manejador de la tarea
    digitalWrite(ledPin, LOW); // Asegurarse de que el LED este apagado
  }
}

void setup()
{
  Serial.begin(115200);
  WiFi.begin(EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Conectando a Wifi");
  }
  Serial.println("Conectado a wifi");
  if (!sht31.begin(0x44))
  {
    Serial.println("No se pudo encontrar un sensor SHT30.");
    while (1)
      delay(10);
  }

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL)
  {
    Serial.println("No se pudo crear el mutex.");
    while (1)
      ;
  }

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, FALLING);

  initializeTasks();
}

void loop()
{
  // nada, por si acaso
}
