#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <Firebase_ESP_Client.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP23X17.h>
#include <Wire.h>
#include <stdint.h> 
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// WiFi & Firebase
#define WIFI_SSID "Tran Tien 2G"
#define WIFI_PASSWORD "12345678"
#define API_KEY "AIzaSyBzh0tixfgNftb1HnK57EAbTByYdlB3GaA"
#define DATABASE_URL "https://iot-system-demo-default-rtdb.asia-southeast1.firebasedatabase.app"

// Firebase
FirebaseData firebaseData;
FirebaseData fbdo; // Declare fbdo as a FirebaseData object
FirebaseConfig config;
FirebaseAuth auth;
bool signupOK = false;
bool firebaseConnected = false; // Declare firebaseConnected as a global variable


// Update ADS1 channel mapping
// ADS1115 ADS1 (0x48)
#define CDS_CHANNEL         0    // ADS1 A0 for CDS (Light sensor)
#define PH_CHANNEL          1    // ADS1 A1 for pH sensor
#define TURBIDITY_CHANNEL   2    // ADS1 A2 for turbidity sensor
// #define DHT11_CHANNEL       3    // ADS1 A3 for DHT11 (Temperature and Humidity sensor)

// ADS1115 ADS2 (0x49)
#define DO_CHANNEL          0    // ADS2 A0 for DO sensor
#define TDS_CHANNEL         1    // ADS2 A1 for TDS sensor
#define NH3_CHANNEL         2    // ADS2 A2 for NH3 sensor
#define RAIN_CHANNEL        3    // ADS2 A3 for Rain sensor

// ADS1115 ADS3 (0x4B)
#define H2S_CHANNEL         0    // ADS3 A0 for H2S sensor
#define NO2_CHANNEL         1    // ADS3 A1 for NO2 sensor
#define CH4_CHANNEL         2    // ADS3 A2 for CH4 sensor
#define DS18B20_CHANNEL     3    // ADS3 A3 for DS18B20 (Temperature sensor)
// ======== CONSTANTS ========
#define Offset 0.00            //deviation compensate
#define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;
// Define sensitivity values (you'll need to get these from your sensor datasheets)
#define NH3_SENSITIVITY 10.0  // Example: mV per ppm
#define H2S_SENSITIVITY 20.0  // Example: mV per ppm
#define NO2_SENSITIVITY 15.0  // Example: mV per ppm
#define CH4_SENSITIVITY 5.0   // Example: mV per ppm

// Define zero gas voltage (calibration point)
#define NH3_ZERO_VOLTAGE 0.4  // Example: voltage when NH3 = 0 ppm
#define H2S_ZERO_VOLTAGE 0.2  // Example: voltage when H2S = 0 ppm
#define NO2_ZERO_VOLTAGE 0.3  // Example: voltage when NO2 = 0 ppm
#define CH4_ZERO_VOLTAGE 0.5  // Example: voltage when CH4 = 0 ppm
#define DO_ZERO_VOLTAGE 0.6   // Example: voltage when DO = 0 ppm
#define DO_SENSITIVITY 40.0   // Example: mV per ppm (adjust based on your sensor's datasheet)
#define DS18B20_ZERO_VOLTAGE 0.0 // Example: voltage when DS18B20 = 0 °C
#define DS18B20_SENSITIVITY 10.0 // Example: mV per °C (adjust based on your sensor's datasheet)



// ========== ADS1115 Setup ==========
Adafruit_ADS1115 ads1; // 0x48
Adafruit_ADS1115 ads2; // 0x49
Adafruit_ADS1115 ads3; // 0x4B

// ========== MCP23017 (Relay Control) ==========
Adafruit_MCP23X17 mcp;
bool mcpInitialized = false; // Declare mcpInitialized as a global variable

// ========== TFT Display ==========
#define TFT_CS     5
#define TFT_DC     17
#define TFT_RST    16
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
#define ILI9341_BROWN 0x8200 // Custom color for TDS

// ========== Software SPI (Touch) ==========
#define TOUCH_CS    25
#define TOUCH_IRQ   33
#define TOUCH_CLK   26
#define TOUCH_MISO  35
#define TOUCH_MOSI  32

// ========== Button Control ==========
#define BTN_WIDTH   55
#define BTN_HEIGHT  50
#define BTN_COUNT   8
int btnX[BTN_COUNT] = {10, 70, 130, 190, 10, 70, 130, 190};
int btnY[BTN_COUNT] = {100, 100, 100, 100, 170, 170, 170, 170};
bool btnState[BTN_COUNT] = {false};
const char* btnLabels[BTN_COUNT] = {"A", "B", "C", "D", "E", "F", "G", "H"};
unsigned long lastControlTabExitTime = 0; // Stores time when CONTROL_TAB is exited
bool wasInControlTab = false;             // Tracks whether previously in CONTROL_TAB


#define SCOUNT           30    // TDS sample count

// TDS variables
int    tdsBuffer[SCOUNT];
int    tdsIndex = 0;
float  tdsValue = 0, temperature = 25.0f;
// =================== Biến toàn cục ===================
volatile float nh3Voltage ;
volatile float h2sVoltage ;
volatile float no2Voltage ;
volatile float ch4Voltage ;
volatile float phVoltage ;
volatile float phValue ;
// Removed duplicate declaration of tdsValue
volatile int turbidityNTU = 0;
// Global variables to store PPM values
float nh3PPM, h2sPPM, no2PPM, ch4PPM, doValue, rainValue, ds18b20Value;


// =================== Giá trị hiệu chỉnh (offset) ===================
#define NH3_OFFSET 0.05
#define H2S_OFFSET -0.03
#define NO2_OFFSET 0.02
#define CH4_OFFSET 0.01
#define PH_OFFSET 0.1
#define TDS_OFFSET -5.0
#define TURBIDITY_OFFSET 2
#define DO_OFFSET 0.05 // Example offset value for DO sensor
#define DS18B20_OFFSET 0.0 // Example offset value for DS18B20 sensor
#define DS18B20_SENSITIVITY 10.0 // Example sensitivity value for DS18B20 sensor (adjust as needed)
#define RAIN_OFFSET 0.0 // Example offset value for Rain sensor (adjust as needed)
#define RAIN_SENSITIVITY 1.0 // Example sensitivity value for Rain sensor (adjust as needed)
#define RAIN_ZERO_VOLTAGE 0.0 // Example zero voltage for Rain sensor (adjust as needed)

static unsigned long samplingTimeTurbidity = millis();
#define SAMPLING_INTERVAL   800 // ms

// Interrupt flags
volatile bool updateDisplayFlag = false; // Declare updateDisplayFlag
volatile bool readGasSensorsFlag = false; // Declare readGasSensorsFlag
volatile bool readWaterSensorsFlag = false; // Declare readWaterSensorsFlag

// Ensure flags are declared before their first usage

// Interrupt-safe function to set flags
void IRAM_ATTR setFlag(volatile bool* flag) {
  *flag = true;
}

TaskHandle_t TaskGasHandle;
TaskHandle_t TaskWaterHandle;
TaskHandle_t TaskDisplayHandle;
TaskHandle_t TaskFirebaseHandle; // Declare TaskFirebaseHandle
// ========== Tabs ==========
enum Tab { MONITOR_TAB, CONTROL_TAB, SETTINGS_TAB };
Tab currentTab = MONITOR_TAB;

// ========== Function Declarations ==========
void drawButton(int index, bool pressed);
void drawTabs();
void drawMonitorTab();
void drawControlTab();
void drawSettingsTab();
void changeTab(Tab tab);
void touchTask(void* parameter);
void pum_test();
void initWiFi(); // Forward declaration for initWiFi
void initFirebase(); // Forward declaration for initFirebase
void Control_inDisplayControlTab();
void readPHSensor();
float calculateMedian(int* arr, int size); // Declaration for calculateMedian
void readCH4(); // Declaration for readCH4
void readH2S(); // Ensure readH2S is also declared
void readNH3(); // Ensure readNH3 is also declared
void readNO2(); // Ensure readNO2 is also declared
void readDo(); // Ensure readDo is also declared
void readTDS(); // Ensure readTDS is also declared
void readTurbidity(); // Ensure readTurbidity is also declared
void readTemperature(); // Ensure readTemperature is also declared
void readRain(); // Ensure readRain is also declared
void readDS18B20(); // Ensure readDS18B20 is also declared
void Task_Display(void *pvParameters); // Forward declaration for Task_Display
void Task_Read_Gas(void *pvParameters); // Forward declaration for Task_Read_Gas
void Task_Read_Water(void *pvParameters);
void sendSensorDataToFirebase(); // Forward declaration for sendSensorDataToFirebase
void Task_FirebaseRelayControl(void *pvParameters); // Forward declaration for Task_FirebaseRelayControl
void FirebaseSendTask(void *pvParameters); // Forward declaration for FirebaseSendTask
void updateRelayState(uint8_t relayPin, const String& state);
void IRAM_ATTR setFlag(volatile bool* flag); // Forward declaration for setFlag
void setupInterrupts(); // Forward declaration for setupInterrupts
void processButtonPress(); // Forward declaration for processButtonPress




// ========== Software SPI ==========
void softSPI_begin() {
  pinMode(TOUCH_CS, OUTPUT);
  pinMode(TOUCH_CLK, OUTPUT);
  pinMode(TOUCH_MOSI, OUTPUT);
  pinMode(TOUCH_MISO, INPUT);
  pinMode(TOUCH_IRQ, INPUT);
  digitalWrite(TOUCH_CS, HIGH);
  digitalWrite(TOUCH_CLK, HIGH);
}

uint8_t softSPI_transfer(uint8_t dataOut) {
  uint8_t dataIn = 0;
  for (int i = 7; i >= 0; i--) {
    digitalWrite(TOUCH_CLK, LOW);
    digitalWrite(TOUCH_MOSI, (dataOut >> i) & 1);
    delayMicroseconds(1);
    digitalWrite(TOUCH_CLK, HIGH);
    dataIn <<= 1;
    if (digitalRead(TOUCH_MISO)) dataIn |= 0x01;
    delayMicroseconds(1);
  }
  return dataIn;
}

uint16_t readTouchAxis(uint8_t command) {
  digitalWrite(TOUCH_CS, LOW);
  softSPI_transfer(command);
  uint16_t high = softSPI_transfer(0x00);
  uint16_t low = softSPI_transfer(0x00);
  digitalWrite(TOUCH_CS, HIGH);
  return ((high << 8) | low) >> 3;
}

void drawButton(int index, bool pressed) {
  uint16_t color = pressed ? ILI9341_RED : ILI9341_BLUE;
  tft.fillRect(btnX[index], btnY[index], BTN_WIDTH, BTN_HEIGHT, color);
  tft.drawRect(btnX[index], btnY[index], BTN_WIDTH, BTN_HEIGHT, ILI9341_BLACK);

  // Draw button label
  tft.setCursor(btnX[index] + 15, btnY[index] + 5);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print(btnLabels[index]);

  // Draw button state ("ON" or "OFF")
  tft.setCursor(btnX[index] + 10, btnY[index] + 30);
  tft.setTextSize(1);
  tft.print(pressed ? "ON" : "OFF");
}

int currentPage = 0; // Tracks the current page (0: GAS, 1: WATER)

void drawMonitorTab() {
  tft.fillScreen(ILI9341_BLACK);  
  drawTabs();

  const int screenWidth = 240;
  const int screenHeight = 260; // Adjusted height
  const int leftMargin = 10; // Adjusted for alignment
  const int sectionWidth = screenWidth - 2 * leftMargin;
  const int rowHeight = 30; // Adjusted for better spacing
  const int labelX = leftMargin + 5;
  const int valueX = leftMargin + 60; // Adjusted for better spacing
  const int iconX = leftMargin + 190; // Space for icons
  int yPos = 60; // Start position adjusted for better spacing

  if (currentPage == 0) {
    // GAS section header
    tft.setTextSize(2);
    tft.fillRect(leftMargin, yPos, sectionWidth, rowHeight, ILI9341_NAVY);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(labelX + 30, yPos + 5);
    tft.print("GAS DATA");

    // GAS data rows
    yPos += rowHeight + 5;
    auto drawRow = [&](const char* label, float value, const char* unit, uint16_t iconColor) {
      tft.fillRect(leftMargin, yPos, sectionWidth, rowHeight - 2, 0x4416);
      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(labelX, yPos + 5);
      tft.print(label);
      tft.setTextColor(ILI9341_YELLOW);
      tft.setCursor(valueX, yPos + 5);
      tft.print(value, 2);
      tft.print(unit);

      // Draw icon
      tft.fillCircle(iconX + 10, yPos + rowHeight / 2 - 1, 8, iconColor); // Adjusted icon position

      yPos += rowHeight + 5;
    };

    drawRow("NH3:", nh3PPM, " PPM", ILI9341_RED);
    drawRow("H2S:", h2sPPM, " PPM", ILI9341_GREEN);
    drawRow("NO2:", no2PPM, " PPM", ILI9341_BLUE);
    drawRow("CH4:", ch4PPM, " PPM", ILI9341_ORANGE);
  } else if (currentPage == 1) {
    // WATER section header
    tft.setTextSize(2);
    tft.fillRect(leftMargin, yPos, sectionWidth, rowHeight, ILI9341_DARKGREEN);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(labelX + 20, yPos + 5);
    tft.print("WATER DATA");

    // WATER data rows
    yPos += rowHeight + 5;
    auto drawWaterRow = [&](const char* label, float value, const char* unit, uint16_t iconColor, bool isPH = false) {
      tft.fillRect(leftMargin, yPos, sectionWidth, rowHeight - 2, 0x0392);
      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(labelX, yPos + 5);
      tft.print(label);

      if (isPH) {
        if (value < 6.0 || value > 8.5)
          tft.setTextColor(ILI9341_RED);
        else
          tft.setTextColor(ILI9341_GREEN);
      } else {
        tft.setTextColor(ILI9341_YELLOW);
      }

      tft.setCursor(valueX, yPos + 3);
      tft.print(value, (strcmp(unit, " ppm") == 0) ? 0 : 1);
      tft.print(unit);

      // Draw icon
      tft.fillCircle(iconX + 10, yPos + rowHeight / 2 - 1, 8, iconColor); // Adjusted icon position

      yPos += rowHeight + 10; // Increased spacing between rows
    };

    drawWaterRow("pH:", phValue, "", ILI9341_RED, true);
    drawWaterRow("TDS:", tdsValue, " ppm", ILI9341_GREEN);
    drawWaterRow("Turb:", turbidityNTU, " NTU", ILI9341_BLUE);
    drawWaterRow("Temp:", temperature, " C", ILI9341_ORANGE);
    drawWaterRow("DO:", doValue, " ml/L", ILI9341_CYAN);
    drawWaterRow("Rain:", rainValue, " mm", ILI9341_PURPLE);
    drawWaterRow("DS18B20:", ds18b20Value, " C", ILI9341_YELLOW);
  }
}

void drawControlTab() {
  tft.fillScreen(ILI9341_WHITE);
  drawTabs();
  for (int i = 0; i < BTN_COUNT; i++) {
    drawButton(i, btnState[i]);
  }
}

void drawSettingsTab() {
  tft.fillScreen(ILI9341_WHITE);
  drawTabs();
  tft.setCursor(20, 80);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println("Settings Tab");
}

void drawTabs() {
  int tabWidth = 80, tabHeight = 60;
  for (int i = 0; i < 3; i++) {
    uint16_t color = (currentTab == i) ? ILI9341_GREEN : ILI9341_LIGHTGREY;
    tft.fillRect(i * tabWidth, 0, tabWidth, tabHeight, color);
    tft.drawRect(i * tabWidth, 0, tabWidth, tabHeight, ILI9341_BLACK);
    tft.setCursor(i * tabWidth + 15, 20);
    tft.setTextColor(ILI9341_BLACK);
    tft.setTextSize(1);
    if (i == 0) tft.print("Monitor");
    else if (i == 1) tft.print("Control");
    else if (i == 2) tft.print("Settings");
  }
}

void changeTab(Tab tab) {
  currentTab = tab;
  
  // Always draw the appropriate tab
  if (tab == MONITOR_TAB) {
    drawMonitorTab();  // This will display the most current sensor data
  }
  else if (tab == CONTROL_TAB) {
    drawControlTab();
  }
  else if (tab == SETTINGS_TAB) {
    drawSettingsTab();
  }

  // Detect when exiting CONTROL_TAB
  if (wasInControlTab && currentTab != CONTROL_TAB) {
    lastControlTabExitTime = millis();
    wasInControlTab = false;
  }

  // Update state if inside CONTROL_TAB
  if (currentTab == CONTROL_TAB) {
    wasInControlTab = true;
  }
}

// ========== Interrupt Handling ==========
volatile bool buttonPressed = false;

void IRAM_ATTR handleButtonInterrupt() {
  buttonPressed = true;
}

void setupInterrupts() {
  for (int i = 0; i < BTN_COUNT; i++) {
    pinMode(btnX[i], INPUT_PULLUP); // Assuming buttons are connected to btnX pins
    attachInterrupt(digitalPinToInterrupt(btnX[i]), handleButtonInterrupt, FALLING);
  }
}

void processButtonPress() {
  if (buttonPressed) {
    buttonPressed = false;
    for (int i = 0; i < BTN_COUNT; i++) {
      if (digitalRead(btnX[i]) == LOW) {
        btnState[i] = !btnState[i];
        drawButton(i, btnState[i]);
        updateRelayState(i, btnState[i] ? "ON" : "OFF");
      }
    }
  }
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  
  // Initialize hardware first
  softSPI_begin();
  Wire.begin(21, 22);
  
  // Initialize TFT display
  tft.begin();
  tft.setRotation(1);
  
  // Initialize ADS1115 modules with proper error checking
  if (!ads1.begin(0x48)) {
    // Failed to initialize ADS1115 at address 0x48
  } else {
    ads1.setGain(GAIN_ONE); // Adjust gain as needed
  }
  
  if (!ads2.begin(0x49)) {
    // Failed to initialize ADS1115 at address 0x49
  } else {
    ads2.setGain(GAIN_ONE);
  }
  
  if (!ads3.begin(0x4B)) {
    // Failed to initialize ADS1115 at address 0x4B
  } else {
    ads3.setGain(GAIN_ONE);
  }
  
  // Initialize MCP23017
  if (!mcp.begin_I2C()) {
    mcpInitialized = false;
  } else {
    mcpInitialized = true;
    
    // Configure MCP pins as outputs
    for (int i = 0; i < 8; i++) {
      mcp.pinMode(i, OUTPUT);
      //mcp.digitalWrite(i, LOW);
    }
  }
  
  // Initialize WiFi and Firebase before creating tasks
  initWiFi();
  
  // Draw initial screen
  drawMonitorTab();
  
  // Only create tasks after all hardware is initialized - REDUCED STACK SIZES
  xTaskCreatePinnedToCore(touchTask, "Touch", 3072, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(Task_Read_Gas, "Gas", 2048, NULL, 1, &TaskGasHandle, 0);
  xTaskCreatePinnedToCore(Task_Read_Water, "Water", 2048, NULL, 1, &TaskWaterHandle, 0);
  xTaskCreatePinnedToCore(Task_Display, "Display", 3072, NULL, 1, &TaskDisplayHandle, 1);
  xTaskCreatePinnedToCore(FirebaseSendTask, "Firebase", 8192, NULL, 1, &TaskFirebaseHandle, 1);
 xTaskCreatePinnedToCore(Task_FirebaseRelayControl, "FirebaseRelayControl", 4096,NULL,1,NULL,1);
  
  setupInterrupts();
}

// ========== Main Loop ==========
void loop() {
  delay(20);

  static unsigned long lastMillis = 0;
  if (millis() - lastMillis > 50000) {
    pum_test();
    lastMillis = millis();
  }

  if (currentTab == CONTROL_TAB) {
    Control_inDisplayControlTab();
  }
  
  processButtonPress();

  // Trigger sensor reading and display update flags
  setFlag(&readGasSensorsFlag); // Ensure flag is declared before usage
  setFlag(&readWaterSensorsFlag);
  setFlag(&updateDisplayFlag);
}

// ========== Test Relay Toggle - Optimized ==========
void pum_test() {
  if (!mcpInitialized || currentTab == CONTROL_TAB || 
      (millis() - lastControlTabExitTime) < 5000) {
    return;
  }

  // Toggle relays every 10 seconds
  delay(10000);
  mcp.digitalWrite(0, HIGH);
  delay(10000);  
  mcp.digitalWrite(0, LOW);

  mcp.digitalWrite(1, HIGH);
  delay(10000);  // 
  mcp.digitalWrite(1, LOW);
}

//========== Read ADC sensor  Value  ========== (tds sensor, ph sensor,tubydity sensor, DO sensor)


void readPH() {
  int16_t adc = ads1.readADC_SingleEnded(PH_CHANNEL);
  phVoltage = (adc * 0.1875 / 1000.0) + PH_OFFSET;

  float slope = -5.6548 + (0.0361 * temperature);
  float intercept = 15.509 - (0.0272 * temperature);
  phValue = (slope * phVoltage) + intercept;

  if (phValue < 0) phValue = 0;
  if (phValue > 14) phValue = 14;
}

// ================ read tds sensor  ==================
void readTDS() {
int16_t adc = ads2.readADC_SingleEnded(TDS_CHANNEL);
float tdsVoltage = adc * 0.1875 / 1000.0;
tdsValue = (133.42 * pow(tdsVoltage, 3)) - (255.86 * pow(tdsVoltage, 2)) + (857.39 * tdsVoltage);

if (tdsValue < 0) tdsValue = 0;
}

void readTurbidity() {
static float voltage = 0.0f;

if (millis() - samplingTimeTurbidity > SAMPLING_INTERVAL) {
  int16_t adc = ads1.readADC_SingleEnded(TURBIDITY_CHANNEL);
  voltage = adc * 4.096 / 32767.0; // ADS1115 16bit, 4.096V full-scale

  if (voltage >= 3.8 && voltage <= 4.4) {
    turbidityNTU = 0;
  } else if (voltage < 1.0) {
    turbidityNTU = 5;
  } else if (voltage < 1.5) {
    turbidityNTU = 10;
  } else if (voltage < 2.0) {
    turbidityNTU = 15;
  } else if (voltage < 2.5) {
    turbidityNTU = 20;
  } else if (voltage < 3.0) {
    turbidityNTU = 25;
  } else if (voltage < 3.5) {
    turbidityNTU = 30;
  } else if (voltage < 4.0) {
    turbidityNTU = 35;
  } else if (voltage < 4.5) {
    turbidityNTU = 40;
  } else {
    turbidityNTU = 45;
  }

  samplingTimeTurbidity = millis();
}
}

// =================== Các Task FreeRTOS - Optimized ===================
void Task_Read_Gas(void *pvParameters) {
  for (;;) {
    if (readGasSensorsFlag) {
      readGasSensorsFlag = false; // Reset flag
      readNH3();
      readH2S();
      readNO2();
      readCH4();
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void Task_Read_Water(void *pvParameters) {
  for (;;) {
    if (readWaterSensorsFlag) {
      readWaterSensorsFlag = false; // Reset flag
      readPH();
      readTDS();
      readTurbidity();
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Optimized display task
void Task_Display(void *pvParameters) {
  unsigned long lastScreenUpdate = 0;
  unsigned long lastFirebaseUpdate = 0;
  
  for (;;) {
    if (updateDisplayFlag) {
      updateDisplayFlag = false; // Reset flag
      if (currentTab == MONITOR_TAB) {
        drawMonitorTab();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ========== Median Calculation ==========
float calculateMedian(int* arr, int size) {
  int sorted[size];
  memcpy(sorted, arr, size * sizeof(int));
  std::sort(sorted, sorted + size);
  if (size % 2 == 0) {
    return (sorted[size / 2 - 1] + sorted[size / 2]) / 2.0;
  } else {
    return sorted[size / 2];
  }
}
// =================== funtion read gas sensor ===================
void readNH3() {
  int16_t adc = ads2.readADC_SingleEnded(NH3_CHANNEL);
  nh3Voltage = (adc * 0.1875 / 1000.0) + NH3_OFFSET;
  // Convert voltage to PPM
  nh3PPM = (nh3Voltage - NH3_ZERO_VOLTAGE) * 1000.0 / NH3_SENSITIVITY;
  if (nh3PPM < 0) nh3PPM = 0; // Prevent negative readings
}

void readH2S() {
  int16_t adc = ads3.readADC_SingleEnded(H2S_CHANNEL);
  h2sVoltage = (adc * 0.1875 / 1000.0) + H2S_OFFSET;
  // Convert voltage to PPM
  h2sPPM = (h2sVoltage - H2S_ZERO_VOLTAGE) * 1000.0 / H2S_SENSITIVITY;
  if (h2sPPM < 0) h2sPPM = 0;
}

void readNO2() {
  int16_t adc = ads3.readADC_SingleEnded(NO2_CHANNEL);
  no2Voltage = (adc * 0.1875 / 1000.0) + NO2_OFFSET;
  // Convert voltage to PPM
  no2PPM = (no2Voltage - NO2_ZERO_VOLTAGE) * 1000.0 / NO2_SENSITIVITY;
  if (no2PPM < 0) no2PPM = 0;
}

void readCH4() {
  int16_t adc = ads3.readADC_SingleEnded(CH4_CHANNEL);
  ch4Voltage = (adc * 0.1875 / 1000.0) + CH4_OFFSET;
  // Convert voltage to PPM
  ch4PPM = (ch4Voltage - CH4_ZERO_VOLTAGE) * 1000.0 / CH4_SENSITIVITY;
  if (ch4PPM < 0) ch4PPM = 0;
}
// read DO sensor
void readDO() {
  int16_t adc = ads2.readADC_SingleEnded(DO_CHANNEL);
  float doVoltage = (adc * 0.1875 / 1000.0) + DO_OFFSET;
  // Convert voltage to PPM (example conversion, adjust as needed)
  doValue = (doVoltage - DO_ZERO_VOLTAGE) * 1000.0 / DO_SENSITIVITY;
  if (doValue < 0) doValue = 0;
}
// read DS18B20 sensor
void readDS18B20() {
  int16_t adc = ads3.readADC_SingleEnded(DS18B20_CHANNEL);
  float ds18b20Voltage = (adc * 0.1875 / 1000.0) + DS18B20_OFFSET;
  // Convert voltage to temperature (example conversion, adjust as needed)
  ds18b20Value = (ds18b20Voltage - DS18B20_ZERO_VOLTAGE) * 1000.0 / DS18B20_SENSITIVITY;
  if (ds18b20Value < -55) ds18b20Value = -55; // Prevent unrealistic readings
}
// read rain sensor
void readRain() {
  int16_t adc = ads2.readADC_SingleEnded(RAIN_CHANNEL);
  float rainVoltage = (adc * 0.1875 / 1000.0) + RAIN_OFFSET;
  // Convert voltage to mm (example conversion, adjust as needed)
  rainValue = (rainVoltage - RAIN_ZERO_VOLTAGE) * 1000.0 / RAIN_SENSITIVITY;
  if (rainValue < 0) rainValue = 0; // Prevent negative readings
}

// ========== Control Tab Priority Logic ==========
void Control_inDisplayControlTab() {
  if (!mcpInitialized) {
    static unsigned long lastWarningTime = 0;
    if (millis() - lastWarningTime > 5000) {
      lastWarningTime = millis();
    }
    return;
  }
  
  for (int i = 0; i < BTN_COUNT; i++) {
    bool currentState = mcp.digitalRead(i);
    bool desiredState = btnState[i];
    
    if (currentState != desiredState) {
      mcp.digitalWrite(i, desiredState ? HIGH : LOW);
    }
  }
}
// ========== Touch Input Handler ==========
void handleSwipeGesture(int deltaY); // Forward declaration of handleSwipeGesture

void touchTask(void* parameter) {
  static int lastY = -1;

  while (true) {
    if (digitalRead(TOUCH_IRQ) == LOW) {
      uint16_t xRaw = readTouchAxis(0xD0);
      uint16_t yRaw = readTouchAxis(0x90);
      int x = map(xRaw, 300, 3800, 0, tft.width());
      int y = map(yRaw, 3800, 300, 0, tft.height());

      if (lastY != -1 && abs(y - lastY) > 10) { // Detect swipe
        handleSwipeGesture(y - lastY);
      }

      lastY = y;

      if (y <= 60) {
        Tab tappedTab = (Tab)(x / 80);
        if (tappedTab != currentTab) {
          changeTab(tappedTab);
        }
        delay(300);
      } else if (currentTab == CONTROL_TAB) {
        for (int i = 0; i < BTN_COUNT; i++) {
          if (x >= btnX[i] && x <= btnX[i] + BTN_WIDTH &&
              y >= btnY[i] && y <= btnY[i] + BTN_HEIGHT) {
            // Toggle button state
            btnState[i] = !btnState[i];
            drawButton(i, btnState[i]);

            // Update relay state on Firebase
            String state = btnState[i] ? "ON" : "OFF";
            updateRelayState(i, state);

            delay(200);
          }
        }
      }
    } else {
      lastY = -1; // Reset lastY when touch is released
    }
    delay(10);
  }
}

// Optimized Firebase task
void FirebaseSendTask(void *pvParameters) {
  // Initialize WiFi and Firebase first
  initWiFi();
  delay(1000);
  initFirebase();
  
  while (true) {
    // Check WiFi connection before sendpumpum data
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
      
      int timeout = 0;
      while (WiFi.status() != WL_CONNECTED && timeout < 10) {
        delay(500);
        timeout++;
      }
      
      if (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(6000 / portTICK_PERIOD_MS); // Longer delay on failure
        continue;
      }
    }
    
    sendSensorDataToFirebase();
    vTaskDelay(6000 / portTICK_PERIOD_MS); // Increase to 6 seconds to reduce updates
  }
}

// Add these functions to your code
void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
  } else {
  }
}

void initFirebase() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  
  // Try anonymous sign-in with proper error handling
  if (Firebase.signUp(&config, &auth, "", "")) {
    firebaseConnected = true;
    
    config.token_status_callback = tokenStatusCallback;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    
  } else {
    if (config.signer.signupError.message.length() > 0) {
    }
    firebaseConnected = false;
  }
}

// Optimized Firebase data sending
void sendSensorDataToFirebase() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (!firebaseConnected) {
    initFirebase();
    if (!firebaseConnected) {
      return;
    }
  }

  FirebaseJson json;
  
  // Simplified data structure - only send critical data
  json.set("gas/nh3", nh3PPM);
  json.set("gas/h2s", h2sPPM);
  json.set("gas/no2", no2PPM);
  json.set("gas/ch4", ch4PPM);
  json.set("water/ph", phValue);
  json.set("water/tds", tdsValue);
  json.set("water/turbidity", turbidityNTU);
  json.set("water/temp", temperature);
  json.set("water/do", doValue);
  json.set("water/rain", rainValue);
  json.set("water/ds18b20", ds18b20Value);
  json.set("ts", millis());
  
  Firebase.RTDB.updateNode(&fbdo, "/sensorData", &json);
}
// Control devices based on Firebase get  data to esp32  check message  on and off
void fetchRelayStatesFromFirebase() {
  if (WiFi.status() != WL_CONNECTED || !firebaseConnected) return;

  for (int i = 0; i < BTN_COUNT; i++) {
    String path = "/devices/relay" + String(i + 1) + "/message";  // relay1 đến relay8
    if (Firebase.RTDB.getString(&fbdo, path)) {
      String state = fbdo.stringData();
      bool relayOn = (state == "ON");

      btnState[i] = relayOn;
      mcp.digitalWrite(i, relayOn ? HIGH : LOW);
      Serial.printf("Relay %d set to %s\n", i + 1, state.c_str());
    } else {
      Serial.printf("Failed to read %s: %s\n", path.c_str(), fbdo.errorReason().c_str());
    }
  }
}

void Task_FirebaseRelayControl(void *pvParameters) {
  while (true) {
    fetchRelayStatesFromFirebase();
    vTaskDelay(pdMS_TO_TICKS(5000)); // Kiểm tra mỗi 5 giây
  }
}

// Function to send button state to Firebase
void updateRelayState(uint8_t relayPin, const String& state) {
  if (WiFi.status() != WL_CONNECTED || !firebaseConnected) return;

  String path = "/devices/relay" + String(relayPin + 1) + "/message";  // relay1 to relay8
  if (Firebase.RTDB.setString(&fbdo, path, state)) {
    Serial.printf("Relay %d state updated to %s on Firebase\n", relayPin + 1, state.c_str());
  } else {
    Serial.printf("Failed to update relay %d state: %s\n", relayPin + 1, fbdo.errorReason().c_str());
  }
}

// ========== Scroll Handling ==========
int scrollOffset = 0; // Tracks the vertical scroll offset
const int maxScrollOffset = 100; // Adjust based on the number of parameters
const int scrollStep = 20; // Amount to scroll per swipe

void scrollDisplay(int delta) {
  scrollOffset += delta;
  if (scrollOffset < 0) scrollOffset = 0;
  if (scrollOffset > maxScrollOffset) scrollOffset = maxScrollOffset;

  drawMonitorTab(); // Redraw the monitor tab with the updated scroll offset
}

void handleSwipeGesture(int deltaY) {
  if (deltaY > 0 && currentPage > 0) {
    currentPage--; // Swipe down to go to the previous page
  } else if (deltaY < 0 && currentPage < 1) {
    currentPage++; // Swipe up to go to the next page
  }
  drawMonitorTab(); // Redraw the monitor tab with the updated page
}
