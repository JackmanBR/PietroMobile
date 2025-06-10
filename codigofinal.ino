#define BLYNK_TEMPLATE_ID ""   // sete a contanste do seu templateId
#define BLYNK_TEMPLATE_NAME "" // sete constante do seu template name
#define BLYNK_AUTH_TOKEN ""    // sete a constante do seu blynk token
#include <time.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Blynk.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

uint32_t irBuffer[100];
uint32_t redBuffer[100];

#define REPORTING_PERIOD_MS 1000 // sete a frequencia do periodo de tempo das atualizações que serão enviadas ao app do blynk

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = ""; // sete o id da rede onde a esp32 vai ser conectada
char pass[] = ""; // sete a senha da rede onde a esp32 vai ser conectada

uint32_t tsLastReport = 0; // sete o valor que armazena a hora em que a última atualização foi enviada para o app do blynk

int32_t bufferLength;  // comprimento do dado
int32_t spo2;          // valor da taxa de oxigenação
int8_t validSPO2;      // valor para mostrar se o calculo de oxigenação foi válido
int32_t heartRate;     // velocidade do calculo dos batimentos cardíacos através do algoritmo de Maxim's.
int8_t validHeartRate; // valor para mostrar se o calculo de batimentos cardíacos foi válido

byte pulseLED = 2; // led de validade para a esp32
byte readLED = 19; // led para o envio de dados do blynk

long lastBeat = 0; // tempo da última ocorrência da suposta batida do coração

float beatsPerMinute;         // armazena o batimento de acordo com o algoritmo personalizado
int beatAvg = 0, sp02Avg = 0; // armazena o batimento médio médio e a taxa de oxigenação
float ledBlinkFreq;           // armazena a frequência para piscar o LED de pulso

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static const unsigned char PROGMEM logo2_bmp[] =
    {
        0x03,
        0xC0,
        0xF0,
        0x06,
        0x71,
        0x8C,
        0x0C,
        0x1B,
        0x06,
        0x18,
        0x0E,
        0x02,
        0x10,
        0x0C,
        0x03,
        0x10,
        0x04,
        0x01,
        0x10,
        0x04,
        0x01,
        0x10,
        0x40,
        0x01,
        0x10,
        0x40,
        0x01,
        0x10,
        0xC0,
        0x03,
        0x08,
        0x88,
        0x02,
        0x08,
        0xB8,
        0x04,
        0xFF,
        0x37,
        0x08,
        0x01,
        0x30,
        0x18,
        0x01,
        0x90,
        0x30,
        0x00,
        0xC0,
        0x60,
        0x00,
        0x60,
        0xC0,
        0x00,
        0x31,
        0x80,
        0x00,
        0x1B,
        0x00,
        0x00,
        0x0E,
        0x00,
        0x00,
        0x04,
        0x00,
};

static const unsigned char PROGMEM logo3_bmp[] =
    {0x01, 0xF0, 0x0F, 0x80, 0x06, 0x1C, 0x38, 0x60, 0x18, 0x06, 0x60, 0x18, 0x10, 0x01, 0x80, 0x08,
     0x20, 0x01, 0x80, 0x04, 0x40, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x02, 0xC0, 0x00, 0x08, 0x03,
     0x80, 0x00, 0x08, 0x01, 0x80, 0x00, 0x18, 0x01, 0x80, 0x00, 0x1C, 0x01, 0x80, 0x00, 0x14, 0x00,
     0x80, 0x00, 0x14, 0x00, 0x80, 0x00, 0x14, 0x00, 0x40, 0x10, 0x12, 0x00, 0x40, 0x10, 0x12, 0x00,
     0x7E, 0x1F, 0x23, 0xFE, 0x03, 0x31, 0xA0, 0x04, 0x01, 0xA0, 0xA0, 0x0C, 0x00, 0xA0, 0xA0, 0x08,
     0x00, 0x60, 0xE0, 0x10, 0x00, 0x20, 0x60, 0x20, 0x06, 0x00, 0x40, 0x60, 0x03, 0x00, 0x40, 0xC0,
     0x01, 0x80, 0x01, 0x80, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x30, 0x0C, 0x00,
     0x00, 0x08, 0x10, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x01, 0x80, 0x00};

void setup()
{
    ledcSetup(0, 0, 8);         // canal PWM = 0, frequencia inicial do PWM = 0Hz, resolução = 8 bits
    ledcAttachPin(pulseLED, 0); // conecte o pino pulseLED ao canal PWM 0
    ledcWrite(0, 255);          // anexo o pino pulseLED ao canal PWM 0

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.display();
    delay(3000);
    Blynk.begin(auth, ssid, pass);
    Serial.begin(115200);

    Serial.print("Inicializando PIETRO Mobile");

    // Inicializa o sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Use uma porta I2C padrão, 400kHz de velocidade
    {
        Serial.println(F("MAX30105 não encontrado. Por favor confira a ligação/configuração do mesmo!"));
        while (1)
            ;
    }

    byte ledBrightness = 50; // opção: 0=Off to 255=50mA
    byte sampleAverage = 1;  // opção: 1, 2, 4, 8, 16, 32
    byte ledMode = 2;        // opção: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    byte sampleRate = 100;   // opção: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 69;     // opção: 69, 118, 215, 411
    int adcRange = 4096;     // opção: 2048, 4096, 8192, 16384

    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configure o sensor com estas configurações
}

void loop()
{

    Blynk.run();

    bufferLength = 100; // comprimento do buffer de 100 armazena 4 segundos de amostras executadas a 25sps

    // leia as primeiras 100 amostras e determine o alcance do sinal
    for (byte i = 0; i < bufferLength; i++)
    {
        while (particleSensor.available() == false)
            particleSensor.check();

        redBuffer[i] = particleSensor.getIR();
        irBuffer[i] = particleSensor.getRed();
        particleSensor.nextSample();

        Serial.print(F("red: "));
        Serial.print(redBuffer[i], DEC);
        Serial.print(F("\t ir: "));
        Serial.println(irBuffer[i], DEC);
    }

    // calcular a frequência cardíaca e SpO2 após as primeiras 100 amostras (primeiros 4 segundos de amostras)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    // Coletando continuamente amostras do MAX30102. A frequência cardíaca e SpO2 são calculadas a cada 1 segundo
    while (1)
    {
        Blynk.run();
        // despejar os primeiros 25 conjuntos de amostras na memória e deslocar os últimos 75 conjuntos de amostras para o topo
        for (byte i = 25; i < 100; i++)
        {
            redBuffer[i - 25] = redBuffer[i];
            irBuffer[i - 25] = irBuffer[i];
        }

        // pegue 25 conjuntos de amostras antes de calcular a frequência cardíaca.
        for (byte i = 75; i < 100; i++)
        {
            while (particleSensor.available() == false) // temos novos dados?
                particleSensor.check();                 // Verifique o sensor para novos dados

            digitalWrite(readLED, !digitalRead(readLED)); // Pisca o LED integrado a cada leitura de dados

            redBuffer[i] = particleSensor.getRed();
            irBuffer[i] = particleSensor.getIR();
            particleSensor.nextSample(); // Terminamos esta amostra, então passe para a próxima amostra

            long irValue = irBuffer[i];

            // Calcule BPM independente do Algoritmo de Maxim.
            if (checkForBeat(irValue) == true)
            {

                // se o valor da batida for verdadeiro, as informações são enviadas para serem mostardas no display e o puslo é enviado ao buzzer para replicaar a batida através da emissão de som
                display.clearDisplay();
                display.drawBitmap(0, 0, logo3_bmp, 32, 32, WHITE);
                display.setTextSize(1.8);
                display.setTextColor(WHITE);
                display.setCursor(45, 0);
                display.println("BPM");
                display.setCursor(85, 0);
                display.println(beatAvg);
                display.setCursor(45, 25);
                display.println("SPO2%");
                display.setCursor(85, 25);
                display.println(oxy);
                display.display();
                tone(2, 1000);
                delay(100);
                noTone(2);

                // Sentimos uma batida!
                long delta = millis() - lastBeat;
                lastBeat = millis();

                beatsPerMinute = 60 / (delta / 1000.0);
                beatAvg = (beatAvg + beatsPerMinute) / 2;

                if (beatAvg != 0)
                    ledBlinkFreq = (float)(60.0 / beatAvg);
                else
                    ledBlinkFreq = 0;
                ledcWriteTone(0, ledBlinkFreq);
            }
            else
            {
                //se não temos batida, o display apenas mostra as informações zeradas
                display.clearDisplay();
                display.setTextSize(1);
                display.setTextColor(WHITE);
                display.setCursor(20, 5);
                display.println("PIETRO MOBILE ");
                display.setCursor(10, 15);
                display.println("coloque seu dedo");
                display.setCursor(30, 25);
                display.println("no sensor! ");
                display.display();
                noTone(3);
            }
            if (millis() - lastBeat > 10000)
            {
                beatsPerMinute = 0;
                beatAvg = (beatAvg + beatsPerMinute) / 2;

                if (beatAvg != 0)
                    ledBlinkFreq = (float)(60.0 / beatAvg);
                else
                    ledBlinkFreq = 0;
                ledcWriteTone(0, ledBlinkFreq);
            }
        }

        // Após coletar 25 novas amostras recalcular HR e SP02
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

        Serial.print(beatAvg, DEC);

        Serial.print(F("\t HRvalid="));
        Serial.print(validHeartRate, DEC);

        Serial.print(F("\t SPO2="));
        Serial.print(sp02Avg, DEC);

        Serial.print(F("\t SPO2Valid="));
        Serial.println(validSPO2, DEC);

        // Calcula o SPO2 médio para exibir transições suaves no aplicativo Blynk
        if (validSPO2 == 1 && spo2 < 100 && spo2 > 0)
        {
            sp02Avg = (sp02Avg + spo2) / 2;
        }
        else
        {
            spo2 = 0;
            sp02Avg = (sp02Avg + spo2) / 2;
            ;
        }

        // Envie dados para o aplicativo Blynk em intervalos regulares
        if (millis() - tsLastReport > REPORTING_PERIOD_MS)
        {
            //Em casos de sinais válidos, envio de dados as portas virtuais do Blynk
            Blynk.virtualWrite(V3, beatAvg);
            Blynk.virtualWrite(V4, sp02Avg);

            tsLastReport = millis();
        }else{
            //Em casos de sinais inválidos ou ausentes, envio de dados as portas virtuais do Blynk, só que enviado o valor "0"
            Blynk.virtualWrite(V3, 0);
            Blynk.virtualWrite(V4, 0);
        }
    }
}