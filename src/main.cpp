#include <config.h>
#include <structs.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP085.h>
#include <math.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_BMP085 bmp;
bool bmpReady = false;

RotorChannel front{PIN_SENSOR_FRONT};
RotorChannel rear{PIN_SENSOR_REAR};

float measuredRPM = 0;
float ambientPa = SEA_LEVEL_PA;
bool crankDetected = false;

#ifdef SIMULATION_MODE
// ---- Synthetic sensor for Wokwi testing ----
unsigned long simTestStartMs = 0;

void resetSimulation()
{
    simTestStartMs = millis();
}

float cumulativeRevs(float elapsedMs)
{
    const float rampMs = 800.0f;
    const float startRPM = 90.0f;
    const float steadyRPM = 220.0f;

    if (elapsedMs <= rampMs)
    {
        return (startRPM * elapsedMs +
                (steadyRPM - startRPM) * elapsedMs * elapsedMs / (2.0f * rampMs)) /
               60000.0f;
    }
    float revsAtRamp = (startRPM * rampMs + (steadyRPM - startRPM) * rampMs / 2.0f) / 60000.0f;
    return revsAtRamp + steadyRPM * (elapsedMs - rampMs) / 60000.0f;
}

float simulateBar(int pin)
{
    float elapsedMs = (float)(millis() - simTestStartMs);
    float phaseOffset = (pin == PIN_SENSOR_FRONT) ? 0.0f : 0.5f;
    float revs = cumulativeRevs(elapsedMs) + phaseOffset;

    float segment = fmodf(revs, 3.0f);
    int faceIndex = (int)segment;
    float withinFace = segment - faceIndex;

    const float windowFraction = 0.15f;
    const float windowStart = 0.5f - windowFraction / 2.0f;
    const float windowEnd = 0.5f + windowFraction / 2.0f;

    float depthFactor = 0.0f;

    if (withinFace >= windowStart && withinFace <= windowEnd)
    {
        float local = (withinFace - windowStart) / windowFraction;
        depthFactor = 1.0f - fabsf(local - 0.5f) * 2.0f;
    }

    const float baselineBar = 0.5f;
    float peakHeightBar = 8.0f;

    // Make front rotor's face 2 read noticeably worse, to prove the display
    // can show an uneven result (simulates a marginal seal).
    if (pin == PIN_SENSOR_FRONT && faceIndex == 1)
        peakHeightBar = 4.0f;

    float value = baselineBar + peakHeightBar * depthFactor;
    value += ((float)random(-3, 3)) / 100.0f; // +/- 0.03 bar jitter
    return value < 0 ? 0 : value;
}
#endif

float readBar(int pin)
{
#ifdef SIMULATION_MODE
    return simulateBar(pin);
#else
    int raw = analogRead(pin);
    float vAdc = (raw / (float)ADC_MAX) * ADC_VREF;
    float vSensor = vAdc / DIVIDER_RATIO;
    float bar = (vSensor - SENSOR_VMIN) / (SENSOR_VMAX - SENSOR_VMIN) * SENSOR_BAR_MAX;
    return bar < 0 ? 0 : bar;
#endif
}

void confirmFaceValue(RotorChannel &ch, float value, unsigned long now, bool wasNatural)
{
    bool trustInterval = wasNatural && !ch.lastConfirmWasForced;

    if (trustInterval && ch.lastDipTime != 0)
        ch.lastPulseInterval = now - ch.lastDipTime;

    if (ch.dipCounter >= WARMUP_DIPS)
    {
        if (trustInterval && ch.lastDipTime != 0)
        {
            unsigned long interval = now - ch.lastDipTime;
            ch.intervalSumMs += interval;
            ch.intervalCount++;
            Serial.print(ch.pin == PIN_SENSOR_FRONT ? "FRONT" : "REAR");
            Serial.print(" pulse#");
            Serial.print(ch.dipCounter);
            Serial.print(" interval=");
            Serial.print(interval);
            Serial.println("ms");
        }
        int slot = (ch.dipCounter - WARMUP_DIPS) % FACES_PER_ROTOR;

        if (!ch.faceHasData[slot] || value < ch.facePeak[slot])
            ch.facePeak[slot] = value;
        ch.faceHasData[slot] = true;
    }

    ch.lastDipTime = now;
    ch.dipCounter++;
    ch.lastConfirmWasForced = !wasNatural;
}

void updateChannel(RotorChannel &ch)
{
    float reading = readBar(ch.pin);
    unsigned long now = millis();

    if (ch.windowStartMs == 0)
        ch.windowStartMs = now;

    unsigned long expectedMs = ch.lastPulseInterval > 0 ? ch.lastPulseInterval : DEFAULT_EXPECTED_INTERVAL_MS;
    unsigned long timeoutMs = (unsigned long)(expectedMs * FACE_TIMEOUT_MULTIPLIER);
    bool windowExpired = (now - ch.windowStartMs) > timeoutMs;

    if (ch.rising)
    {
        if (reading > ch.currentPeak)
            ch.currentPeak = reading;
    }
    else
    {
        if (reading < ch.troughRef)
            ch.troughRef = reading;
    }

    bool amplitudeConfirmed = ch.rising
                                  ? (ch.currentPeak - reading) > MIN_DIP_DEPTH_BAR
                                  : (reading - ch.troughRef) > MIN_DIP_DEPTH_BAR;

    if (ch.rising && amplitudeConfirmed)
    {
        confirmFaceValue(ch, ch.currentPeak, now, true);
        ch.rising = false;
        ch.troughRef = reading;
        ch.windowStartMs = now;
    }
    else if (!ch.rising && amplitudeConfirmed)
    {

        ch.rising = true;
        ch.currentPeak = reading;
    }
    else if (windowExpired)
    {
        float forcedValue = ch.rising ? ch.currentPeak : reading;
        confirmFaceValue(ch, forcedValue, now, false);
        ch.rising = true;
        ch.currentPeak = reading;
        ch.windowStartMs = now;
    }

    ch.lastReading = reading;
}

float calculateRPM()
{
    unsigned long totalMs = front.intervalSumMs + rear.intervalSumMs;
    int totalCount = front.intervalCount + rear.intervalCount;
    if (totalCount == 0)
        return 0;

    float avgIntervalMs = (float)totalMs / totalCount;
    if (avgIntervalMs <= 0)
        return 0;
    return 60000.0f / avgIntervalMs;
}

float rpmCorrectionFactor(float rpm)
{
    if (rpm <= 0)
        return 1.0f;
    return 1.0f + RPM_CORRECTION_COEFF * (REFERENCE_RPM - rpm);
}

// Scales result depending on the altitude where engine is measured
float altitudeCorrectionFactor(float pa)
{
    if (pa <= 0)
        return 1.0f;
    float factor = SEA_LEVEL_PA / pa;
    if (factor < 0.7f)
        factor = 0.7f;
    if (factor > 1.3f)
        factor = 1.3f;
    return factor;
}

float normalizeReading(float rawBar, float rpm, float pa)
{
    return rawBar * rpmCorrectionFactor(rpm) * altitudeCorrectionFactor(pa);
}

void drawResults(bool testRunning, bool lowSamples)
{
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    if (!testRunning && lowSamples)
    {
        display.print(crankDetected ? "LOW SAMPLES-RETEST" : "NO CRANK - RETRY");
    }
    else if (measuredRPM > 0)
    {
        display.print("RPM ");
        display.print((int)measuredRPM);
        display.setCursor(64, 0);
        display.print("Unit: bar");
    }
    else
    {
        if (!testRunning)
            display.print("COMPRESSION TEST");
        else
            display.print(crankDetected ? "TESTING..." : "WAITING FOR CRANK");
    }
    display.drawLine(0, 9, 128, 9, SSD1306_WHITE);

    if (testRunning)
    {
        display.display();
        return;
    }

    display.setCursor(20 + 5, 12);
    display.print("F1");
    display.setCursor(58 + 5, 12);
    display.print("F2");
    display.setCursor(96 + 5, 12);
    display.print("F3");

    display.setTextSize(1);
    display.setCursor(0, 24);
    display.print("FR");
    display.setTextSize(1);
    for (int i = 0; i < FACES_PER_ROTOR; i++)
    {
        display.setCursor(20 + i * 38, 24);
        if (front.faceHasData[i])
            display.print(normalizeReading(front.facePeak[i], measuredRPM, ambientPa), 1);
        else
            display.print("--");
    }

    display.setTextSize(1);
    display.setCursor(0, 48);
    display.print("RR");
    display.setTextSize(1);
    for (int i = 0; i < FACES_PER_ROTOR; i++)
    {
        display.setCursor(20 + i * 38, 48);
        if (rear.faceHasData[i])
            display.print(normalizeReading(rear.facePeak[i], measuredRPM, ambientPa), 1);
        else
            display.print("--");
    }
    display.setTextSize(1);

    display.display();
}

void setup()
{
    Serial.begin(115200);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    analogReadResolution(12);

#ifdef SIMULATION_MODE
    Serial.println(">>> SIMULATION_MODE ACTIVE <<<");
#endif

    Wire.begin(D4, D5);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println("OLED not found");
        while (true)
            delay(1000);
    }

    bmpReady = bmp.begin();
    if (!bmpReady)
    {
        Serial.println("BMP180 not found - altitude correction disabled, assuming sea level");
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 16);
    display.println("Press BOOT to");
    display.println("start test, then");
    display.println("crank the engine.");
    display.display();
}

void loop()
{
    static bool testRunning = false;
    static bool lastButtonState = HIGH;
    static unsigned long lastSample = 0;
    static unsigned long phaseStartMs = 0;

    bool buttonNow = (digitalRead(PIN_BUTTON) == LOW);

    if (buttonNow && !lastButtonState && !testRunning)
    {
        front.reset();
        rear.reset();
        measuredRPM = 0;
        crankDetected = false;
        ambientPa = bmpReady ? (float)bmp.readPressure() : SEA_LEVEL_PA;
        phaseStartMs = millis();
#ifdef SIMULATION_MODE
        resetSimulation();
#endif
        testRunning = true;
        drawResults(true, false);
    }
    lastButtonState = buttonNow;

    if (testRunning)
    {
        unsigned long now = millis();
        if (now - lastSample >= SAMPLE_INTERVAL_MS)
        {
            lastSample = now;
            updateChannel(front);
            updateChannel(rear);

            if (!crankDetected)
            {
                bool frontConfirmed = front.dipCounter >= 2 &&
                                      front.lastPulseInterval >= MIN_PLAUSIBLE_INTERVAL_MS &&
                                      front.lastPulseInterval <= MAX_PLAUSIBLE_INTERVAL_MS;
                bool rearConfirmed = rear.dipCounter >= 2 &&
                                     rear.lastPulseInterval >= MIN_PLAUSIBLE_INTERVAL_MS &&
                                     rear.lastPulseInterval <= MAX_PLAUSIBLE_INTERVAL_MS;

                if (frontConfirmed || rearConfirmed)
                {
                    // Two consecutive pulses at a plausible cranking speed
                    crankDetected = true;
                    phaseStartMs = now;
                    drawResults(true, false);
                }
            }

            if (crankDetected)
                measuredRPM = calculateRPM();
        }

        bool enoughData = crankDetected &&
                          front.steadyPulses() >= MIN_STEADY_PULSES &&
                          rear.steadyPulses() >= MIN_STEADY_PULSES;

        unsigned long limit = crankDetected ? MAX_TEST_MS : WAIT_FOR_CRANK_TIMEOUT_MS;
        bool timedOut = (now - phaseStartMs) >= limit;

        if (enoughData || timedOut)
        {
            testRunning = false;
            bool lowSamples = timedOut && !enoughData;
            drawResults(false, lowSamples);
        }
    }
}
