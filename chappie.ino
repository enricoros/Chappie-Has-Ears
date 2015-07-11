// Copyright Enrico Ros 2016. All rights reserved.

#include <Arduino.h>
#include <Servo.h>

// uncomment the following to enable debugging on errors (disable for prod!)
//#define DEBUGGING


// misc constants
#define LILY_PIN_LED            13  // D13
#define LILY_HW_SERIAL_SPEED    57600 // fixed, can't go beyond it (errors)
#define MASTER_LOOP_DELAY       19  // ms

// additional console
#define LILY_SW_CONSOLE_SPEED   57600
#define LILY_PIN_SW_CONSOLE_RX  2   // D2
#define LILY_PIN_SW_CONSOLE_TX  3   // D3

// BT pins
#define LILY_PIN_BT_RECONFIG    4   // D4 (pullup)

// Servo(s) pins and params
#define LILY_PIN_SERVO_L_EAR    11  // D11
#define LILY_PIN_SERVO_R_EAR    12  // D12
#define EARS_MAX_SPEED          16  // max steps per loop
#define EARS_MAX_ACCEL          2   // max dSteps per loop
#define EARS_MAGIC_DECEL        2   // magic constant to start decelerating at the right time

// constant operation pins
#define LILY_PIN_MODE_ANALOG_A  5   // D5 (pullup)
#define LILY_PIN_MODE_ANALOG_B  6   // D6 (pullup)
#define LILY_PIN_MODE_SIT       7   // D7 (pullup)
#define LILY_PIN_MODE_STANDUP   8   // D8 (pullup)
#define LILY_PIN_MODE_DEMO      9   // D9 (pullup)



// the global console: can be null, can be Software (since the HW serial is busy with bluetooth)
//#define CONSOLE_PRESENT
#define CONSOLE_SOFTWARE

// console types implementation
#if defined(CONSOLE_PRESENT)
Stream * _Console = 0;
#define CONSOLE_ADD(...)        _Console->print(__VA_ARGS__)
#define CONSOLE_LINE(...)       _Console->println(__VA_ARGS__)
#if defined(CONSOLE_SOFTWARE)
#include <SoftwareSerial.h>
void initConsole() {
    SoftwareSerial *swConsole = new SoftwareSerial(LILY_PIN_SW_CONSOLE_RX, LILY_PIN_SW_CONSOLE_TX);
    swConsole->begin(LILY_SW_CONSOLE_SPEED);
    _Console = swConsole;
}
#else
void initConsole() {
    Serial.begin(LILY_HW_SERIAL_SPEED);
    _Console = &Serial;
}
#endif
#else
#undef CONSOLE_SOFTWARE
#define CONSOLE_ADD(...)
#define CONSOLE_LINE(...)
void initConsole() {}
#endif


static void initOutput(int pin, int level) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, level);
}

void crossStreams(Stream *s1, Stream *s2) {
    while (s1 && s1->available()) {
        const char c1 = (char)s1->read();
        if (s2)
            s2->print(c1);
    }
    while (s2 && s2->available()) {
        const char c2 = (char)s2->read();
        if (s1)
            s1->print(c2);
    }
}



/**
 * @brief The BlueLink class communicates via a Blueetooth dongle
 *
 * This class can also activate passthrough mode (for AT commands, for example)
 * to another serial (e.g. serial 0).
 *
 * The serial packet is comprised by: { 0xA5, B1, B2, B3, B4 },
 *   where the first is the sync byte (in case we lost it), and the other
 *   come from the packet encoding standard.
 */
class BlueLink {
    HardwareSerial *m_btSerial;
public:
    BlueLink(HardwareSerial *btSerial)
        : m_btSerial(btSerial)
    {
    }

    void init() const {
        // check if the user requested to reconfigure
        if (digitalRead(LILY_PIN_BT_RECONFIG) == LOW)
            initReconfigure();
        else
            initNormal();
    }

    /**
     * @brief readPacket Decodes an incoming instruction packet transmission. Enrico's standard.
     * @param destBuffer a pointer to at least maxLength chars
     * @param maxLength of the destination command (the src packet has 1 sync byte too)
     * @return true if one or more packets have been decoded (just the last is kept though)
     */
    bool readPacket(byte *destBuffer, int maxLength) {
        // we need maxLength + 1 sync byte
        while (m_btSerial->available() >= (maxLength + 1)) {
            // validate the sync byte
            byte syncByte = (byte)m_btSerial->read();
            if (syncByte != 0xA5) {
                // ERROR: we were unsynced
                CONSOLE_ADD("e21: ");
                CONSOLE_LINE((int)syncByte);
                // try the next
                continue;
            }

            // read the next 4 bytes
            destBuffer[0] = (byte)m_btSerial->read();
            destBuffer[1] = (byte)m_btSerial->read();
            destBuffer[2] = (byte)m_btSerial->read();
            destBuffer[3] = (byte)m_btSerial->read();
            // we're ok for further processing
            return true;
        }

        // we didn't get any packet
        return false;
    }

private:
    void initNormal() const {
        CONSOLE_ADD("   * open BT... ");
        m_btSerial->begin(LILY_HW_SERIAL_SPEED, SERIAL_8N1);
        delay(100);
        CONSOLE_LINE("done.");
    }

    /* Reconf Manually with:
     *  $$$ SF,1\n SN,Chappie-Ears\n SP,1337\n ST,0\n SU,57.6\n ---\n
     */
    bool initReconfigure() const {
        CONSOLE_LINE("   * reconfiguraton requested ");

        // jump up to the modem speed and move it down temporarily to 57600
        if (true /*lowerBtSpeed*/) {
            // start with the modem speed
            CONSOLE_ADD("   * lowering BT modem speed to 57600.. ");
            m_btSerial->begin(115200);
            delay(100);
            m_btSerial->print("$");
            m_btSerial->print("$");
            m_btSerial->print("$");
            delay(100);
            // move the modem to 57600 and in data mode
            m_btSerial->println("U,57.6,N");
            CONSOLE_LINE("done");
        }

        // now open at the right speed
        initNormal();

        CONSOLE_ADD("   * reconfiguring BT... ");

        // enter command mode
        if (!writeAndConfirm("$$$", "CMD\r\n", 5, 1000))
            return false;

        // factory config
        if (!writeAndConfirm("SF,1\n", "AOK\r\n", 5, 2000))
            return false;

        // rename device
        if (!writeAndConfirm("SN,Chappie-Ears\n", "AOK\r\n", 5, 1000))
            return false;

        // set pin to 1337
        if (!writeAndConfirm("SP,1337\n", "AOK\r\n", 5, 1000))
            return false;

        // turn config timer off
        if (!writeAndConfirm("ST,0\n", "AOK\r\n", 5, 1000))
            return false;

        // set the default speed
        if (!writeAndConfirm("SU,57.6\n", "AOK\r\n", 5, 1000))
            return false;

        // set the GPIO2 to Input, to not have the leds, which save power (restore with S%,0404)
        if (!writeAndConfirm("S%,0400\n", "AOK\r\n", 5, 1000))
            return false;

        // go back to data mode
        if (!writeAndConfirm("---\n", "END\r\n", 5, 1000))
            return false;

        CONSOLE_LINE("done.");
        return true;
    }

    bool writeAndConfirm(const char *msg, const char *expected, int length, int timeoutMs) const {
        m_btSerial->print(msg);
        bool ok = (expected == 0) || matchReply(expected, length, timeoutMs);
        if (!ok) {
            CONSOLE_ADD(ok ? "   * BlueLink: wrote: '" : "   * BlueLink: exception writing: '");
            CONSOLE_LINE(msg);
        }
        return ok;
    }

    bool matchReply(const char *expected, int length, int timeoutMs) const {
        int sIdx = 0;
        while (timeoutMs > 0) {
            while (m_btSerial->available()) {
                char c = (char)m_btSerial->read();
                if (expected[sIdx] != c) {
                    CONSOLE_ADD("   * BlueLink: expected ");
                    CONSOLE_ADD((int)expected[sIdx]);
                    CONSOLE_ADD(" gotten ");
                    CONSOLE_ADD((int)c);
                    CONSOLE_ADD(" at index ");
                    CONSOLE_LINE((int)sIdx);
                    return false;
                }
                sIdx++;
                if (sIdx >= length)
                    return true;
            }
            delay(1);
            timeoutMs--;
        }
        return false;
    }
};



/**
 * This class encapsulates the motor control of Chappie
 */
class ChappieEars
{
public:
    ChappieEars() /*: m_multiplier(1.0f)*/ {}

    /// Connect to the hardware pins. Call once
    void init(int pin_leftEar, int pin_rightEar) {
        m_LeftEar.attachToPin(pin_leftEar);
        m_RightEar.attachToPin(pin_rightEar);
        setLeftEar(1.0, true);
        setRightEar(1.0, true);
    }

    // individual controls (all between -1 ... 1)
#define SETTERS(setter, getter, control, modifier) \
    void setter (float np /* -1 ... 1 */, bool instantaneous = false) {\
        control.setTargetNp(/*(m_multiplier != 1.0) ? (np * m_multiplier) :*/ modifier np, instantaneous); \
    } \
    float getter () { return control.setNp; }
    SETTERS(setLeftEar, getLeftEar, m_LeftEar, -)
    SETTERS(setRightEar, getRightEar, m_RightEar,)

#ifdef EARS_MAX_SPEED
    // loop to move smoothly
    void loop() {
        if (m_LeftEar.inMotion)
            m_LeftEar.reachTarget();
        if (m_RightEar.inMotion)
            m_RightEar.reachTarget();
    }
#endif

    // limits (active when setting the next setpoint)
    //void setMultiplier(float m) { m_multiplier = constrain(m, 0.01, 2.0); }
    //float getMultiplier() const { return m_multiplier; }

private:
    struct Control {
        Servo servo;
        float setNp;
        int targetPos;

        int currentPos;
        int currentSpeed;

        bool inMotion;

        Control() : setNp(0), targetPos(90), currentPos(90), currentSpeed(0), inMotion(false) {}

        void attachToPin(int pin) {
            servo.attach(pin);
        }

        void setTargetNp(float np, bool force) {
            if (np == setNp && !force)
                return;
            setNp = np;
            targetPos = round(np * 90.0f) + 90;
            if (targetPos < 0)
                targetPos = 0;
            else if (targetPos > 180)
                targetPos = 180;
            if (force) {
                currentPos = targetPos;
                currentSpeed = 0;
                inMotion = false;
                apply(targetPos);
                return;
            }
            if (targetPos != currentPos) {
#ifdef EARS_MAX_SPEED
                inMotion = true;
#else
                apply(targetPos);
#endif
            }
        }

#ifdef EARS_MAX_SPEED
        void reachTarget() {
            // use a limited Velocity and Acceleration algorithm
            int dPos = targetPos - currentPos;
            if (dPos > 0) {
                // adapt speed (and keep it in the 1 .. MAX_SPEED range)
                int tSpeedP = EARS_MAGIC_DECEL * sqrt(dPos); // note, the multiplier should be dependent on the SMOOTH_ constants
                if (tSpeedP > currentSpeed) {
                    currentSpeed += min(tSpeedP - currentSpeed, EARS_MAX_ACCEL);
                    if (currentSpeed > EARS_MAX_SPEED)
                        currentSpeed = EARS_MAX_SPEED;
                } else if (tSpeedP < currentSpeed)
                    currentSpeed -= min(currentSpeed - tSpeedP, EARS_MAX_ACCEL);

                // change position
                apply(currentPos + min(dPos, currentSpeed));
            } else if (dPos < 0) {
                // adapt speed (and keep it in the -MAX_SPEED .. -1 range)
                int tSpeedN = EARS_MAGIC_DECEL * -sqrt(-dPos); // note, the multiplier should be dependent on the SMOOTH_ constants
                if (tSpeedN < currentSpeed) {
                    currentSpeed -= min(currentSpeed - tSpeedN, EARS_MAX_ACCEL);
                    if (currentSpeed < -EARS_MAX_SPEED)
                        currentSpeed = -EARS_MAX_SPEED;
                } else if (tSpeedN > currentSpeed)
                    currentSpeed += min(tSpeedN - currentSpeed, EARS_MAX_ACCEL);

                // change position
                apply(currentPos + max(dPos, currentSpeed));
            }
            if (currentPos == targetPos) {
                currentSpeed = 0;
                inMotion = false;
            }
        }
#endif

        void apply(int pos) {
            CONSOLE_ADD("apply: ");
            CONSOLE_LINE(pos);
            servo.write(constrain(pos, 0, 180));
            currentPos = pos;
        }
    } m_LeftEar, m_RightEar;
};


/**
 * @brief The DemoMode class runs an automated demo sequence, taking control of the main system loop.
 */
class DemoMode {
private:
    ChappieEars *mEars;
    long mNextDeadlineMs;

public:
    DemoMode(ChappieEars *chappieEars)
        : mEars(chappieEars)
    {
    }

    void intro() {
        for (int i = 300; i > 50; i -= i/3) {
            digitalWrite(LILY_PIN_LED, HIGH);
            delay(i/2);
            digitalWrite(LILY_PIN_LED, LOW);
            delay(i/2);
        }
        mNextDeadlineMs = 0;
    }

    void outro() {
        mEars->setLeftEar(1);
        mEars->setRightEar(1);
    }

    void run() {
        // if enought time passed, randomize the positions
        long currentTimeMs = millis();
        if (currentTimeMs < mNextDeadlineMs)
            return;

        // move the ears to a random point in space
        if (random(100) > 70) {
            float nextLeft = (float)random(-10, 101) / 100.0f;
            mEars->setLeftEar(nextLeft);
        }
        if (random(100) > 70) {
            float nextRight = (float)random(-10, 101) / 100.0f;
            mEars->setRightEar(nextRight);
        }

        // wait up to 2.5s with a trapezoidal probability distribution
        mNextDeadlineMs = currentTimeMs + random(2000) + random(500);
        if (random(100) >= 95)
            mNextDeadlineMs += random(15000);
    }
};


class LPF {
#define LPF_MAX_SIZE 16
public:
    void init(int size) {
        mSize = min(size, LPF_MAX_SIZE);
        mSizeF = (float)mSize;
        mItems = 0;
        mHeadIdx = 0;
    }

    // @return true if buffer is full and we ca read
    bool add(float n) {
        mBuffer[mHeadIdx++] = n;
        if (mHeadIdx >= mSize)
            mHeadIdx = 0;
        if (mItems < mSize)
            mItems++;
        return mItems == mSize;
    }

    float getAvg() const {
        float sum = 0;
        // NOTE: keeping a running sum would probably accumulate errors
        for (int i = 0; i < mSize; i++)
            sum += mBuffer[i];
        return sum / mSizeF;
    }

private:
    int mSize;
    float mSizeF;
    int mItems;
    float mBuffer[LPF_MAX_SIZE];
    int mHeadIdx;
};

/**
 * @brief React to Analog inputs
 */
class AnalogMode {
private:
    ChappieEars *mEars;
    int mLPin, mLMin, mLMax;
    int mRPin, mRMin, mRMax;
    bool mInverted;
    bool mLearnOnFly;
    LPF mLLpf, mRLpf;

public:
    AnalogMode(ChappieEars *chappieEars)
        : mEars(chappieEars)
    { }

    void intro(int type) {
        switch (type) {
        case 1:
            mInverted = false;
            mLearnOnFly = false;
            mLPin = A0;
            mRPin = A1;
            break;
        case 2:
            mInverted = true;
            mLearnOnFly = false;
            mLPin = A2;
            mRPin = A3;
            break;
        default:
            CONSOLE_LINE("wrong analog mode");
            return;
        }

        showFeedback(false);
        calibrate();
        showFeedback(true);

        mLLpf.init(8);
        mRLpf.init(8);
    }

    void calibrate() {
        mLMin = mRMin = 1023;
        mLMax = mRMax = 0;
        // do 200 reads in typical delay
        int value;
        for (int i = 0; i < 4000; i += 20) {
#define READ_LEARN_MINMAX(val, pin, inv, outMin, outMax) \
            val = analogRead(pin); if (inv) val = 1023 - val; \
            if (val > outMax) outMax = val; if (val < outMin) outMin = val;
            // learn
            READ_LEARN_MINMAX(value, mLPin, mInverted, mLMin, mLMax);
            READ_LEARN_MINMAX(value, mRPin, mInverted, mRMin, mRMax);
            // like in the prod run
            delay(20);
        }
        // safety constraints
#define KEEP_CONSTRAINED(max, min, low, high, gap) \
        if (min < low) min = low; if (max > high) max = high; \
        if ((max - min) < gap) { max = min + gap/2; min -= gap/2; } \
        if (min < low) min = low; if (max > high) max = high;
        KEEP_CONSTRAINED(mLMax, mLMin, 0, 1023, 60);
        KEEP_CONSTRAINED(mRMax, mRMin, 0, 1023, 60);
    }

    void run() {
        int tmpValue;
        float nL, nR;

        // perform the reads and normalization
#define PROCESS_CHANNEL(raw, pin, outMin, outMax, outNV, lpf, setter) \
        raw = analogRead(pin); \
        if (mInverted) raw = 1023 - raw; \
        if (mLearnOnFly) { \
            if (raw < outMin) outMin = raw; \
            if (raw > outMax) outMax = raw; \
        } else raw = constrain(raw, outMin, outMax); \
        if (true) { /* linear mapping */ outNV = 2 * ((float)(raw - outMin)) / ((float)(outMax - outMin)) - 1; } \
        if (lpf.add(outNV)) mEars->setter(lpf.getAvg());

        // read and aply
        PROCESS_CHANNEL(tmpValue, mLPin, mLMin, mLMax, nL, mLLpf, setLeftEar);
        PROCESS_CHANNEL(tmpValue, mRPin, mRMin, mRMax, nR, mRLpf, setRightEar);

        CONSOLE_LINE(mLLpf.getAvg());

        // console output
#if 0
        CONSOLE_ADD("L: ");
        CONSOLE_ADD(nL);
        CONSOLE_ADD(",  R: ");
        CONSOLE_ADD(nR);
        CONSOLE_LINE(" ");
#endif
    }

    void showFeedback(bool fast) {
        for (int i = (fast ? 300 : 900); i > 50; i -= i/3) {
            digitalWrite(LILY_PIN_LED, HIGH);
            delay(i/2);
            digitalWrite(LILY_PIN_LED, LOW);
            delay(i/2);
        }
    }
};



// runtime globals
ChappieEars * sChappieEars;
BlueLink * sBlueLink;
DemoMode * sDemoMode;
AnalogMode * sAnalogMode;
byte sCommandBuffer[4];
bool readSimpleCommand(Stream *stream);
bool executeCommandPacket(const byte *command);

void setupAndExplainPullup(int pin, const char * text) {
    pinMode(pin, INPUT_PULLUP);
    CONSOLE_ADD("   * pin ");
    CONSOLE_ADD(pin);
    CONSOLE_ADD(", ");
    CONSOLE_LINE(text);
}


void setup() {
    initOutput(LILY_PIN_LED, HIGH);
    initOutput(LILY_PIN_SERVO_L_EAR, LOW);
    initOutput(LILY_PIN_SERVO_R_EAR, LOW);
    pinMode(A0, INPUT_PULLUP);
    pinMode(A1, INPUT_PULLUP);
    pinMode(A2, INPUT_PULLUP);
    pinMode(A3, INPUT_PULLUP);

    // console
    initConsole();
    CONSOLE_LINE("Chappie Booting...");

    // i/os
    CONSOLE_LINE(" * init I/O");
    setupAndExplainPullup(LILY_PIN_MODE_ANALOG_A,   "Analog A mode ");
    setupAndExplainPullup(LILY_PIN_MODE_ANALOG_B,   "Analog B mode ");
    setupAndExplainPullup(LILY_PIN_MODE_SIT,        "Sit      mode ");
    setupAndExplainPullup(LILY_PIN_MODE_STANDUP,    "StandUp  mode ");
    setupAndExplainPullup(LILY_PIN_MODE_DEMO,       "DEMO     mode ");
    setupAndExplainPullup(LILY_PIN_BT_RECONFIG,     "BT reconfigure (boot) ");
    delay(100);

    // init Bluetooth
    CONSOLE_LINE(" * init BT");
    sBlueLink = new BlueLink(&Serial);
    sBlueLink->init();

    // init ears
    CONSOLE_LINE(" * init Ears");
    sChappieEars = new ChappieEars();
    sChappieEars->init(LILY_PIN_SERVO_L_EAR, LILY_PIN_SERVO_R_EAR);

    // init misc
    sDemoMode = new DemoMode(sChappieEars);
    sAnalogMode = new AnalogMode(sChappieEars);

    // explain...
    CONSOLE_LINE("Chappie Ready!");
    digitalWrite(LILY_PIN_LED, LOW);
}


enum Mode {
    Mode_Undefined  = 0,
    Mode_Sit        = 1,
    Mode_StandUp    = 2,
    Mode_AnalogA    = 3,
    Mode_AnalogB    = 4,
    Mode_Demo       = 5,
    Mode_BT         = 6
};
Mode sCurrentMode = Mode_Undefined;
Mode sForcedRemoteMode = Mode_Undefined;

void loop() {
    // uncomment the following 2 lines to enable a direct talk with the BT modem
    //crossStreams(&Serial, _Console);
    //return;

    // check the next mode (and execute instant sequences)
    Mode nextMode = sForcedRemoteMode;
    if (digitalRead(LILY_PIN_MODE_SIT) == LOW)
        nextMode = Mode_Sit;
    else if (digitalRead(LILY_PIN_MODE_STANDUP) == LOW)
        nextMode = Mode_StandUp;
    else if (digitalRead(LILY_PIN_MODE_ANALOG_A) == LOW)
        nextMode = Mode_AnalogA;
    else if (digitalRead(LILY_PIN_MODE_ANALOG_B) == LOW)
        nextMode = Mode_AnalogB;
    else if (digitalRead(LILY_PIN_MODE_DEMO) == LOW)
        nextMode = Mode_Demo;

    // execute transitions
    if (nextMode && nextMode != sCurrentMode) {
        // exit previous mode
        switch (sCurrentMode) {
        case Mode_Demo: sDemoMode->outro(); break;
        }

        // enter current mode
        sForcedRemoteMode = Mode_Undefined;
        sCurrentMode = nextMode;
        switch (sCurrentMode) {
        case Mode_Sit: sChappieEars->setLeftEar(-1); sChappieEars->setRightEar(-1); break;
        case Mode_StandUp: sChappieEars->setLeftEar(1); sChappieEars->setRightEar(1); break;
        case Mode_AnalogA: sAnalogMode->intro(1); break;
        case Mode_AnalogB: sAnalogMode->intro(2); break;
        case Mode_Demo: sDemoMode->intro(); break;
        }
    }

    // run current mode
    switch (sCurrentMode) {
    case Mode_AnalogA: case Mode_AnalogB: sAnalogMode->run(); break;
    case Mode_Demo: sDemoMode->run(); break;
    }

    // check the BT serial for new commands
    bool hasNewCommand = false;
    if (sBlueLink->readPacket(sCommandBuffer, 4))
        hasNewCommand = true;

#ifdef CONSOLE_PRESENT
    // check the console for new commands
    if (!hasNewCommand)
        hasNewCommand = readSimpleCommand(_Console);
#endif

    // execute a command, when received (and blink the LED)
    if (hasNewCommand)
        if (executeCommandPacket(sCommandBuffer))
            digitalWrite(LILY_PIN_LED, HIGH);

    // vital motion update
#ifdef EARS_MAX_SPEED
    sChappieEars->loop();
#endif

    // upper limit to the loop
    delay(MASTER_LOOP_DELAY);

    // save energy, stop the LED now
    if (hasNewCommand)
        digitalWrite(LILY_PIN_LED, LOW);
}


bool executeCommandPacket(const byte *msg) {
    const byte rCmd    = msg[0];
    const byte rTarget = msg[1];
    const byte rValue1 = msg[2];
    const byte rValue2 = msg[3];

    switch (rCmd) {
    // C: 01 -> set individual ears (target: 1..N, rValue1: angle [0..200])
    case 0x01:
        switch (rTarget) {
        case 1: sChappieEars->setLeftEar(((float)rValue1 - 100.0f) / 100.0f); return true;
        case 2: sChappieEars->setRightEar(((float)rValue1 - 100.0f) / 100.0f); return true;
        }; break;

    // C: 02 -> set ears pairs (target ignored)
    case 0x02: {
        float nLeft = ((float)rValue1 - 100.0f) / 100.0f;
        float nRight = ((float)rValue2 - 100.0f) / 100.0f;
        sChappieEars->setLeftEar(nLeft);
        sChappieEars->setRightEar(nRight);
        sForcedRemoteMode = Mode_BT;
        } return true;

    // C: 04 -> set flag (target: flag index) (flag specific behavior)
    case 0x04:
        switch (rTarget) {
        case 1: sForcedRemoteMode = rValue1 ? Mode_Demo : Mode_BT; return true;
        case 2: sForcedRemoteMode = rValue1 ? Mode_AnalogA : Mode_BT; return true;
        case 3: sForcedRemoteMode = rValue1 ? Mode_AnalogB : Mode_BT; return true;
        }; break;
    }

    // no/bad command
#ifdef DEBUGGING
    LOG_NL("e02");
#endif
    return false;
}

bool readSimpleCommand(Stream *stream) {
    while (stream->available()) {
        char c = (char)stream->read();

        // empty buffer (don't enable in prod! 100ms delays kill)
        //delay(100);
        //while (console->available())
        //    console->read();

        switch (c) {
        case 'u': case 'U':
            sCommandBuffer[0] = 2; sCommandBuffer[2] = 200; sCommandBuffer[3] = 200; return true;
        case 'm': case 'M':
            sCommandBuffer[0] = 2; sCommandBuffer[2] = 100; sCommandBuffer[3] = 100; return true;
        case 'd': case 'D':
            sCommandBuffer[0] = 2; sCommandBuffer[2] = 0; sCommandBuffer[3] = 0; return true;
        case 'l': case 'L':
            sCommandBuffer[0] = 2; sCommandBuffer[2] = 200; sCommandBuffer[3] = 50; return true;
        case 'r': case 'R':
            sCommandBuffer[0] = 2; sCommandBuffer[2] = 50; sCommandBuffer[3] = 200; return true;
        }

    }
    return false;
}
