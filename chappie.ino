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
#define LILY_PIN_SIT_MODE       7   // D7 (pullup)
#define LILY_PIN_STANDUP_MODE   8   // D8 (pullup)
#define LILY_PIN_DEMO_MODE      9   // D9 (pullup)




// the global console: can be null, can be Software (since the HW serial is busy with bluetooth)
#define CONSOLE_PRESENT
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
            servo.write(pos);
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
    bool mHasJumper;
    bool mSoftJumper;
    bool mWasEnabled;
    long mNextDeadlineMs;

public:
    DemoMode(ChappieEars *chappieEars)
        : mEars(chappieEars)
        , mHasJumper(false)
        , mSoftJumper(false)
        , mWasEnabled(false)
    {
    }

    bool getEnabled() {
        mHasJumper = digitalRead(LILY_PIN_DEMO_MODE) == LOW || mSoftJumper;
        return mHasJumper || mWasEnabled;
    }

    bool setSoftEnablement(bool enabled) {
        mSoftJumper = enabled;
    }

    void run() {
        // if just enabled: intro
        if (!mWasEnabled) {
            for (int i = 300; i > 50; i -= i/3) {
                digitalWrite(LILY_PIN_LED, HIGH);
                delay(i/2);
                digitalWrite(LILY_PIN_LED, LOW);
                delay(i/2);
            }
            mNextDeadlineMs = 0;
        }

        // if just disabled: outro
        if (!mHasJumper) {
            if (mWasEnabled) {
                mWasEnabled = false;
                mEars->setLeftEar(1);
                mEars->setRightEar(1);
            }
            return;
        }

        // if enought time passed, randomize the positions
        long currentTimeMs = millis();
        if (currentTimeMs < mNextDeadlineMs)
            return;

        // move the ears to a random point in space
        float nextLeft = (float)random(-10, 101) / 100.0f;
        float nextRight = (float)random(-10, 101) / 100.0f;
        mEars->setLeftEar(nextLeft);
        mEars->setRightEar(nextRight);

        // wait up to 2.5s with a trapezoidal probability distribution
        mNextDeadlineMs = currentTimeMs + random(2000) + random(500);
        if (random(100) >= 95)
            mNextDeadlineMs += random(15000);
        mWasEnabled = true;
        return;
    }
};



// runtime globals
ChappieEars * sChappieEars;
BlueLink * sBlueLink;
DemoMode *sDemoMode = 0;
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

    // console
    initConsole();
    CONSOLE_LINE("Chappie Booting...");

    // i/os
    CONSOLE_LINE(" * init I/O");
    setupAndExplainPullup(LILY_PIN_BT_RECONFIG, "BT reconfigure (boot)");
    setupAndExplainPullup(LILY_PIN_SIT_MODE, "SIT mode");
    setupAndExplainPullup(LILY_PIN_STANDUP_MODE, "StandUp mode");
    setupAndExplainPullup(LILY_PIN_DEMO_MODE, "DEMO mode");
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

    // explain...
    CONSOLE_LINE("Chappie Ready!");
    digitalWrite(LILY_PIN_LED, LOW);
}



void loop() {
    // uncomment the following 2 lines to enable a direct talk with the BT modem
    //crossStreams(&Serial, _Console);
    //return;

    if (digitalRead(LILY_PIN_SIT_MODE) == LOW) {
        // [SIT] mode
        sChappieEars->setLeftEar(-1);
        sChappieEars->setRightEar(-1);
    } if (digitalRead(LILY_PIN_STANDUP_MODE) == LOW) {
        // [STANDUP] mode
        sChappieEars->setLeftEar(1);
        sChappieEars->setRightEar(1);
    } else if (sDemoMode->getEnabled()) {
        // [DEMO] mode (enabled with jumper at boot time, or via software)
        sDemoMode->run();
    } else {
        // Analog Input Mode
        // TODO ...
    }

    // vital motion update
#ifdef EARS_MAX_SPEED
    sChappieEars->loop();
#endif

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
        } return true;

    // C: 04 -> set flag (target: flag index) (flag specific behavior)
    case 0x04:
        switch (rTarget) {
        case 1: sDemoMode->setSoftEnablement(rValue1 != 0); return true;
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
