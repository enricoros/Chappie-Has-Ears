// Copyright Enrico Ros 2016. All rights reserved.

#include <Arduino.h>
#include <SoftwareSerial.h>

#define LILY_LED                LED_BUILTIN // D13
#define LILY_HW_SERIAL_SPEED    57600

#define BT_ENABLE
#define BT_RECONFIG_JUMPER_1    6
#define BT_RECONFIG_JUMPER_2    7

#define SERVO_ENABLE
#define SERVO_CONTROL_LEFT      10
#define SERVO_CONTROL_RIGHT     11
#define SERVO_CONTROL_REFGND    9


static void initOutput(int pin, int level) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, level);
}

static void configureJumper(int pin, int pinGnd) {
    pinMode(pin, INPUT);
    digitalWrite(pin, HIGH);
    initOutput(pinGnd, LOW);
}

static bool hasJumper(int pin, int /*pinGnd*/) {
    return digitalRead(pin) == LOW;
}


#ifdef BT_ENABLE
/**
 * @brief The BlueLink class communicates via a Blueetooth serial.
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
    Stream *m_ptSerial;
public:
    BlueLink(HardwareSerial *btSerial, unsigned long blueModemSpeed = 115200, Stream *passthroughSerial = 0)
        : m_btSerial(btSerial)
        , m_ptSerial(passthroughSerial)
    {
        // use a jumper to know whether to reflash the config or not
        configureJumper(BT_RECONFIG_JUMPER_1, BT_RECONFIG_JUMPER_2);

        // init communication to the module
        m_btSerial->begin(blueModemSpeed);
        delay(100);
    }

    bool requestedReconfigure() const {
        return hasJumper(BT_RECONFIG_JUMPER_1, BT_RECONFIG_JUMPER_2);
    }

    bool reConfigureModem() const {
        // enter command mode
        if (!writeAndConfirm("$$$", "CMD\r\n", 5, 1000))
            return false;

        // factory config
        if (!writeAndConfirm("SF,1\n", "AOK\r\n", 5, 2000))
            return false;

        // rename device
        if (!writeAndConfirm("SN,Chappie-Ears\n", "AOK\r\n", 5, 1000))
            return false;

        // turn config timer off
        if (!writeAndConfirm("ST,0\n", "AOK\r\n", 5, 1000))
            return false;

        // set pin to 1337
        if (!writeAndConfirm("SP,1337\n", "AOK\r\n", 5, 1000))
            return false;
        return true;
    }

    bool readPacket(byte *destBuffer, int maxLength) {
        // we need maxLength + 1 sync byte
        while (m_btSerial->available() >= (maxLength + 1)) {
            // validate the sync byte
            byte syncByte = (byte)m_btSerial->read();
            if (syncByte != 0xA5) {
                // ERROR: we were unsynced
                if (m_ptSerial) {
                    m_ptSerial->print("e21: ");
                    m_ptSerial->println((int)syncByte);
                }
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

    void loopPassthrough() {
        while (m_btSerial->available()) {
            char c = (char)m_btSerial->read();
            if (m_ptSerial) {
                //m_ptSerial->println(c);
                m_ptSerial->print(c);
            }
        }
        while (m_ptSerial && m_ptSerial->available()) {
            m_btSerial->print((char)m_ptSerial->read());
            //m_ptSerial->print(">B\n");
        }
    }

private:
    bool writeAndConfirm(const char *msg, const char *expected, int length, int timeoutMs) const {
        m_btSerial->print(msg);
        bool ok = matchReply(expected, length, timeoutMs);
        if (!ok && m_ptSerial) {
            m_ptSerial->print("BlueLink: exception writing: '");
            m_ptSerial->print(msg);
            m_ptSerial->print("'\n");
        }
        return ok;
    }

    bool matchReply(const char *expected, int length, int timeoutMs) const {
        int sIdx = 0;
        while (timeoutMs > 0) {
            while (m_btSerial->available()) {
                char c = (char)m_btSerial->read();
                if (expected[sIdx] != c) {
                    if (m_ptSerial) {
                        m_ptSerial->print("BlueLink: expecting '");
                        m_ptSerial->print((int)expected[sIdx]);
                        m_ptSerial->print("' gotten '");
                        m_ptSerial->print((int)c);
                        m_ptSerial->print("'\n");
                    }
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
BlueLink *sBlueLink = 0;
#endif

#ifdef SERVO_ENABLE
#include <Servo.h>
/**
 * This class encapsulates the motor control of Chappie
 */
class ChappieEars
{
    Servo m_sLeftEar;
    Servo m_sRightEar;
    float m_pLeftEar;
    float m_pRightEar;
    float m_multiplier;
    int normalToServo(float n) const {
        const int s = round(n * 90.0f);
        if (s <= -90)
            return 0;
        if (s >= 90)
            return 180;
        return s + 90;
    }
public:
    ChappieEars()
        : m_pLeftEar(0)
        , m_pRightEar(0)
        , m_multiplier(1.0)
    {
    }

    /// Connect to the hardware pins. Call once
    void init(int pin_leftEar, int pin_rightEar, int pin_gnd) {
        // set the ground pin to LOW
        initOutput(pin_gnd, LOW);

        // attach
        m_sLeftEar.attach(pin_leftEar);
        m_sRightEar.attach(pin_rightEar);

        // write initial values
        setLeftEar(m_pLeftEar);
        setRightEar(m_pRightEar);
    }

    /* limits (active when setting the next setpoint) */

    void setMultiplier(float m) { m_multiplier = constrain(m, 0.01, 2.0); }
    float getMultiplier() const { return m_multiplier; }

    /* set points (all between -1 ... 1) */

    void setLeftEar(float np) {
        if (np != m_pLeftEar) {
            m_pLeftEar = np;
            if (m_multiplier != 1.0)
                np *= m_multiplier;
            m_sLeftEar.write(normalToServo(-np));
        }
    }
    float getLeftEar() const { return m_pLeftEar; }

    void setRightEar(float np) {
        if (np != m_pRightEar) {
            m_pRightEar = np;
            if (m_multiplier != 1.0)
                np *= m_multiplier;
            m_sRightEar.write(normalToServo(np));
        }
    }
    float getRightEar() const { return m_pRightEar; }
};
ChappieEars *sChappieEars = 0;
#endif

#define LILY_SW_CONSOLE_RX      10  // D10
#define LILY_SW_CONSOLE_TX      11  // D11
#define LILY_SW_CONSOLE_SPEED   57600

// the software console (since the HW serial is busy with bluetooth)
Stream *serialConsole;

void setup() {
    // high during setup
    initOutput(LILY_LED, HIGH);

    // serial console: hw serial vs software serial
#if 0
    SoftwareSerial *swConsole = new SoftwareSerial(LILY_SW_CONSOLE_RX, LILY_SW_CONSOLE_TX);
    swConsole->begin(LILY_SW_CONSOLE_SPEED);
    serialConsole = swConsole;
#else
    Serial.begin(LILY_HW_SERIAL_SPEED);
    serialConsole = &Serial;
#endif
    serialConsole->println("Chappie Booting...");

    // init Bluetooth (reconfigure module if jumper is present)
#ifdef BT_ENABLE
    serialConsole->println(" * init BT");
    sBlueLink = new BlueLink(&Serial, LILY_HW_SERIAL_SPEED, serialConsole);
    if (sBlueLink->requestedReconfigure()) {
        serialConsole->println("   * reconfiguring BT");
        sBlueLink->reConfigureModem();
    }
#endif

    // init ears
#ifdef SERVO_ENABLE
    serialConsole->println(" * init Ears");
    sChappieEars = new ChappieEars();
    sChappieEars->init(SERVO_CONTROL_LEFT, SERVO_CONTROL_RIGHT, SERVO_CONTROL_REFGND);
#endif

    // explain...
    serialConsole->println("Chappie Ready!");
    digitalWrite(LILY_LED, LOW);
}

enum Command {
    Cmd_None    = 0,
    Cmd_Up      = 1,
    Cmd_Down    = 2,
    Cmd_Mid     = 3,
    Cmd_Left    = 4,
    Cmd_Right   = 5
};
int readCommandFromConsole(Stream *serialConsole) {
    while (serialConsole->available()) {
        char c = (char)serialConsole->read();

        // empty buffer
#if 0
        delay(100);
        while (serialConsole->available())
            serialConsole->read();
#endif

        // serialConsole->println((int)c);
        switch (c) {
        case 'u': case 'U':
            return Cmd_Up;
        case 'm': case 'M':
            return Cmd_Mid;
        case 'd': case 'D':
            return Cmd_Down;
        case 'l': case 'L':
            return Cmd_Left;
        case 'r': case 'R':
            return Cmd_Right;
        }
    }
    return Cmd_None;
}

void loop() {
    Command c = (Command)readCommandFromConsole(serialConsole);
    if (c == Cmd_None)
        return;

#ifdef SERVO_ENABLE
    switch (c) {
    case Cmd_Up:
        sChappieEars->setLeftEar(1);
        sChappieEars->setRightEar(1);
        break;
    case Cmd_Down:
        sChappieEars->setLeftEar(-1);
        sChappieEars->setRightEar(-1);
        break;
    case Cmd_Mid:
        sChappieEars->setLeftEar(0);
        sChappieEars->setRightEar(0);
        break;
    case Cmd_Left:
        sChappieEars->setLeftEar(1);
        sChappieEars->setRightEar(-0.5);
        break;
    case Cmd_Right:
        sChappieEars->setLeftEar(-0.5);
        sChappieEars->setRightEar(1);
        break;
    }
#endif

    digitalWrite(LILY_LED, HIGH);
    delay(300);
    digitalWrite(LILY_LED, LOW);
}

