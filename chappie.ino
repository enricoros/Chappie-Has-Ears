// Copyright Enrico Ros 2016. All rights reserved.

#include <Arduino.h>

// uncomment the following for talking directly to the BT module (after initialization)
//#define DEBUG_SWCONSOLE_2_BT_PASSTHROUGH

// uncomment the following to give commands from the Console: U,D,M,L,R
#define DEBUG_SWCONSOLE_2_SERVOS


// misc constants
#define LILY_PIN_LED            LED_BUILTIN // D13
#define LILY_HW_SERIAL_SPEED    57600   // fixed, can't go beyond it (errors)

// additional console
#include <SoftwareSerial.h>
#define LILY_SW_CONSOLE_SPEED   57600
#define LILY_PIN_SW_CONSOLE_RX  2  // D2
#define LILY_PIN_SW_CONSOLE_TX  3  // D3

// BT pins
#define BT_ENABLE
#define LILY_PIN_BT_RECONFIG    4 // set to GND to reconfig

// Servo(s) pins
#define SERVO_ENABLE
#define LILY_PIN_SERVO_GND      9
#define LILY_PIN_SERVO_L_EAR    10
#define LILY_PIN_SERVO_R_EAR    11



static void initOutput(int pin, int level) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, level);
}

/*static void configureJumper(int pin, int pinGnd) {
    pinMode(pin, INPUT);
    digitalWrite(pin, HIGH);
    initOutput(pinGnd, LOW);
}

static bool hasJumper(int pin, int pinGnd_unused) {
    return digitalRead(pin) == LOW;
}*/

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
    Stream *m_console;
#define SAFE(x) if (x) x
#define CONSOLE SAFE(m_console)
public:
    BlueLink(HardwareSerial *btSerial, Stream *console = 0)
        : m_btSerial(btSerial)
        , m_console(console)
    {
    }

    void init() const {
        pinMode(LILY_PIN_BT_RECONFIG, INPUT_PULLUP);
        delay(50);

        // check if the user requested to reconfigure
        if (digitalRead(LILY_PIN_BT_RECONFIG) == LOW)
            initReconfigure();
        else
            initNormal();
    }

    int readChars() {
        int hasChars = m_btSerial->available();
        if (!hasChars)
            return 0;
        CONSOLE->print("BT bytes < ");
        while (m_btSerial->available() >= 1) {
            CONSOLE->print((byte)m_btSerial->read());
            CONSOLE->print(" ");
        }
        CONSOLE->println(">");
        return hasChars;
    }

    bool unused_readPacket5(byte *destBuffer, int maxLength) {
        // we need maxLength + 1 sync byte
        while (m_btSerial->available() >= (maxLength + 1)) {
            // validate the sync byte
            byte syncByte = (byte)m_btSerial->read();
            if (syncByte != 0xA5) {
                // ERROR: we were unsynced
                if (m_console) {
                    m_console->print("e21: ");
                    m_console->println((int)syncByte);
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

private:
    void initNormal() const {
        CONSOLE->print("   * open BT... ");
        m_btSerial->begin(LILY_HW_SERIAL_SPEED, SERIAL_8N1);
        delay(100);
        CONSOLE->println("done.");
    }

    /* Reconf Manually with:
     *  $$$ SF,1\n SN,Chappie-Ears\n SP,1337\n ST,0\n SU,57.6\n ---\n
     */
    bool initReconfigure() const {
        CONSOLE->println("   * reconfiguraton requested ");

        // jump up to the modem speed and move it down temporarily to 57600
        if (true /*lowerBtSpeed*/) {
            // start with the modem speed
            CONSOLE->print("   * lowering BT modem speed to 57600.. ");
            m_btSerial->begin(115200);
            delay(100);
            m_btSerial->print("$");
            m_btSerial->print("$");
            m_btSerial->print("$");
            delay(100);
            // move the modem to 57600 and in data mode
            m_btSerial->println("U,57.6,N");
            CONSOLE->println("done");
        }

        // now open at the right speed
        initNormal();

        CONSOLE->print("   * reconfiguring BT... ");

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

        // go back to data mode
        if (!writeAndConfirm("---\n", "END\r\n", 5, 1000))
            return false;

        CONSOLE->println("done.");
        return true;
    }

    bool writeAndConfirm(const char *msg, const char *expected, int length, int timeoutMs) const {
        m_btSerial->print(msg);
        bool ok = (expected == 0) || matchReply(expected, length, timeoutMs);
        if (m_console && !ok) {
            m_console->print(ok ? "   * BlueLink: wrote: '" : "   * BlueLink: exception writing: '");
            m_console->println(msg);
        }
        return ok;
    }

    bool matchReply(const char *expected, int length, int timeoutMs) const {
        int sIdx = 0;
        while (timeoutMs > 0) {
            while (m_btSerial->available()) {
                char c = (char)m_btSerial->read();
                if (expected[sIdx] != c) {
                    if (m_console) {
                        m_console->print("   * BlueLink: expected ");
                        m_console->print((int)expected[sIdx]);
                        m_console->print(" gotten ");
                        m_console->print((int)c);
                        m_console->print(" at index ");
                        m_console->println((int)sIdx);
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


// the software console (since the HW serial is busy with bluetooth)
Stream *sConsole = 0;

void setup() {
    // high during setup
    initOutput(LILY_PIN_LED, HIGH);

    // serial console: hw serial vs software serial
    bool useSwConsole = true;
    if (useSwConsole) {
        SoftwareSerial *swConsole = new SoftwareSerial(LILY_PIN_SW_CONSOLE_RX, LILY_PIN_SW_CONSOLE_TX);
        swConsole->begin(LILY_SW_CONSOLE_SPEED);
        sConsole = swConsole;
    } else {
        Serial.begin(LILY_HW_SERIAL_SPEED);
        sConsole = &Serial;
    }
    sConsole->println("Chappie Booting...");

    // init Bluetooth
    sConsole->println(" * init BT");
    sBlueLink = new BlueLink(&Serial, sConsole);
    sBlueLink->init();

    // init ears
#ifdef SERVO_ENABLE
    sConsole->println(" * init Ears");
    sChappieEars = new ChappieEars();
    sChappieEars->init(LILY_PIN_SERVO_L_EAR, LILY_PIN_SERVO_R_EAR, LILY_PIN_SERVO_GND);
#endif

    // explain...
    sConsole->println("Chappie Ready!");
    digitalWrite(LILY_PIN_LED, LOW);
}

enum Command {
    Cmd_None    = 0,
    Cmd_Up      = 1,
    Cmd_Down    = 2,
    Cmd_Mid     = 3,
    Cmd_Left    = 4,
    Cmd_Right   = 5
};

#ifdef DEBUG_SWCONSOLE_2_SERVOS
int readConsoleCommand(Stream *console) {
    while (console->available()) {
        char c = (char)console->read();

        // empty buffer
        delay(100);
        while (console->available())
            console->read();

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
#endif


void loop() {

#ifdef DEBUG_SWCONSOLE_2_BT_PASSTHROUGH
    crossStreams(&Serial, sConsole);
    return;
#endif

    Command command = Cmd_None;

#ifdef DEBUG_SWCONSOLE_2_SERVOS
    command = (Command)readConsoleCommand(sConsole);
#endif

    // check the BT serial for new commands
    if (sBlueLink) {
        //sBlueLink->unused_readPacket5();
        sBlueLink->readChars();
    }

    if (command == Cmd_None)
        return;

    sConsole->print("Chappie Command ");
    sConsole->println((int)command);

#ifdef SERVO_ENABLE
    switch (command) {
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

    digitalWrite(LILY_PIN_LED, HIGH);
    delay(300);
    digitalWrite(LILY_PIN_LED, LOW);
}

