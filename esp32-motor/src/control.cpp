/*
 * ============================================================================
 *  control.cpp - Styring av 3 DC-motorer og knappekjede på ESP32-S3-N8R2
 * ============================================================================
 *
 *  Kort: YD-ESP32-23 (ESP32-S3-N8R2)
 *  Framework: Arduino (ESP32 Arduino core 3.x, f.eks. i Arduino IDE / PlatformIO)
 *  Programmering: via «COM»-porten (USB-UART).
 *
 *  Hva programmet gjør
 *  -------------------
 *  1. Styrer 3 børstede DC-motorer, hver for seg: på/av + fart (PWM, én retning).
 *     Hver motor styres av en IRLZ44N-MOSFET (low-side: motor mellom + og drain).
 *  2. Leser en kjede av 4 normalt lukkede (NC) knapper koblet i serie.
 *     Kjeden er en sikkerhetsfunksjon: Hvis en knapp trykkes, brytes kjeden,
 *     og ALLE motorer stoppes umiddelbart (nødstopp-logikk).
 *  3. Tar imot kommandoer som tekst over Serial (USB-UART). Samme tekst-
 *     kommandoer kan senere sendes over Bluetooth - se PLAN.md.
 *
 *  Kommandoer (skriv i Serial Monitor, 115200 baud, avslutt med Enter):
 *    HELP            - vis kommandoliste
 *    STATUS          - vis tilstanden til alle motorer og knappekjeden
 *    ALL OFF         - stopp alle motorer
 *    M1 ON           - slå på motor 1 (bruker sist satte fart)
 *    M1 OFF          - slå av motor 1
 *    M1 S128         - sett fart på motor 1 (0-255), slår ikke på motoren
 *    (M2/M3 fungerer likt)
 *
 *  Viktige innstillinger i Arduino IDE for ESP32-S3:
 *    - USB CDC On Boot: Disabled   (slik at Serial går til COM-porten)
 *    - Flash Size: 8MB, PSRAM: OPI PSRAM (kortet har 2 MB QSPI PSRAM)
 * ============================================================================
 */

#include <Arduino.h>

// ----------------------------------------------------------------------------
//  1. KONFIGURASJON - endre disse hvis kobling endres
// ----------------------------------------------------------------------------

constexpr uint8_t NUM_MOTORS = 3;

// GPIO for motor 1, 2, 3 (gate på MOSFET via 500 Ω motstand).
// Unngå 0, 3, 45, 46 (strapping), 19/20 (USB), 43/44 (UART), 48 (RGB-LED).
constexpr uint8_t MOTOR_PINS[NUM_MOTORS] = {4, 5, 6};

// GPIO for knappekjeden (alle 4 NC-knapper i serie mellom denne pinnen og GND).
constexpr uint8_t BUTTON_CHAIN_PIN = 1;

// PWM-innstillinger
constexpr uint32_t PWM_FREQ_HZ    = 20000;  // 20 kHz: over hørbart område, ingen hvin
constexpr uint8_t  PWM_RESOLUTION = 8;      // 8 bit -> duty 0..255

// Myk start/stopp (rampe): begrenser strømstøt når motoren får ny fart.
constexpr uint8_t  RAMP_STEP        = 5;    // hvor mye farten endres per steg
constexpr uint32_t RAMP_INTERVAL_MS = 20;   // tid mellom hvert steg

// Debounce for knappekjeden: ignorer kortvarige spiker (kontaktstøy).
constexpr uint32_t DEBOUNCE_MS = 30;

constexpr uint32_t SERIAL_BAUD = 115200;

// ----------------------------------------------------------------------------
//  2. DATASTRUKTURER OG TILSTAND
// ----------------------------------------------------------------------------

struct Motor {
    uint8_t pin;          // GPIO som driver MOSFET-gaten
    bool    running;      // ønsket tilstand: skal motoren gå?
    uint8_t targetSpeed;  // ønsket fart (0..255) - beholdes selv når motoren er av
    uint8_t currentSpeed; // faktisk fart som sendes til PWM (etter rampe)
};

Motor motors[NUM_MOTORS];

// Knappekjede-tilstand
bool     chainClosed   = true;   // true = kjeden er lukket (ingen knapp trykket)
bool     lastRawChain  = true;   // siste rå avlesning (før debounce)
uint32_t lastChangeMs  = 0;      // tidspunkt for siste endring i rå avlesning

// ----------------------------------------------------------------------------
//  3. HJELPEFUNKSJONER FOR MOTORER
// ----------------------------------------------------------------------------

// Stopper alle motorer (ønsket tilstand settes til AV).
// Fart-verdien (targetSpeed) beholdes slik at brukeren kan slå på igjen med samme fart.
void stopAllMotors() {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].running = false;
    }
}

// Setter faktisk PWM til hver motor. Kalles fast fra loop().
// Rampen flytter currentSpeed mot ønsket verdi i små steg.
// Hvis knappekjeden er brutt, settes farten direkte til 0 (nødstopp, ingen rampe).
void updateMotors() {
    static uint32_t lastRampMs = 0;
    uint32_t now = millis();
    bool rampStep = (now - lastRampMs) >= RAMP_INTERVAL_MS;
    if (rampStep) lastRampMs = now;

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        Motor &m = motors[i];

        // Ønsket fart: 0 hvis motoren er av ELLER knappekjeden er brutt
        uint8_t want = (m.running && chainClosed) ? m.targetSpeed : 0;

        if (!chainClosed) {
            m.currentSpeed = 0;                     // nødstopp: umiddelbart
        } else if (rampStep) {
            if (m.currentSpeed < want) {
                uint16_t next = m.currentSpeed + RAMP_STEP;
                m.currentSpeed = (next > want) ? want : next;
            } else if (m.currentSpeed > want) {
                int16_t next = (int16_t)m.currentSpeed - RAMP_STEP;
                m.currentSpeed = (next < want) ? want : (uint8_t)next;
            }
        }

        ledcWrite(m.pin, m.currentSpeed);           // sett PWM-duty
    }
}

// ----------------------------------------------------------------------------
//  4. KNAPPEKJEDE
// ----------------------------------------------------------------------------
//  NC-knapper i serie med INPUT_PULLUP:
//    - Alle knapper ikke trykket  -> kjeden lukket -> pinne = LOW
//    - En knapp trykket           -> kjeden brutt  -> pinne = HIGH (intern pull-up)
//  Det vil si: LOW = OK, HIGH = noe er trykket.
//
//  OBS: Med én kjede vet vi at NOE er trykket, ikke HVILKEN knapp.
//  Se PLAN.md for alternativ med én pinne per knapp.

void setupButtons() {
    pinMode(BUTTON_CHAIN_PIN, INPUT_PULLUP);
}

void updateButtons() {
    bool raw = (digitalRead(BUTTON_CHAIN_PIN) == LOW);   // true = lukket
    uint32_t now = millis();

    // Start timer når rå verdi endrer seg
    if (raw != lastRawChain) {
        lastRawChain = raw;
        lastChangeMs = now;
    }

    // Godta ny tilstand først når den har vært stabil i DEBOUNCE_MS
    if (raw != chainClosed && (now - lastChangeMs) >= DEBOUNCE_MS) {
        chainClosed = raw;
        if (!chainClosed) {
            stopAllMotors();
            Serial.println("! KNAPP TRYKKET - alle motorer stoppet (nødstopp)");
        } else {
            Serial.println("Knappekjede lukket igjen. Motorer må slås på manuelt.");
        }
    }
}

// ----------------------------------------------------------------------------
//  5. KOMMANDOER (tekst)
// ----------------------------------------------------------------------------
//  handleCommand() er felles inngang for ALLE kommandokilder.
//  Når Bluetooth legges til, kalles den samme funksjonen fra BLE-callbacken.

void printHelp() {
    Serial.println();
    Serial.println("Kommandoer:");
    Serial.println("  HELP            - vis denne listen");
    Serial.println("  STATUS          - vis tilstand");
    Serial.println("  ALL OFF         - stopp alle motorer");
    Serial.println("  M<1-3> ON       - slå på motor");
    Serial.println("  M<1-3> OFF      - slå av motor");
    Serial.println("  M<1-3> S<0-255> - sett fart (slår ikke på motoren)");
    Serial.println();
}

void printStatus() {
    Serial.printf("Knappekjede: %s\n", chainClosed ? "lukket (OK)" : "BRUTT (nødstopp)");
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        Serial.printf("  M%d: %-4s mål=%3d faktisk=%3d\n",
                      i + 1,
                      motors[i].running ? "PÅ" : "AV",
                      motors[i].targetSpeed,
                      motors[i].currentSpeed);
    }
}

// Behandler kommandoer av formen "M<n> ON|OFF|S<verdi>"
void handleMotorCommand(const String &cmd) {
    int space = cmd.indexOf(' ');
    if (space < 2) {
        Serial.println("Feil: bruk f.eks. M1 ON, M1 OFF, M1 S128");
        return;
    }

    int idx = cmd.substring(1, space).toInt() - 1;   // "M1" -> 0
    if (idx < 0 || idx >= NUM_MOTORS) {
        Serial.println("Feil: motornummer må være 1-3");
        return;
    }

    String arg = cmd.substring(space + 1);
    arg.trim();
    Motor &m = motors[idx];

    if (arg == "ON") {
        if (!chainClosed) {
            Serial.println("Feil: knappekjede er brutt, kan ikke starte motor");
            return;
        }
        m.running = true;
    } else if (arg == "OFF") {
        m.running = false;
    } else if (arg.startsWith("S")) {
        int value = arg.substring(1).toInt();
        if (value < 0 || value > 255) {
            Serial.println("Feil: fart må være 0-255");
            return;
        }
        m.targetSpeed = (uint8_t)value;
    } else {
        Serial.println("Feil: ukjent motorkommando");
        return;
    }
    printStatus();
}

void handleCommand(String cmd) {
    cmd.trim();
    cmd.toUpperCase();
    if (cmd.length() == 0) return;

    if (cmd == "HELP") {
        printHelp();
    } else if (cmd == "STATUS") {
        printStatus();
    } else if (cmd == "ALL OFF") {
        stopAllMotors();
        printStatus();
    } else if (cmd.charAt(0) == 'M') {
        handleMotorCommand(cmd);
    } else {
        Serial.println("Ukjent kommando. Skriv HELP.");
    }
}

// Leser tegn fra Serial og bygger opp én linje om gangen
void readSerialCommands() {
    static String buffer;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            handleCommand(buffer);
            buffer = "";
        } else if (c != '\r') {
            buffer += c;
        }
    }
}

// ----------------------------------------------------------------------------
//  6. SETUP OG LOOP
// ----------------------------------------------------------------------------

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);   // gi USB-UART tid til å koble til

    setupButtons();

    // Sett opp PWM-kanal for hver motor. Gaten holdes lav (0 duty) fra start,
    // slik at motorene ikke starter ved oppstart.
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].pin          = MOTOR_PINS[i];
        motors[i].running      = false;
        motors[i].targetSpeed  = 0;
        motors[i].currentSpeed = 0;

        ledcAttach(MOTOR_PINS[i], PWM_FREQ_HZ, PWM_RESOLUTION);
        ledcWrite(MOTOR_PINS[i], 0);
    }

    Serial.println();
    Serial.println("=== Motorstyring klar ===");
    printHelp();
    printStatus();
}

void loop() {
    readSerialCommands();   // kommandoer fra Serial (senere også Bluetooth)
    updateButtons();        // sjekk knappekjeden (nødstopp)
    updateMotors();         // oppdater PWM med rampe
}
