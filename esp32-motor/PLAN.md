# Prosjektplan: ESP32-S3 motorstyring og Bluetooth

Denne filen beskriver maskinvaren, hvordan den kobles, hva `control.cpp` gjør,
og veien videre mot styring over Bluetooth.

---

## 1. Maskinvare (oversikt)

| Komponent | Detalj |
|---|---|
| Mikrokontroller | YD-ESP32-23 (2022-V1.3), ESP32-S3-N8R2 (8 MB flash, 2 MB PSRAM) |
| Motorer | 3× liten børstet DC-motor (RF-300/370), 3–6 V |
| Motordrivere | IRLZ44N logic-level MOSFET (én per motor), reserve: 2N2222 / S8050 / BC337 |
| Gate-/basemotstand | 500 Ω (MOSFET), 1 kΩ (BJT-reserve) |
| Dioder | 1N4007 eller 1N5819, én over hver motor (stripe mot batteri +) |
| Strøm til motorer | Felles batteri, f.eks. 4×AA ≈ 6 V |
| Knapper | 4 stk. normalt lukkede (NC) trykknapper i serie |

Pinne-bruk i nåværende kode:

| Funksjon | GPIO | Merknad |
|---|---|---|
| Motor 1 | 4 | PWM via 500 Ω til gate |
| Motor 2 | 5 | PWM via 500 Ω til gate |
| Motor 3 | 6 | PWM via 500 Ω til gate |
| Knappekjede | 1 | Intern pull-up, LOW = kjede lukket |
| Ledige | 2, 41, 42 | Kan brukes til ekstra knapper eller statuslampe |

---

## 2. Kobling

### 2.1 Per motor (MOSFET-versjon, low-side)

```
  Batteri +  ──────┬─────────────┐
                   │             │
                 [Diode]      Motor (+)
                 (stripe      
                  mot +)      Motor (–)
                   │             │
                   └─────────────┤
                                 │ Drain (D)
   GPIO ──[500 Ω]──── Gate (G)   │
                              IRLZ44N
                                 │ Source (S)
                                GND (felles)

   Gate-til-Source: 10 kΩ pulldown (holder MOSFET av ved oppstart, GPIO er flytende)
```

Pinne-rekkefølge IRLZ44N (tekst mot deg, bein ned): **G – D – S**, metallfliken = D.

Gjør dette for alle tre motorer. Motorene deler GND, men ikke + fra ESP32.

### 2.2 Alternativ med BJT (reserve)

Samme som over, men:
- Base via 1 kΩ fra GPIO (ikke 500 Ω til gate).
- Gate-pulldown erstattes av 10 kΩ base–emitter.
- Kollektor → motor (–), emitter → GND.
- 2N2222 / S8050 / BC337 tåler motorstrøm på noen hundre mA. BC547 og 2N3904 er for svake.

### 2.3 Knappekjede

```
  GPIO 1 ──[NC]──[NC]──[NC]──[NC]── GND
```

- Ingen knapp trykket: kjeden er lukket → GPIO leser LOW → **OK**.
- Én knapp trykket: kjeden brytes → GPIO leser HIGH → **nødstopp**.

### 2.4 Fellesjord (viktig)

ESP32-GND og batteri-GND **må være koblet sammen**. Ellers virker ikke PWM-styringen av MOSFET-ene.

### 2.5 Tilkobling til WSL (hver gang kortet kobles til)

Kortet vises ikke i WSL av seg selv. USB-porten må sendes inn fra Windows hver gang kortet kobles til eller fra, eller etter at det har startet på nytt. Gjør dette før `pio run -t upload`.

**I PowerShell på Windows (som administrator):**

```powershell
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

- `bind` trengs bare første gang for en gitt USB-port. Hvis `STATE` allerede er `Shared`, hopp over den.
- `attach` må gjøres hver gang kortet kobles til på nytt.
- BUSID kan endre seg mellom porter og etter RESET/BOOT. Kjør alltid `usbipd list` først.

**I WSL:**

```bash
ls /dev/ttyACM* 2>/dev/null
```

Hvis porten vises, er den klar. Oppdater `upload_port` og `monitor_port` i `platformio.ini` hvis navnet er et annet.

**Porter på kortet:**
- COM-porten (CH343, `1a86:55d3`): normalt `CDC_ON_BOOT=0`, opplasting uten BOOT/RESET.
- Native USB (`303a:4001` / `303a:1001`): krever `CDC_ON_BOOT=1` for Serial. Hvis opplasting feiler med `No serial data received`, hold BOOT, trykk RESET, slipp BOOT, og kjør opplastingen med én gang.

---

## 3. Hva `control.cpp` gjør

Programmet er delt i seksjoner (se kommentarene i filen):

1. **Konfigurasjon:** pinner, PWM-frekvens (20 kHz), rampe og debounce-tid.
2. **Motorer:** `Motor`-struct med `running` (ønsket på/av), `targetSpeed` (ønsket fart), `currentSpeed` (faktisk PWM etter rampe).
3. **Rampe:** farten endres i små steg for å unngå strømstøt. Ved nødstopp settes farten til 0 med en gang.
4. **Knappekjede:** rå avlesning med debounce. Brutt kjede → alle motorer av, og motorer kan ikke slås på før kjeden er lukket.
5. **Kommandoer:** tekst-kommandoer i `handleCommand()`, som er den felles inngangen for alle kommandokilder.
6. **Setup/loop:** `loop()` kjører tre ting hele tiden: lese kommandoer, sjekke knapper, oppdatere PWM.

### Designvalg du kan endre

| Valg | Nåværende | Alternativ |
|---|---|---|
| Knappekjede brutt | Stopper alle motorer | Kun varsel |
| Motor etter nødstopp | Må slås på manuelt | Gjenopptas automatisk |
| Knapper | Én kjede (vet ikke hvilken) | Én pinne per knapp (vet hvilken) |
| PWM-frekvens | 20 kHz | 1–5 kHz hvis motorene oppfører seg rart |

### Kjent begrensning: serie-kjede gir ikke hvilken knapp

Med én kjede ser programmet bare «noe er trykket». Hvis du må vite *hvilken* knapp,
kobl hver knapp til egen GPIO med NC til GND og `INPUT_PULLUP`. Da leser pinnen LOW når
knappen ikke er trykket, og HIGH når den er trykket. Funksjonen `updateButtons()` må da
utvides til én `bool` per knapp.

### Test uten Bluetooth

1. Åpne Serial Monitor på 115200 baud.
2. Skriv `HELP`, og deretter `M1 S100` og `M1 ON`.
3. Trykk en knapp: motoren skal stoppe, og meldingen om nødstopp vises.
4. Slipp knappen og skriv `M1 ON` på nytt.

Ting å sjekke før du kobler til motorene:
- Fungerer PWM med multimeter eller LED på gaten gjennom 500 Ω?
- Er MOSFET-ene riktig vendt (G–D–S)?
- Er dioden vendt riktig (stripe mot +)?

---

## 4. Plan: styring over Bluetooth

ESP32-S3 støtter Bluetooth Low Energy (BLE). Den enkleste veien er en **Nordic UART-tjeneste** (NUS)
via biblioteket **NimBLE-Arduino**. Da kan en mobil-app eller PC sende de samme tekstkommandoene
som i Serial Monitor.

### Fase 0 – Forberedelse (gjort)
- [x] Motorer og knapper testet over Serial.
- [x] Kommandoene er samlet i `handleCommand()`, som er den felles inngangen.

### Fase 1 – BLE-server med tekst-kommandoer
1. Installer **NimBLE-Arduino** i Arduino IDE eller PlatformIO.
2. Lag en BLE-server med tjenesten og karakteristikkene til Nordic UART:
   - Tjeneste UUID: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
   - RX (skriving fra klient): `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
   - TX (lesing/varsler til klient): `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`
3. I `onWrite`-callbacken: legg mottatt tekst i en kø (`String`), **ikke** kall motorfunksjoner direkte fra BLE-tråden.
4. I `loop()`: hent fra køen og kall `handleCommand()`. Samme kode som Serial.
5. Send svar (f.eks. `printStatus()`-output) tilbake via TX-karakteristikken.

Hvorfor kø: BLE-callbacken kjører i en egen oppgave. Ved å sende kommandoene via `loop()`
unngår du race-forhold mellom BLE og motorstyringen.

### Fase 2 – Pålitelighet og sikkerhet
- **Watchdog for BLE-tap:** hvis klienten kobler fra, stopp alle motorer (`stopAllMotors()` i `onDisconnect`).
- **Timeout:** motoren stopper hvis det ikke har kommet kommando på f.eks. 2 s mens den går.
- **Pariring:** legg til PIN eller bonding, slik at ikke hvem som helst kan styre robotten.
- **Nødstopp:** knappekjeden skal fortsatt stoppe motorene uavhengig av BLE. Den er den fysiske sikkerheten.

### Fase 3 – Strukturert protokoll (valgfritt)
Hvis tekstkommandoene blir for mange, gå over til en enkel binær eller JSON-protokoll:
- Eksempel: `{"m":1,"on":true,"s":128}`.
- Bruk ArduinoJson til parsing.
- `handleCommand()` tar da en struct i stedet for tekst. Resten av koden er uendret.

### Fase 4 – App eller nettside
- Enkel webside med Web Bluetooth (Chrome/Edge): knapper for hver motor og en glidebryter for fart.
- Alternativt: mobilapp med BLE-bibliotek (f.eks. nRF Connect for testing).

### Fase 5 – Utvidelser
- Status tilbake til klient: knappetilstand, motorstrøm (ved å måle spenning over shunt) og temperatur.
- Flere motorer: utvid `NUM_MOTORS` og `MOTOR_PINS`.
- Retningsstyring: krever H-bro (f.eks. DRV8833). IRLZ44N-oppsettet støtter kun én retning.
- OTA-oppdatering via WiFi, siden kortet har 8 MB flash.

---

## 5. Sjekkliste før BLE

- [ ] `control.cpp` kompilerer uten feil (ESP32 Arduino core 3.x).
- [ ] Alle tre motorer kan styres hver for seg over Serial.
- [ ] Nødstopp via knappekjeden fungerer for alle fire knappene.
- [ ] Hver MOSFET har 10 kΩ pulldown gate–source.
- [ ] Felles GND mellom ESP32 og batteri.
- [ ] Dioder over hver motor er montert riktig.
- [ ] Bestem om knappene skal være én kjede eller separate pinner.
