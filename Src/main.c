/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// I2C adresa PCF8574T shiftana u lijevo za 1 jer I2C treba 7-bit adresiranje
#define PCF8574T_ADDRESS  0x27 << 1 // 0x4E

// LCD
#define LCD_BACKLIGHT 0x08      // Upravlja LCD pozadinskim svjetlom (0x08 = ON)
#define LCD_ENABLE    0x04      // Omogućuje pin za LCD operacije (slanje podatka ili komandi)
#define LCD_RS        0x01      // Odabir registara, 0 = LCD ceka naredbu, 1 = ceka podatke za prikaz

// RFID Komande
#define PCD_IDLE        0x00
#define PCD_AUTHENT     0x0E
#define PCD_RECEIVE     0x08
#define PCD_TRANSMIT    0x04
#define PCD_TRANSCEIVE  0x0C    // Prenosi i prima podatke, komunikacija sa karticom
#define PCD_RESETPHASE  0x0F    // soft reset MFRC522

// RFID Registri
#define CommandReg        0x01  // Zadaje RFID-u koju komandu da izvrši
#define CommIEnReg        0x02  // Omogućuje prekid kada se nešto dogodi
#define CommIrqReg        0x04  // Pokazuje je li neka operacija završena
#define DivIrqReg        0x05   // nadzire ostale uvjete prekida
#define ErrorReg         0x06   // detektira greške u komunikaciji
#define FIFODataReg      0x09   // FIFODataReg je registar kroz koji se šalju i primaju podaci
#define FIFOLevelReg     0x0A   // Koliko podataka je trenutno u FIFO memoriji.
#define WaterLevelReg    0x0B   // Postavlja granicu koliko podataka smije biti u FIFO bufferu
#define ControlReg       0x0C   // Sadrži kontrolne bitove
#define BitFramingReg    0x0D   // Koliko bita treba poslati
#define CollReg          0x0E   // Provjerava je li bilo sudara podataka prilikom komunikacije sa više kartica
#define ModeReg          0x11   // Postavlja osnovni radni način RFID čitača.
#define TxControlReg     0x14   // Pali antenu, bez ovog RFID ne šalje ništa
#define TxASKReg         0x15   // Amplitude Shift Keying je metoda koju čitač koristi za slanje podataka kartici. Ovaj registar određuje kako se točno oblikuje izlazni signal.
// Podešavaju tajmer u RFID čipu.
#define TModeReg         0x2A
#define TPrescalerReg    0x2B   // Timer prescaler settings, određuje brzinu brojanja
#define TReloadRegH      0x2C   // gornji bajt tajmera
#define TReloadRegL      0x2D   // donji bajt tajmera

// RFID PICC Komande
#define PICC_ANTICOLL    0x93   // Čita UID i sprječava da više kartica zbune čitač
// Prva komanda koju RFID čitač šalje nakon što upali antenu
#define PICC_REQIDL      0x26   // Detekcija RFID kartice u mirovanju

#define MFRC522_SPI      &hspi2
#define RFID_RST_PIN     GPIO_PIN_8 // reset
#define RFID_RST_PORT    GPIOA
// NSS je signalna linija koja se koristi za omogućavanje ili onemogućavanje komunikacije s određenim SLAVE uređajem
#define RFID_NSS_PIN     GPIO_PIN_4 // čip selecet, aktivno kad je low
#define RFID_NSS_PORT    GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//========================//
//   KARTICE STUDENATA    //
//========================//
const uint8_t Student1[4] = {0x16, 0xFC, 0xDE, 0x12};
const uint8_t Student2[4] = {0x94, 0x24, 0x09, 0x1E};
const uint8_t Student3[4] = {0x67, 0x64, 0xE8, 0x2B};
//const uint8_t Student4[4] = {0xEA, 0x4C, 0xC0, 0x2E};

//=========================//
//   BROJAC PRISUTNOSTI    //
//=========================//
// Svaki student ima svoj brojač prisutnosti, indeks odgovara studentu (0=Student1, 1=Student2, 2=Student3)
uint32_t attendance[3] = {0, 0, 0};

//======================//
//   PINovi STUDENATA   //
//======================//
const char Student1_PIN[] = "1234";
const char Student2_PIN[] = "4545";
const char Student3_PIN[] = "2323";

//===========================//
//   PRACENJE DOLAZAKA       //
//===========================//
const char *StudentNames[3] = {"Adrian", "Sara", "Iva"}; // imena za prikaz na LCD-u

uint8_t presentToday[3] = {0, 0, 0}; // 1 = bio na satu danas, 0 = nije

//==================================//
//   BROJAC ODRZANIH SATI           //
//==================================//
//broji koliko je sati odrzano, pocinje na 1 jer se smatra da je plocica u prvom satu od trenutka kad se upali
uint32_t classesHeld = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//=============================//
//   LCD FUNCTION PROTOTYPES   //
//=============================//
void I2C_LCD_Init(void);
void I2C_LCD_Send_Cmd(uint8_t cmd) ;               // Šalje naredbu LCD-u (npr. očisti ekran)
void I2C_LCD_Send_Data(uint8_t data);              // Šalje podatak (znak) na LCD za prikaz (npr. pokaži znak, broj...)
void I2C_LCD_Send_String(char *str);               // Šalje tekst (string) na LCD za prikaz
void I2C_LCD_Send_Byte(uint8_t val, uint8_t mode); // I2C_LCD_Send_Cmd i Data zovu ovu funkciju koja određuje oće li se tretirati kao naredba 0 ili podatak 1
void I2C_LCD_Pulse_En(uint8_t data);               // Šalje kratki signal bez kojeg LCD ništa ne šalje


//==============================//
//   RFID FUNCTION PROTOTYPES   //
//==============================//
void RC522_Init(void);
void RC522_WriteRegister(uint8_t reg, uint8_t value); // 1.šalje adresu "reg" u koju se piše / 2.šalje "value" vrijednost koja se piše (tako se RFIDu govori da npr. upali antenu)
uint8_t RC522_ReadRegister(uint8_t reg);              // Pročita što piše u određenom registru "reg"
uint8_t RC522_Request(uint8_t *cardType);             // Provjerava je li kartica prisutna u polju čitača i identificira njezin tip - Ako je - vraća 1 / Nije - 0
uint8_t RC522_Anticoll(uint8_t *serNum);              // Sprječavanja sudare (ako ima više kartica) kako bi odabrao jednu karticu i pročitao njezin UID
void RC522_Reset(void);
void RC522_AntennaOn(void);                           // Uključi antenu RFID čitača i tako omogućuje komunikaciju sa karticama

//==============================//
//   PIN FUNCTION PROTOTYPE     //
//==============================//
uint8_t WaitForPIN(const char *correctPIN); // Čeka unos PINa putem UART-a i uspoređuje ga s ispravnim

//==============================//
//  SUMMARY FUNCTION PROTOTYPE  //
//==============================//
void ShowClassSummary(void); // Prikazuje na LCD-u tko je bio na satu, a tko nije

//============================//
//   POSTOTAK PROTOTYPE       //
//============================//
void ShowAttendancePercentage(void); // Prikazuje postotak dolazaka za svakog studenta (attendance / classesHeld)

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Varijable su deklarirane negdje drugdje
// "extern" korsitim zato što omogućuje da ih koristim u bilo kojem dijelu programa bez potrebe za ponovnim definiranjem
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart2;


//===========================//
//   LCD CONTROL FUNCTIONS   //
//===========================//
// Funkcija za slanje bajta preko I2C
// Šalje bajt (val) u dvije polovice jer je u 4-bitnom modu (4 gornja i 4 donja)
void I2C_LCD_Send_Byte(uint8_t val, uint8_t mode) {
    uint8_t high_nibble = (val & 0xF0) | LCD_BACKLIGHT | mode;        // uzima gornjih 4 bita(7-4) od bajta "val" i kombinira ih sa LCD_BACKLIGHT i mode, donje bitove (3-0) postavlja na 0
    uint8_t low_nibble = ((val << 4) & 0xF0) | LCD_BACKLIGHT | mode;  // uzima donja 4 bita(3-0) od bajta "val" i shifta u lijevo za 4 u poziciju high nibblea, donje bitove postavlja na 0
    I2C_LCD_Pulse_En(high_nibble); // Prvo šalje gornja 4 bita
    I2C_LCD_Pulse_En(low_nibble);  // Zatim šalje donja 4 bita
}

// Generira enable puls
void I2C_LCD_Pulse_En(uint8_t data) { // uzima podatke koje već imamo, dodaje EN bit i privremeno sprema u "temp", pa kad ga pošaljemo LCD-u, linija EN ide u 1 i omogućava da LCD pročita podatke
    uint8_t temp = data | LCD_ENABLE; // EN = 1, svi ostali bitovi ostaju isti zbog |
    HAL_I2C_Master_Transmit(&hi2c1, PCF8574T_ADDRESS, &temp, 1, HAL_MAX_DELAY); // šalje pripremljeni bajt "temp" preko I2C sabirnice na PCF8574T
    HAL_Delay(1);
    temp &= ~LCD_ENABLE; // invertira brojeve - sve je 1 osim EN bita koji je 0
    HAL_I2C_Master_Transmit(&hi2c1, PCF8574T_ADDRESS, &temp, 1, HAL_MAX_DELAY); // isto kao i ovo gore ali sada "temp" ima EN pin očišćen
    HAL_Delay(1);
}

// Funkcija za slanje komande LCD-u
// 0 indikator da se šalje naredba
void I2C_LCD_Send_Cmd(uint8_t cmd) {
    I2C_LCD_Send_Byte(cmd, 0);
}

// Funkcija za slanje podataka (data)
void I2C_LCD_Send_Data(uint8_t data) {
    I2C_LCD_Send_Byte(data, LCD_RS);  // RS = 1 - znači da se šalje podatak, LCD_RS definiran gore
}

// Funkcija za slanje stringa na LCD
void I2C_LCD_Send_String(char *str) {
    while (*str) {                    // while prolazi kroz svaki znak stringa dok zadnji znak nije \0 i šalje ga pomoću I2C_LCD_Send_Data()
        I2C_LCD_Send_Data(*str++);    // *str++ pomiče pokazivač na sljedeći znak u nizu
    }
}

// Inicijalizacija LCD-a
void I2C_LCD_Init(void) {
    HAL_Delay(50);
    I2C_LCD_Send_Cmd(0x33); // forsiranje LCD-a u 4-bitni način rada
    HAL_Delay(5);
    I2C_LCD_Send_Cmd(0x32); // potvrda da je u 4-bitnom načinu
    HAL_Delay(5);
    I2C_LCD_Send_Cmd(0x28); // 4-bit, 2 linije, 5x8 font
    I2C_LCD_Send_Cmd(0x0C); // Uključi prikaz, isključi kursor
    I2C_LCD_Send_Cmd(0x06); // Automatski pomak kursora
    I2C_LCD_Send_Cmd(0x01); // Očisti zaslon
    HAL_Delay(2);
}


//===========================//
//   RFID READER FUNCTIONS   //
//===========================//
// Prvi korak u SPI, priprema čip za primanje adrese i vrijednosti registra
void RC522_WriteRegister(uint8_t reg, uint8_t value) {
	// NSS (Slave Selecet) se postavlja na LOW što znači da se RFID modul aktivira i spreman je primiti podatke putem SPI-a
    HAL_GPIO_WritePin(RFID_NSS_PORT, RFID_NSS_PIN, GPIO_PIN_RESET);
    uint8_t data[2];             // kreira polje od 2 bajta koji se šalju preko SPI
    data[0] = (reg << 1) & 0x7E; // adresa registra shiftana ulijevo za 1 i "maskirana" s 0x7E
    data[1] = value;             // vrijednost koja se upisuje u reigstar
    // šalje 2 bajta putem SPI (prvi bajt - adresa registra, drugi bajt - vrijednost koja se upisuje u taj registar)
    HAL_SPI_Transmit(MFRC522_SPI, data, 2, HAL_MAX_DELAY); // šalje 2 bajta data od gore čipu preko SPI
    // NSS se postavalja na HIGH, završava SPI prijenos i RFID zna da je upis podataka završen
    HAL_GPIO_WritePin(RFID_NSS_PORT, RFID_NSS_PIN, GPIO_PIN_SET);
}

// Pomoću SPI-a čita vrijednosti iz registra RFID-a
uint8_t RC522_ReadRegister(uint8_t reg) {
	// NSS je LOW (omogućuje komunikaciju između mikrokontrolera i RFID-a)
    HAL_GPIO_WritePin(RFID_NSS_PORT, RFID_NSS_PIN, GPIO_PIN_RESET);
    uint8_t data[2];
    data[0] = ((reg << 1) & 0x7E) | 0x80; // komanda za čitanje (bit 7 = 1 za čitanje)
    data[1] = 0;                          // lažni bajt jer čip vraća vrijednost kao odgovor
    HAL_SPI_TransmitReceive(MFRC522_SPI, data, data, 2, HAL_MAX_DELAY); // istovremeno se pomiču 2 bajta, 1. je "smeće" 2. je zapravo vrijednost registra
    // NSS je HIGH (završava SPI komunikacija)
    HAL_GPIO_WritePin(RFID_NSS_PORT, RFID_NSS_PIN, GPIO_PIN_SET);
    return data[1];
}
// Postavlja (uključuje) određene bitove u zadanom registru RFID čitača / npr. uključivanje antene
void RC522_SetBitMask(uint8_t reg, uint8_t mask) {
    uint8_t tmp = RC522_ReadRegister(reg); // pročita trenutnu vrijednost "reg" i spremi u privremenu varijablu tmp
    RC522_WriteRegister(reg, tmp | mask);  // koristi OR da bi postavio na 1 one bitove koji su 1 u mask
}
// Briše određene bitove u zadanom registru RFID-a
void RC522_ClearBitMask(uint8_t reg, uint8_t mask) { // postavi na 0 bitove u registru čiša bez da dira ostale bitove
    uint8_t tmp = RC522_ReadRegister(reg);           // pročita trenutni sadržaj "reg" i sprema se u "tmp"
    RC522_WriteRegister(reg, tmp & (~mask));         // AND s negacijom - isključi (postavi na 0) bitove definirane u mask / Zapisuje natrag izmijenjenu vrijednost u registar
}
// Provjerava jel antena OFF, ako je UPALI ju
void RC522_AntennaOn(void) {
    uint8_t temp = RC522_ReadRegister(TxControlReg); // čita sadržaj registra TxControlReg - kontrolira jesu li upravljački programi antene omogućeni ili ne
    if (!(temp & 0x03)) {                            // provjerava najniža 2 bita registra, ako su oba 0 antena je OFF
        RC522_SetBitMask(TxControlReg, 0x03);        // ovo pali antenu
    }
}

void RC522_Reset(void) {
    RC522_WriteRegister(CommandReg, PCD_RESETPHASE); // u CommandReg piše PCD_RESETPHASE time se čip resetira
}

void RC522_Init(void) {
    HAL_GPIO_WritePin(RFID_RST_PORT, RFID_RST_PIN, GPIO_PIN_SET); // reset pin mora biti u visokom stanju za normalan rad
    HAL_Delay(50);

    RC522_Reset();

    // Inicijalizacija timera RFIDa
    RC522_WriteRegister(TModeReg, 0x8D);        // omogućuje automatsko pokretanje timera, ključno za otkrivanje odgovora kartica unutar određenog vremena
    RC522_WriteRegister(TPrescalerReg, 0x3E);   // Postavlja frekvenciju timera na otp 40kHz, to određuje koliko timer brzo odbrojava operacije npr. komunikacija s karticom
    RC522_WriteRegister(TReloadRegL, 30);       // definira koliko dugo čip čeka odgovor kartice
    RC522_WriteRegister(TReloadRegH, 0);

    RC522_WriteRegister(TxASKReg, 0x40);        // 100% ASK modulation (potrebno za pouzdanu komunikaciju s karticama)
    RC522_WriteRegister(ModeReg, 0x3D);         // osigurava da čip koristi ispravne CRC postavke za kartice
    RC522_AntennaOn();                          // Uključuje antenu RFID čitača, bez toga čitač ne može vidjeti kartice.
}
// Glavna komunikacijska funkcija - obavlja komunikaciju između RC522 RFID čitača i kartice
uint8_t RC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen) {
    uint8_t status = 0;     // oznaka statusa komunikacije / status: 0=OK, 1=greška
    uint8_t irqEn = 0x00;   // koje prekide treba omogućiti u čipu
    uint8_t waitIRq = 0x00; // koji prekid se očekuje (npr. primljeni podaci ili stanje mirovanja)
    uint8_t lastBits;       // sprema broj valjanih bitova u zadnjem bajtu primljenih podataka
    uint8_t n;              // varijabla za spremanje privremenih vrijednosti (npr. broj bajtova iz FIFO-a)
    uint16_t i;             // brojač za petlje

    switch (command) {
        case PCD_TRANSCEIVE: // naredba koja šalje podatke kartici i prima odgovor
            irqEn = 0x77;    // omogu
            waitIRq = 0x30;  // čeka da se dogodi RxIRq (primljen podatak) ili IdleIRq (gotova operacija)
            break;
        default:
            break;
    }

    RC522_WriteRegister(CommIEnReg, irqEn | 0x80);     // omogućuje specifične prekide (npr. primljenje podatke ili dovršetak naredbe)
    RC522_ClearBitMask(CommIrqReg, 0x80);              // osigurava da zastarjele "flags" ne ometaju novu operaciju
    RC522_SetBitMask(FIFOLevelReg, 0x80);              // FIFO bafer (služi za pohranu podataka sa i na karticu) se prazni i priprema za nove podatke)
    RC522_WriteRegister(CommandReg, PCD_IDLE);         // Postavlja RC552 u PCD_IDLE (Osigurava da nije nijedna stara komanda aktivna) i tako sprjećava konflikte

    // Pisanje podataka u FIFO
    for (i = 0; i < sendLen; i++) {                    // prolazi kroz spremik sendData i zapisuje svaki bajt u FIFO registar
        RC522_WriteRegister(FIFODataReg, sendData[i]); // podaci koji će se prenijeti na karticu
    }

    // Izvršavanje naredbe
    RC522_WriteRegister(CommandReg, command);          // piše odabranu "command" u CommandReg
    if (command == PCD_TRANSCEIVE) {                   // ako je "command" PCD_TRANSCEIVE šalje FIFO podatke kartici
        RC522_SetBitMask(BitFramingReg, 0x80);
    }

    // Čekanje dovršetka naredbe(dok npr.nisu primljeni podaci ili završena operacija ili error)
    i = 2000; // stavio sam 2000 da ne bude beskonačno
    do {
        n = RC522_ReadRegister(CommIrqReg); // ispituje CommIrqReg do 2000 puta čekajući prekid koji oznaćava završetak naredbe
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq)); // i != 0 ponovi petlju dok nije isteklo vrijeme
                                                         // n & 0x01 nastavi petlju dok timer čipa nije istekao
    RC522_ClearBitMask(BitFramingReg, 0x80);             // Osigurava da čip prestane slati podatke nakon što je naredba dovršena
    if (i != 0) {                                        // provjerava je li petlja završila zbog prekida a ne zbog isteka vremena
        if (!(RC522_ReadRegister(ErrorReg) & 0x1B)) {    // ako nije bilo errora status = 0, dolje na kraju else- ako je bilo errora status = 2 pogreška
            status = 0;                                  // status = 0 uspješno
            if (n & irqEn & 0x01) {
                status = 1;                              // status = 1 isteklo vrijeme
            }

            if (command == PCD_TRANSCEIVE) {             // izvršava se samo ako se šalju podaci i očekuju odgovori, ako je nešto drugo preskače se
                n = RC522_ReadRegister(FIFOLevelReg);    // FIFOLevelReg - koliko bajtova je slobodno u FIFO
                lastBits = RC522_ReadRegister(ControlReg) & 0x07; // koliko valjanih bitova ima u posljednjem primljenom bajtu
                if (lastBits) {                          // računa backLen (duljinu odgovora u bitovima)
                    *backLen = (n - 1) * 8 + lastBits;   // ako zadnji bajt nije potpuno popunjen ukupuna duljina je puni bajt
                } else {
                    *backLen = n * 8;                    // else cijeli bajt
                }

                if (n == 0) {                            // Ako nema podataka (n == 0), postavi barem 1 bajt (jer čitanje iz FIFO uvijek treba dati nešto)
                    n = 1;
                }
                if (n > 16) {                            // Ako ima više od 16 bajtova, ograniči na 16
                    n = 16;
                }

                // Čita bajtove iz FIFO i sprema ih (pohranjuje se odgovor kartice
                for (i = 0; i < n; i++) {                // Petlja čita svaki bajt iz FIFO buffer registra FIFODataReg.
                    backData[i] = RC522_ReadRegister(FIFODataReg); // Rezultat se sprema u polje backData
                }
            }
        } else {
            status = 2;
        }
    }

    return status;
}

uint8_t RC522_Request(uint8_t *cardType) {    // deklarira funkciju koja šalje zahtjev za detekciju RFID kartice i pohranjuje njezin tip u "cardType"
    uint8_t status;                           // vraća "status" 0 za uspjeh a 1 za failure
    uint8_t backBits;                         // pohranjuje broj bitova u odgovoru kartice (koji vraća RC522_ToCard)
    RC522_WriteRegister(BitFramingReg, 0x07); // registar koji podešava način kako se šalju i primaju podaci

    cardType[0] = PICC_REQIDL;                // komanda koja detektira karticu u polju čitača, postavlja prvi bajt polja "cardType" na "PICC_REQIDL"
    status = RC522_ToCard(PCD_TRANSCEIVE, cardType, 1, cardType, &backBits); // Poziva "RC522_ToCard" za slanje naredbe "PICC_REQIDL" i primanje odgovora kartice.
 /* PCD_TRANSCEIVE - naredba čitaču da pošalje podatke i primi odgovor
    cardType - podatak koji se šalje,
    1 - broj bajotva koji se šalje,
    cardType - buffer u koji ćemo spremiti podatke koje kartica vrati
    &backBits - koliko bitova je vraćeno. */
    if ((status != 0) || (backBits != 0x10)) { // provjerava je li došlo do greške u komunikaciji
                                               // ako komunikacije nije uspjela ili ako kartica nije vratila očekivanih 16 bitova

        status = 1;                            // "status" se postavlja na 1 = greška
    }

    return status;                             // "status" 0 = uspjeh 1 = neuspjeh
}
// funkcija koja sprječava sudare više kartica
uint8_t RC522_Anticoll(uint8_t *serNum) {      // "serNum" je pokazivač na niz od 5 bajtova (pohranjeni su 4 bajta UID kartice i 1 bajt za kontrolu
    uint8_t status;            // sprema rezultat komunikacije 0 = ok, ako nije 0 greška
    uint8_t i;                 // brojač petlje za izračun kontrolne sume
    uint8_t serNumCheck = 0;   // koristi se za kontrolnu sumu (checksum) - XOR
    uint8_t backLen;           // pohranjuje broj bitova koji vraća "RC522_ToCard"
    RC522_WriteRegister(BitFramingReg, 0x00); // slanje i primanje punih bajtova za naredbu protiv kolizije
    // slanje komandi kartici i zahtjev za njen UID
    serNum[0] = PICC_ANTICOLL; // postavlja 1.bajt "serNum" na anti-collison command kod
    serNum[1] = 0x20;          // osigurava da kartica pošalje puni UID
    // Ovo je zapravo komunikacija s kartiocm, njezin odabir i dovinanje njezinog UIDa
    status = RC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &backLen);
  /* PCD_TRANSCEIVE - Naredba čipu za slanje podataka i primanje odgovora
     serNum - šaljemo prema kartici
     2 - 2 bajta dužina podatka
     serNum - odgovor kartice UID kontrolni bajt */
    if (status == 0) {                  // osigurava da se validacija izvrši samo ako je odogovor uspješno primljen
        for (i = 0; i < 4; i++) {       // prolazi kroz prva četiri bajda UIDa kartice i računa XOR kako bi se dobio kontrolni bajt (npr. 0x26)
            serNumCheck ^= serNum[i];   // BCC (0x26) se potom uspoređuje s primljenim BCCom koji je poslala kartica
        }
        if (serNumCheck != serNum[i]) { // ako se podudaraju UID je ispravan
            status = 2;                 // ako ne "status" je 2 i to je greška
        }
    }

    return status;
}

/* USER CODE END 0 */

//==========================//
//   WaitForPIN FUNKCIJA    //
//==========================//
// Čeka da student upiše PIN u Serial Monitor (UART2), pa uspoređuje s ispravnim PINom
// Vraća 1 ako je PIN točan, 0 ako nije
uint8_t WaitForPIN(const char *correctPIN) {
    char received[10];
    uint8_t idx = 0;
    uint8_t byte;

    memset(received, 0, sizeof(received));

    // pomakni kursor u 2. red
    I2C_LCD_Send_Cmd(0xC0);

    while (idx < 9) {
        if (HAL_UART_Receive(&huart2, &byte, 1, 10000) == HAL_OK) {
            if (byte == '\r' || byte == '\n') {
                HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);
                break;
            }
            HAL_UART_Transmit(&huart2, &byte, 1, 100);
            I2C_LCD_Send_Data('*'); // na LCD se ispisuje "*" umjesto stvarne znamenke kao recimo na bankomatu
            received[idx++] = (char)byte;
        } else {
            break;
        }
    }
    received[idx] = '\0'; //null terminator

    if (strcmp(received, correctPIN) == 0) { //string compare, je li unesei PIN isti kao točan PIN
        return 1;  //vrati 1 - točno
    }
    return 0;
}

//==========================//
//   ShowClassSummary       //
//==========================//
// Prikazuje na LCD-u popis tko je bio na satu, tko nije,
void ShowClassSummary(void) {

    // 1. dio - prikaži sve prisutne studente, jednog po jednog
    I2C_LCD_Send_Cmd(0x01);
    I2C_LCD_Send_String("Na satu:");
    HAL_Delay(2000);

    for (uint8_t i = 0; i < 3; i++) { // 3 jer su 3 studenta
        if (presentToday[i] == 1) {   // ako je presentToday[i] == 1, student je prisutan
            I2C_LCD_Send_Cmd(0x01);
            I2C_LCD_Send_String("Prisutni:");
            I2C_LCD_Send_Cmd(0xC0);
            I2C_LCD_Send_String(StudentNames[i]);
            HAL_Delay(2500);
        }
    }

    // 2. dio - prikaži sve odsutne studente, jednog po jednog
    I2C_LCD_Send_Cmd(0x01);
    I2C_LCD_Send_String("Nisu prisutni:");
    HAL_Delay(2000);

    uint8_t anyMissing = 0;
    for (uint8_t i = 0; i < 3; i++) {
        if (presentToday[i] == 0) { //ako je presentToday[i] == 0, student nije prisutan
            anyMissing = 1;
            I2C_LCD_Send_Cmd(0x01);
            I2C_LCD_Send_String("Nisu prisutni:");
            I2C_LCD_Send_Cmd(0xC0);
            I2C_LCD_Send_String(StudentNames[i]);
            HAL_Delay(2500);
        }
    }

    if (anyMissing == 0) {
        I2C_LCD_Send_Cmd(0x01);
        I2C_LCD_Send_String("Svi su prisutni!");
        HAL_Delay(2500);
    }

    // Vrati početnu poruku
    I2C_LCD_Send_Cmd(0x01);
    I2C_LCD_Send_String("Prislonite vasu");
    I2C_LCD_Send_Cmd(0xC0);
    I2C_LCD_Send_String("karticu");
}

//=============================//
//   ShowAttendancePercentage  //
//=============================//
// Prikazuje na LCD-u postotak dolazaka za svakog studenta, jedan po jedan
// Racuna se kao (attendance[i] * 100) / classesHeld (broj STVARNO odrzanih sati)
void ShowAttendancePercentage(void) {

    I2C_LCD_Send_Cmd(0x01);
    I2C_LCD_Send_String("Postotak dolaska:");
    HAL_Delay(2000);

    for (uint8_t i = 0; i < 3; i++) {
    	uint32_t percent = 0;
    	if (classesHeld != 0) {
    	        percent = (attendance[i] * 100UL) / classesHeld;
    	    }

        char line1[16];
        char line2[16];
        sprintf(line1, "%s:", StudentNames[i]);
        sprintf(line2, "%lu/%lu (%lu%%)", attendance[i], classesHeld, percent);

        I2C_LCD_Send_Cmd(0x01);
        I2C_LCD_Send_String(line1);
        I2C_LCD_Send_Cmd(0xC0);
        I2C_LCD_Send_String(line2);
        HAL_Delay(2500);
    }

    // Vrati pocetnu poruku nakon prikaza
    I2C_LCD_Send_Cmd(0x01);
    I2C_LCD_Send_String("Prislonite vasu");
    I2C_LCD_Send_Cmd(0xC0);
    I2C_LCD_Send_String("karticu");
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();

  // Inicijalizacija LCD and RFID
  I2C_LCD_Init();
  RC522_Init();

  // Početna poruka
  I2C_LCD_Send_Cmd(0x01);
  I2C_LCD_Send_String("Prislonite vasu");
  I2C_LCD_Send_Cmd(0xC0);
  I2C_LCD_Send_String("karticu!");

  /* USER CODE BEGIN 2 */
    uint8_t cardUID[5];
    uint8_t cardType[2];


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //===================//
  //   GLAVNA PETLJA   //
  //===================//
  while (1)
  {
	  if (RC522_Request(cardType) == 0) {
	      if (RC522_Anticoll(cardUID) == 0) {
	    	  // Provjeri jel kartica autorizirana
	    	  if (memcmp(cardUID, Student1, 4) == 0) {

	              // Traži unos PINa na LCD-u, student ga upisuje u Serial Monitor na PC-u
	              I2C_LCD_Send_Cmd(0x01);
	              I2C_LCD_Send_String("Unesite vas PIN:");

	              if (WaitForPIN(Student1_PIN)) {

	                  // PIN tocan - povecaj brojac prisutnosti
	                  attendance[0]++;

	                  presentToday[0] = 1;

	                  // Pali zelenu LED lampicu
	                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

	                  // Pokazi dobrodoslicu i broj prisutnosti
	                  char buf1[16];
	                  I2C_LCD_Send_Cmd(0x01);
	                  I2C_LCD_Send_String("Bok Adrian!");
	                  I2C_LCD_Send_Cmd(0xC0);
	                  sprintf(buf1, "Prisutnost:%lu/%lu", attendance[0], classesHeld);
	                  I2C_LCD_Send_String(buf1);

	                  HAL_Delay(3000);
	                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	              } else {
	                  // PIN netocnan - upali crvenu LED
	                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	                  I2C_LCD_Send_Cmd(0x01);
	                  I2C_LCD_Send_String("Pogresan PIN!");
	                  HAL_Delay(3000);
	                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	              }

	              // Vrati pocetnu poruku
	              I2C_LCD_Send_Cmd(0x01);
	              I2C_LCD_Send_String("Prislonite vasu");
	              I2C_LCD_Send_Cmd(0xC0);
	              I2C_LCD_Send_String("karticu");
	              }
	              else if (memcmp(cardUID, Student2, 4) == 0) {

	            	  I2C_LCD_Send_Cmd(0x01);
	            	  I2C_LCD_Send_String("Unesite vas PIN:");

	                  if (WaitForPIN(Student2_PIN)) {

	                      attendance[1]++;

	                      presentToday[1] = 1;

	                      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

	                      char buf2[16];
	                      I2C_LCD_Send_Cmd(0x01);
	                      I2C_LCD_Send_String("Bok Sara!");
	                      I2C_LCD_Send_Cmd(0xC0);
	                      sprintf(buf2, "Prisutnost:%lu/%lu", attendance[1], classesHeld);
	                      I2C_LCD_Send_String(buf2);

	                      HAL_Delay(3000);
	                      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	                  } else {
	                      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	                      I2C_LCD_Send_Cmd(0x01);
	                      I2C_LCD_Send_String("Pogresan PIN!");
	                      HAL_Delay(3000);
	                      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	                  }

	                  I2C_LCD_Send_Cmd(0x01);
	                  I2C_LCD_Send_String("Prislonite vasu");
	                  I2C_LCD_Send_Cmd(0xC0);
	                  I2C_LCD_Send_String("karticu");
	              }
	              else if (memcmp(cardUID, Student3, 4) == 0) {

	            	  I2C_LCD_Send_Cmd(0x01);
	            	  I2C_LCD_Send_String("Unesite vas PIN:");

	                  if (WaitForPIN(Student3_PIN)) {

	                      attendance[2]++;

	                      presentToday[2] = 1;

	                      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

	                      char buf3[16];
	                      I2C_LCD_Send_Cmd(0x01);
	                      I2C_LCD_Send_String("Bok Iva!");
	                      I2C_LCD_Send_Cmd(0xC0);
	                      sprintf(buf3, "Prisutnost:%lu/%lu", attendance[2], classesHeld);
	                      I2C_LCD_Send_String(buf3);

	                      HAL_Delay(3000);
	                      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	                  } else {
	                      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	                      I2C_LCD_Send_Cmd(0x01);
	                      I2C_LCD_Send_String("Pogresan PIN!");
	                      HAL_Delay(3000);
	                      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	                  }

	                  I2C_LCD_Send_Cmd(0x01);
	                  I2C_LCD_Send_String("Prislonite vasu");
	                  I2C_LCD_Send_Cmd(0xC0);
	                  I2C_LCD_Send_String("karticu");
	              }

	              else
	              {
	              	     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	              	     I2C_LCD_Send_Cmd(0x01);
	              	     I2C_LCD_Send_String("Nemate");
	              	     I2C_LCD_Send_Cmd(0xC0);
	              	     I2C_LCD_Send_String("pristup!");

	              	     HAL_Delay(3000);
	              	     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	              	     I2C_LCD_Send_Cmd(0x01);
	              	     I2C_LCD_Send_String("Prislonite vasu");
	              	     I2C_LCD_Send_Cmd(0xC0);
	              	     I2C_LCD_Send_String("karticu!");
	             }
	         }
	  }
	         //=============================//
	         //   B1 GUMB - PRIKAZ POPISA   //
	         //=============================//
	         // Gumb je spojen tako da je normalno u stanju 1 (HIGH), a kad se pritisne ide na 0 (LOW)
	         if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
	             HAL_Delay(50); // mala pauza da se izbjegne debounce
	             if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) { //provjera jel gumb stvarno stisnut

	                 uint32_t pressStart = HAL_GetTick(); // zabiljezi trenutak kad je gumb pritisnut

	                 while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
	                     HAL_Delay(10); // cekaj da korisnik otpusti gumb, da se prikaz ne pokrene vise puta
	                 }

	                 uint32_t pressDuration = HAL_GetTick() - pressStart; // koliko dugo je gumb bio stisnut

	                 if (pressDuration >= 2000) { // ako je gumb pritisnut 2+ sekundee

	                     // Resetiraj tko je danas prisutan, NE diraj attendance[]
	                     for (uint8_t i = 0; i < 3; i++) {
	                         presentToday[i] = 0;
	                     }
	                     classesHeld++; // povećavaj broj nastavnih sati

	                     // Ispisuje redni broj novog sata, npr. "Novi nastavni sat (2)", "(3)" itd.
	                     char classMsg[17];
	                     I2C_LCD_Send_Cmd(0x01);
	                     I2C_LCD_Send_String("Novi nastavni");
	                     I2C_LCD_Send_Cmd(0xC0);
	                     sprintf(classMsg, "sat (%lu)", classesHeld);
	                     I2C_LCD_Send_String(classMsg);
	                     HAL_Delay(2000);

	                     I2C_LCD_Send_Cmd(0x01);
	                     I2C_LCD_Send_String("Prislonite vasu");
	                     I2C_LCD_Send_Cmd(0xC0);
	                     I2C_LCD_Send_String("karticu");
	                 } else {
	                     // Kratak pritisak = prikazi popis i postotke (kao i prije)
	                     ShowClassSummary(); // prikazi tko je na satu, a tko nije
	                     ShowAttendancePercentage(); // prikazi postotak dolazaka za svakog studenta
	                 }
	             }
	         }

	         HAL_Delay(100);
	         /* USER CODE END WHILE */

	             /* USER CODE BEGIN 3 */
	           }
	           /* USER CODE END 3 */
	         }

	         /**
	           * @brief System Clock Configuration
	           * @retval None
	           */
	         void SystemClock_Config(void)
	         {
	           RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	           RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	           /** Configure the main internal regulator output voltage
	           */
	           __HAL_RCC_PWR_CLK_ENABLE();
	           __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	           /** Initializes the RCC Oscillators according to the specified parameters
	           * in the RCC_OscInitTypeDef structure.
	           */
	           RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	           RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	           RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	           RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	           RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	           RCC_OscInitStruct.PLL.PLLM = 16;
	           RCC_OscInitStruct.PLL.PLLN = 336;
	           RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	           RCC_OscInitStruct.PLL.PLLQ = 2;
	           RCC_OscInitStruct.PLL.PLLR = 2;
	           if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	           {
	             Error_Handler();
	           }

	           /** Initializes the CPU, AHB and APB buses clocks
	           */
	           RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                                       |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	           RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	           RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	           RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	           RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	           if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	           {
	             Error_Handler();
	           }
	         }

	         /* USER CODE BEGIN 4 */

	         /* USER CODE END 4 */

	         /**
	           * @brief  This function is executed in case of error occurrence.
	           * @retval None
	           */
	         void Error_Handler(void)
	         {
	           /* USER CODE BEGIN Error_Handler_Debug */
	           /* User can add his own implementation to report the HAL error return state */
	           __disable_irq();
	           while (1)
	           {
	           }
	           /* USER CODE END Error_Handler_Debug */
	         }

	         #ifdef  USE_FULL_ASSERT
	         /**
	           * @brief  Reports the name of the source file and the source line number
	           *         where the assert_param error has occurred.
	           * @param  file: pointer to the source file name
	           * @param  line: assert_param error line source number
	           * @retval None
	           */
	         void assert_failed(uint8_t *file, uint32_t line)
	         {
	           /* USER CODE BEGIN 6 */
	           /* User can add his own implementation to report the file name and line number,
	              ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	           /* USER CODE END 6 */
	         }
	         #endif /* USE_FULL_ASSERT */
