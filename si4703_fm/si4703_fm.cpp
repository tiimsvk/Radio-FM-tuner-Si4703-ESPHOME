#include "si4703_fm.h"
#include "esphome/core/log.h"
#include <cmath> // Pre std::round

namespace esphome {
namespace si4703_fm {

static const char *const TAG = "si4703_fm";

// =============================================================================
// Implementácia Konštruktora
// =============================================================================

Si4703FM::Si4703FM(i2c::I2CBus *parent_bus, GPIOPin *reset_pin, GPIOPin *stc_int_pin) 
    : i2c::I2CDevice(), reset_pin_(reset_pin), stc_int_pin_(stc_int_pin) 
{
    // Manuálne nastavenie bus pointera
    this->bus_ = parent_bus;
}

// =============================================================================
// Implementácia Switch (Power)
// =============================================================================

void Si4703PowerSwitch::write_state(bool state) {
    bool success;
    if (state) {
        success = this->parent_->turn_on();
    } else {
        success = this->parent_->turn_off();
    }
    
    if (success) {
        this->publish_state(state);
    }
}

// =============================================================================
// Implementácia Si4703FM (Power ON/OFF)
// =============================================================================
bool Si4703FM::turn_on() {
    ESP_LOGD(TAG, "Zapínam rádio (Power ON).");
    
    // UŽ SA NEVYPÍNA/NEZAPÍNA ČIP PRIAMO TU. Zapnutie prebehne v si4703_init().

    // Požadované oneskorenie po zapnutí (môžeme ho tu nechať, ak je to potrebné pre stabilizáciu napájania)
    delay(500); 
    
    // KĽÚČOVÁ ZMENA: Volanie inicializácie, ktorá zapne čip, naladí frekvenciu a nastaví hlasitosť.
    if (!this->si4703_init()) return false; 

    return true;
}

bool Si4703FM::turn_off() {
    ESP_LOGD(TAG, "Vypínam rádio (Power OFF).");
    
    // 1. OPRAVA: Prečítaj aktuálne registre z čipu
    if (!this->read_registers()) return false; 
    
    // 2. Modifikácia: Nastav bit DISABLE (D6) a vymaž ENABLE bit (D0)
    // Bit DISABLE (D6, 0x0040) je oficiálny spôsob, ako dať čip do power-down módu.
    this->registers_[POWERCFG] |= 0x0040;  // Nastav DISABLE (D6)
    this->registers_[POWERCFG] &= ~0x0001; // Vymaž ENABLE (D0)
    
    return this->update_all_registers();
}

// =============================================================================
// Implementácia Number (Frequency)
// =============================================================================

void Si4703Frequency::setup() {
    // Pri štarte (po inicializácii hubu) zistíme aktuálnu frekvenciu
    // Oneskorenie, aby sme dali hubu čas na 'si4703_init'
    this->set_timeout(1000, [this]() {
        if (!this->parent_->read_registers()) return;
        float freq = this->parent_->get_current_frequency_mhz();
        if (freq > 0) {
          ESP_LOGD(TAG, "Number setup: Počiatočná frekvencia je %.1f MHz", freq);
          this->publish_state(freq);
        }
    });
}

void Si4703Frequency::control(float freq_mhz) {
    ESP_LOGD(TAG, "Number control: Nastavuje sa frekvencia %.1f MHz", freq_mhz);
    this->parent_->set_channel_from_float(freq_mhz);
    // Hub sa postará o publikovanie stavu po úspešnom ladení
}

// =============================================================================
// NOVÉ: Implementácia Number (Volume)
// =============================================================================

void Si4703Volume::setup() {
    // Pri štarte zistíme aktuálnu hlasitosť
    this->set_timeout(1000, [this]() {
        if (!this->parent_->read_registers()) return;
        uint8_t vol = this->parent_->get_volume();
        ESP_LOGD(TAG, "Volume setup: Počiatočná hlasitosť je %d", vol);
        this->publish_state((float)vol);
    });
}

void Si4703Volume::control(float new_volume) {
    uint8_t vol = (uint8_t)std::round(new_volume);
    
    // Ochrana rozsahu 0-15
    if (vol > 15) vol = 15;
    
    ESP_LOGD(TAG, "Volume control: Nastavuje sa hlasitosť na %d", vol);
    
    if (this->parent_->set_volume(vol)) {
        // Ak je zápis úspešný, publikujeme nový stav
        this->publish_state((float)vol);
    }
}

// =============================================================================
// Implementácia hlavnej triedy Si4703FM (Hub)
// =============================================================================

void Si4703FM::setup() {
    // 1. I/O Pin setup
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    
    if (this->stc_int_pin_ != nullptr) {
        this->stc_int_pin_->setup();
    }
    
    this->reset_pin_->digital_write(true); 
    
    // ODSTRÁNILI SME: if (!this->si4703_init()) ...
    
    // 2. Kontrola stavu Switchu pri štarte
    if (this->power_switch_ != nullptr) {
        bool initial_state = this->power_switch_->get_initial_state();

        if (initial_state) {
             // Ak má byť Rádio ZAPNUTÉ (initial_state = ON):
             this->turn_on(); // Volá turn_on, ktoré volá si4703_init()
             this->power_switch_->publish_state(true);
        } else {
             // Ak má byť Rádio VYPNUTÉ (initial_state = OFF - DEFAULT):
             // Použijeme turn_off, aby sme čip uložili do power-down módu (nastavenie registrov).
             this->turn_off(); 
             this->power_switch_->publish_state(false);
        }
    }
    
    // Ak sa switch nepoužíva, môžete tu pridať fallback na this->turn_on()
    // Ale predpokladáme, že switch sa používa.
    
    ESP_LOGD(TAG, "Si4703 inicializovaný.");
}

// --- Implementácia pomocných funkcií pre entity ---

uint8_t Si4703FM::get_volume() {
    // Volume je v bitoch 0-3 registra SYSCONFIG2 (0x05)
    return this->registers_[SYSCONFIG2] & 0x0F;
}

bool Si4703FM::set_volume(uint8_t volume) {
    if (!this->read_registers()) return false;
    
    // 1. Vyčistíme staré bity hlasitosti (0-3)
    this->registers_[SYSCONFIG2] &= 0xFFF0; 
    // 2. Nastavíme novú hlasitosť
    this->registers_[SYSCONFIG2] |= volume;
    
    if (!this->update_all_registers()) return false;
    
    ESP_LOGV(TAG, "Hlasitosť nastavená na %d. Register SYSCONFIG2: 0x%04X", volume, this->registers_[SYSCONFIG2]);
    return true;
}

float Si4703FM::get_rssi() {
    // RSSI je uložené v registri STATUSRSSI (REG 0x0A) v bitoch 0-5. Max 63.
    uint8_t rssi_val = this->registers_[STATUSRSSI] & 0x3F; 
    return (float)rssi_val;
}

// --- Hlavná slučka (Loop) ---

void Si4703FM::loop() {
    if (millis() - this->last_update_ > this->update_interval_) {
        this->last_update_ = millis();
        
        // Čítame registre iba raz za cyklus
        if (!this->read_registers()) {
            ESP_LOGW(TAG, "Loop: Čítanie registrov z I2C zlyhalo. Dáta sa neaktualizujú!");
            return;
        }

        // 1. Aktualizácia frekvencie (ak nie je ovládaná)
        if (this->frequency_number_ != nullptr) {
            float freq = this->get_current_frequency_mhz();
            if (freq > 0 && this->frequency_number_->state != freq) {
                ESP_LOGD(TAG, "Loop: Aktualizácia frekvencie na %.1f MHz", freq);
                this->frequency_number_->publish_state(freq);
            }
        }
        
        // 2. Publikovanie RSSI
        if (this->rssi_sensor_ != nullptr) {
            // Použijeme dáta z read_registers(), ktoré sme už volali
            float rssi = this->get_rssi(); 
            ESP_LOGV(TAG, "Aktuálne namerané RSSI (REG 0x0A): %.0f dBµV", rssi);

            if (rssi > 0 && (this->rssi_sensor_->state != rssi || !this->rssi_sensor_->has_state())) { 
                ESP_LOGD(TAG, "Loop: Publikovanie RSSI: %.0f dBµV", rssi);
                this->rssi_sensor_->publish_state(rssi);
            }
        }

        // 3. Publikovanie Hlasitosti (ak nie je ovládaná)
        if (this->volume_number_ != nullptr) {
            uint8_t vol = this->get_volume();
            if (this->volume_number_->state != (float)vol || !this->volume_number_->has_state()) {
                ESP_LOGD(TAG, "Loop: Publikovanie Hlasitosti: %d", vol);
                this->volume_number_->publish_state((float)vol);
            }
        }
    }
}
// =============================================================================
// --- Inicializácia čipu ---
// =============================================================================

bool Si4703FM::si4703_init() {
    this->reset_pin_->digital_write(false); // RST LOW
    delay(1);
    this->reset_pin_->digital_write(true); // RST HIGH
    delay(1);
    
    ESP_LOGD(TAG, "I2C skenovanie...");
    if (!this->is_ready()) {
      ESP_LOGE(TAG, "Si4703 I2C zariadenie nenájdené na adrese 0x%02X", this->address_);
      return false;
    }
    ESP_LOGD(TAG, "Si4703 nájdený.");

    if (!this->read_registers()) return false;

    // 1. Povolíme oscilátor (TEST1 = 0x8100)
    this->registers_[TEST1] = 0x8100;
    if (!this->update_all_registers()) return false;
    delay(500); // Oscilátor potrebuje stabilizovať

    // 2. Povolíme čip pre konfiguráciu (POWERCFG = 0x4400 - DMUTE, MONO, ALE BEZ ENABLE)
    if (!this->read_registers()) return false;
    // ZMENA: Odstránili sme ENABLE bit (D0), hodnota 0x4401 zmenená na 0x4400
    this->registers_[POWERCFG] = 0x4400; 
    if (!this->update_all_registers()) return false;
    delay(110); // Power-up delay

    // 3. Nastavenie Európskeho pásma a hlasitosti
    if (!this->read_registers()) return false;
    this->registers_[SYSCONFIG1] |= (1 << 11); // DE (De-emphasis) = 50us (Európa)
    this->registers_[SYSCONFIG2] &= 0xFF0F; // Vymažeme bity 4-7
    this->registers_[SYSCONFIG2] |= (1 << 4); // 100kHz
    
    // Predvolená hlasitosť 8
    const uint8_t initial_volume = 8;
    this->registers_[SYSCONFIG2] &= 0xFFF0; 
    this->registers_[SYSCONFIG2] |= initial_volume;
    
    if (this->stc_int_pin_ != nullptr) {
        this->registers_[SYSCONFIG3] |= (1 << 2);
    }

    if (!this->update_all_registers()) return false; // Pošleme konfiguráciu bez ladenia/zapnutia.

    // 4. Nastavenie predvolenej frekvencie (Tento krok musí zapnúť aj ENABLE bit!)
    const uint16_t initial_channel = 173; 
    ESP_LOGI(TAG, "Nastavujem predvolenú frekvenciu 104.8 MHz...");
    this->tune_to_channel(initial_channel); // TUNE_TO_CHANNEL MUSÍ ZAPNÚŤ ENABLE BIT!
    
    // Finálne čítanie na potvrdenie stavu
    if (!this->read_registers()) return false;

    float freq = this->get_current_frequency_mhz();
    float rssi = this->get_rssi();

    ESP_LOGI(TAG, "Si4703 inicializácia dokončená. Aktuálna frekvencia: %.1f MHz, RSSI: %.0f dBµV, Hlasitosť: %d", freq, rssi, initial_volume);
    
    // Počiatočné publikovanie stavu
    if (this->frequency_number_ != nullptr) {
        this->frequency_number_->publish_state(freq);
    }
    if (this->rssi_sensor_ != nullptr) {
        this->rssi_sensor_->publish_state(rssi);
    }
    if (this->volume_number_ != nullptr) {
        this->volume_number_->publish_state((float)initial_volume);
    }

    return true;
}

// --- Funkcie na ovládanie čipu ---

void Si4703FM::set_channel_from_float(float freq_mhz) {
    uint16_t freq_x10 = (uint16_t)std::round(freq_mhz * 10.0f);
    uint16_t channel_val = (freq_x10 - 875); 
    
    ESP_LOGD(TAG, "Ladenie na %.1f MHz (Hodnota registra: %d)", freq_mhz, channel_val);
    this->tune_to_channel(channel_val);
}

bool Si4703FM::tune_to_channel(uint16_t channel_val) {
    if (!this->read_registers()) return false;
    
    // --- KĽÚČOVÁ ZMENA: ZAPNUTIE ČIPU (ENABLE) ---
    // Čip musí byť zapnutý (D0) pred odoslaním TUNE bitu.
    // Zabezpečíme, že POWERCFG má nastavený ENABLE bit (0x0001)
    this->registers_[POWERCFG] |= 0x0001; 
    
    // Nastavenie kanálu a TUNE bitu
    this->registers_[CHANNEL] &= 0xFE00; 
    this->registers_[CHANNEL] |= channel_val; 
    this->registers_[CHANNEL] |= TUNE; // TUNE je 1 << 15
    
    if (!this->update_all_registers()) return false;

    // Pevná pauza 200 ms
    ESP_LOGD(TAG, "Cakam pevne 200ms na dokoncenie ladenia...");
    delay(200);

    // Vyčistíme TUNE bit (a opäť odosielame POWERCFG s ENABLE)
    if (!this->read_registers()) return false;
    this->registers_[CHANNEL] &= ~TUNE;
    if (!this->update_all_registers()) return false;
    
    // Zabezpečíme, že čítame novú frekvenciu pre HA
    if (!this->read_registers()) return false; 
    
    if (this->frequency_number_ != nullptr) {
        float freq = this->get_current_frequency_mhz();
        // Poznámka: Ak je RSSI 0, freq bude stále 87.5. 
        // Chyba RSSI=0 by mala zmiznúť po správnom naladení.
        this->frequency_number_->publish_state(freq);
        ESP_LOGD(TAG, "Tuning ukonceny, aktualizovany stav na: %.1f MHz", freq);
    }
    // Aktualizujeme aj RSSI a Hlasitosť po ladení
    if (this->rssi_sensor_ != nullptr) {
        this->rssi_sensor_->publish_state(this->get_rssi());
    }
    if (this->volume_number_ != nullptr) {
        this->volume_number_->publish_state((float)this->get_volume());
    }

    return true;
}

float Si4703FM::get_current_frequency_mhz() {
    // Používame už načítané registre, nevoláme read_registers() znova
    uint16_t channel_val = this->registers_[READCHANNEL] & 0x03FF;
    float freq_mhz = ((float)channel_val * 0.1f) + 87.5f;
    return freq_mhz;
}

// --- I2C Komunikácia ---

bool Si4703FM::read_registers() {
    uint8_t read_data[32];
    if (this->read(read_data, 32) != i2c::ERROR_OK) { 
        ESP_LOGE(TAG, "I2C Čítanie (32 bytov) zlyhalo.");
        return false;
    }

    int i = 0;
    for (int x = 0x0A; ; x++) {
      if (x == 0x10) x = 0;
      this->registers_[x] = (uint16_t)(read_data[i] << 8) | read_data[i + 1];
      i += 2;
      if (x == 0x09) break;
    }
    return true;
}

bool Si4703FM::update_all_registers() {
    uint8_t data_to_write[12];
    int i = 0;
    for (int x = 0x02; x <= 0x07; x++) {
      data_to_write[i++] = (this->registers_[x] >> 8) & 0xFF;
      data_to_write[i++] = this->registers_[x] & 0xFF;
    }
    
    if (this->write(data_to_write, 12) != i2c::ERROR_OK) {
      ESP_LOGE(TAG, "I2C Zápis (12 bytov) zlyhal.");
      return false;
    }
    return true;
}

}  // namespace si4703_fm
}  // namespace esphome
