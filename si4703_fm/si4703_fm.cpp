#include "si4703_fm.h"
#include "esphome/core/log.h"
#include <cmath> // Pre std::round

namespace esphome {
namespace si4703_fm {

static const char *const TAG = "si4703_fm";
static const int UPDATE_INTERVAL_MS = 2000; // Zvýšené na 2s pre menej zahltenia

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
// Implementácia Number (Frequency)
// =============================================================================

void Si4703Frequency::setup() {
    // Pri štarte (po inicializácii hubu) zistíme aktuálnu frekvenciu
    float freq = this->parent_->get_current_frequency_mhz();
    if (freq > 0) {
      ESP_LOGD(TAG, "Number setup: Počiatočná frekvencia je %.1f MHz", freq);
      this->publish_state(freq);
    }
}

void Si4703Frequency::control(float freq_mhz) {
    ESP_LOGD(TAG, "Number control: Nastavuje sa frekvencia %.1f MHz", freq_mhz);
    this->parent_->set_channel_from_float(freq_mhz);
    // Publikujeme stav (optimistic update)
    // this->publish_state(freq_mhz); // Hub to už urobí v tune_to_channel
}

// =============================================================================
// Implementácia hlavnej triedy Si4703FM
// =============================================================================

void Si4703FM::setup() {
    ESP_LOGCONFIG(TAG, "Nastavovanie Si4703 FM Tunera (Hub)...");
    
    // Nastavenie pinov
    this->reset_pin_->setup();
    this->stc_int_pin_->setup(); 

    if (!this->si4703_init()) {
      ESP_LOGE(TAG, "Inicializácia Si4703 zlyhala!");
      this->mark_failed();
      return;
    }
    ESP_LOGCONFIG(TAG, "Si4703 inicializovaný.");
}

float Si4703FM::get_rssi() {
    if (!this->read_registers()) return 0.0f;

    // RSSI je uložené v registri STATUSRSSI (REG 0x0A) v bitoch 0-5. Max 63.
    uint8_t rssi_val = this->registers_[STATUSRSSI] & 0x3F; 

    return (float)rssi_val;
}

void Si4703FM::loop() {
    // Pravidelne kontrolujeme, či sa frekvencia nezmenila
    if (millis() - this->last_update_ > UPDATE_INTERVAL_MS) {
        this->last_update_ = millis();
        
        // 1. Aktualizácia frekvencie
        if (this->frequency_number_ != nullptr) {
            float freq = this->get_current_frequency_mhz();
            if (freq > 0 && this->frequency_number_->state != freq) {
                ESP_LOGD(TAG, "Loop: Aktualizácia frekvencie na %.1f MHz", freq);
                this->frequency_number_->publish_state(freq);
            }
        }
        
        // 2. Publikovanie RSSI
        if (this->rssi_sensor_ != nullptr) {
            float rssi = this->get_rssi();
            // Publikujeme iba ak je platná hodnota a zmenila sa (alebo ak je prvá)
            if (rssi > 0 && this->rssi_sensor_->state != rssi) { 
                ESP_LOGD(TAG, "Loop: Publikovanie RSSI: %.0f dBµV", rssi);
                this->rssi_sensor_->publish_state(rssi);
            }
        }
    }
}

bool Si4703FM::si4703_init() {
    // 1. Hardvérový Reset čipu (RST pin)
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

    // 2. Čítanie registrov
    if (!this->read_registers()) return false;

    // 3. Povolíme oscilátor (TEST1 = 0x8100)
    this->registers_[TEST1] = 0x8100;
    if (!this->update_all_registers()) return false;
    delay(500); // Čakáme na oscilátor

    // 4. Povolíme čip (POWERCFG = 0x4401 - Enable, DMUTE, MONO)
    if (!this->read_registers()) return false;
    this->registers_[POWERCFG] = 0x4401; 
    if (!this->update_all_registers()) return false;
    delay(110); // Max powerup time

    // 5. Nastavenie Európskeho pásma a hlasitosti
    if (!this->read_registers()) return false;
    // SYSCONFIG1: DE (De-emphasis) = 50us (Európa)
    this->registers_[SYSCONFIG1] |= (1 << 11); 
    // SYSCONFIG2: BAND = 00 (87.5-108 MHz), SPACE = 01 (100kHz Európa)
    this->registers_[SYSCONFIG2] &= 0xFF0F; 
    this->registers_[SYSCONFIG2] |= (1 << 4); // 100kHz
    // Hlasitosť 8
    this->registers_[SYSCONFIG2] &= 0xFFF0; 
    this->registers_[SYSCONFIG2] |= 0x0008;
    // Povolíme STC (Seek/Tune Complete) na GPIO2/STC pin
    this->registers_[SYSCONFIG3] |= (1 << 2);

    if (!this->update_all_registers()) return false;

    // 6. NASTAVENIE PREDVOLENEJ FREKVENCIE (104.8 MHz)
    ESP_LOGI(TAG, "Nastavujem predvolenú frekvenciu 104.8 MHz...");
    this->tune_to_channel(173); // 173 = 10 * (104.8 - 87.5)
    
    // 7. FINÁLNA AKTUALIZÁCIA STAVU ENTÍT
    float freq = this->get_current_frequency_mhz();
    if (this->frequency_number_ != nullptr) {
        this->frequency_number_->publish_state(freq);
    }
    
    // Nastavenie počiatočného stavu RSSI
    if (this->rssi_sensor_ != nullptr) {
        float rssi = this->get_rssi();
        this->rssi_sensor_->publish_state(rssi);
    }

    ESP_LOGI(TAG, "Si4703 inicializácia dokončená. Aktuálna frekvencia: %.1f MHz, RSSI: %.0f dBµV", freq, this->get_rssi());
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
    
    this->registers_[CHANNEL] &= 0xFE00; 
    this->registers_[CHANNEL] |= channel_val; 
    this->registers_[CHANNEL] |= TUNE; 
    
    if (!this->update_all_registers()) return false;

    // NAHRADENÉ PEVNOU PAUZOU: 200 ms
    ESP_LOGD(TAG, "Cakam pevne 200ms na dokoncenie ladenia...");
    delay(200);

    // Vyčistíme TUNE bit 
    if (!this->read_registers()) return false;
    this->registers_[CHANNEL] &= ~TUNE;
    if (!this->update_all_registers()) return false;
    
    // Zabezpečíme, že čítame novú frekvenciu pre HA
    if (!this->read_registers()) return false; 
    
    if (this->frequency_number_ != nullptr) {
        float freq = this->get_current_frequency_mhz();
        this->frequency_number_->publish_state(freq);
        ESP_LOGD(TAG, "Tuning ukonceny, aktualizovany stav na: %.1f MHz", freq);
    }

    return true;
}

float Si4703FM::get_current_frequency_mhz() {
    if (!this->read_registers()) return 0.0f;
    uint16_t channel_val = this->registers_[READCHANNEL] & 0x03FF;
    // Európa: Freq = 0.1 * Ch + 87.5
    float freq_mhz = ((float)channel_val * 0.1f) + 87.5f;
    return freq_mhz;
}

// --- I2C Komunikácia ---

bool Si4703FM::read_registers() {
    // Si4703 číta registre od 0x0A do 0x0F, a potom od 0x00 do 0x09
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
    // Si4703 zapisuje od 0x02 do 0x07 (12 bytov)
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