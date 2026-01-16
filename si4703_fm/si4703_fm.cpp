#include "si4703_fm.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace si4703_fm {

static const char *const TAG = "si4703_fm";

// =============================================================================
// Implementácia Konštruktora
// =============================================================================

Si4703FM::Si4703FM(i2c::I2CBus *parent_bus, GPIOPin *reset_pin, GPIOPin *stc_int_pin, GPIOPin *gpio1_pin, GPIOPin *gpio2_pin) 
    : i2c::I2CDevice(), reset_pin_(reset_pin), stc_int_pin_(stc_int_pin), gpio1_pin_(gpio1_pin), gpio2_pin_(gpio2_pin) {
    this->bus_ = parent_bus;
    
    // Inicializácia RDS RT buffra ---
    memset(this->rds_rt_buffer_, ' ', 64);
    this->rds_rt_buffer_[64] = '\0';
    memset(this->rds_ct_buffer_, 0, sizeof(this->rds_ct_buffer_));
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
        if (state) {
            // Po zapnutí chvíľu počkáme a potom publikujeme počiatočný stav
            // (Týmto sa rieši počiatočný UNKNOWN stav)
            delay(100); // Krátka pauza na stabilizáciu čipu
            this->parent_->read_registers();
            this->parent_->publish_stereo_status();
        }
    }
}

// =============================================================================
// NOVÉ: Implementácia Switch (Mute)
// =============================================================================

void Si4703MuteSwitch::write_state(bool state) {
    bool success;
    // state = true -> MUTE ON (Stlmiť)
    // state = false -> MUTE OFF (Zapnúť zvuk)
    if (state) {
        success = this->parent_->mute();
    } else {
        success = this->parent_->unmute(true);
    }
    
    if (success) {
        this->publish_state(state);
    }
}

// GPIO1 výstup ako prepínač zosilňovača (HIGH/LOW cez SYSCONFIG1 bits 1:0)
void Si4703AmpSwitch::write_state(bool state) {
    if (this->parent_->set_gpio1_state(state)) {
        this->publish_state(state);
    }
}

// GPIO2 výstup ako prepínač (HIGH/LOW cez SYSCONFIG1 bits 3:2)
void Si4703Gpio2Switch::write_state(bool state) {
    if (this->parent_->set_gpio2_state(state)) {
        this->publish_state(state);
    }
}

// =============================================================================
// Implementácia Button (Seek Up/Down)
// =============================================================================

void Si4703SeekUpButton::press_action() { this->parent_->seek_up(); }
void Si4703SeekDownButton::press_action() { this->parent_->seek_down(); }

// =============================================================================
// Power / Mute
// =============================================================================

bool Si4703FM::turn_on() {
    ESP_LOGV(TAG, "Zapínam rádio (Power ON).");
    if (!this->si4703_init()) return false; 
    return true;
}

bool Si4703FM::turn_off() {
    ESP_LOGV(TAG, "Vypínam rádio (Power OFF).");
    if (!this->read_registers()) return false; 
    
    // 2. Modifikácia: Nastav bit DISABLE (D6) a vymaž ENABLE bit (D0)
    // Bit DISABLE (D6, 0x0040) je oficiálny spôsob, ako dať čip do power-down módu.
    this->registers_[POWERCFG] |= 0x0040;  // Nastav DISABLE (D6)
    this->registers_[POWERCFG] &= ~0x0001; // Vymaž ENABLE (D0)
    
    return this->update_all_registers();
}

//---------------------------------------------------------------
// implementacia mute nefunguje spravne 
// NOVÉ: Softvérové MUTE (DMUTE = 0 & SMUTE = 1)
bool Si4703FM::mute() {
    // 1. Získame aktuálnu hlasitosť a uložíme ju (pre požiadavku obnovenia v HA)
    this->previous_volume_ = this->get_volume();
    
    // 2. Nastavíme DMUTE bit (Bit 14) na 0 (Mute Enable)
    if (!this->read_registers()) return false;
    
    // DMUTE je Bit 14 (0x4000)
    this->registers_[POWERCFG] &= ~0x4000; // Nastav DMUTE = 0 (Mute zapnuté) 
    
    if (!this->update_all_registers()) {
        ESP_LOGE(TAG, "MUTE: Zápis registra POWERCFG zlyhal.");
        return false;
    }

    ESP_LOGI(TAG, "Mute ZAPNUTÉ (DMUTE=0). Posledná hlasitosť: %d", this->previous_volume_);
    
    // 3. NEPUBLIKUJEME zmenu hlasitosti na 0. 
    // Home Assistant si ponechá poslednú hodnotu hlasitosti na posuvníku.
    
    return true;
}

// =============================================================================
// Implementácia Si4703FM (Mute/Unmute)
// =============================================================================

// Hardvérové UNMUTE (použitím DMUTE bitu)
// ZMENA: Pridaný parameter restore_volume
bool Si4703FM::unmute(bool restore_volume) { 
    // 1. Nastavíme DMUTE bit (Bit 14) na 1 (Mute Disable)
    if (!this->read_registers()) return false;

    // DMUTE je Bit 14 (0x4000)
    this->registers_[POWERCFG] |= 0x4000; // Nastav DMUTE = 1 (Mute vypnuté)

    if (!this->update_all_registers()) {
        ESP_LOGE(TAG, "UNMUTE: Zápis registra POWERCFG zlyhal.");
        return false;
    }

    ESP_LOGI(TAG, "Mute VYPNUTÉ (DMUTE=1).");
    
    // 2. Zabezpečíme, že Home Assistant vidí správnu (nemenenú) hlasitosť
    // KONTROLA PARAMETRA: Publikujeme hlasitosť len vtedy, ak bol mute vypnutý prepínačom
    if (restore_volume && this->volume_number_ != nullptr) {
        ESP_LOGI(TAG, "Obnovujem stav hlasitosti v HA na %d.", this->previous_volume_);
        // Obnoví Home Assistant posuvník hlasitosti na pôvodnú hodnotu
        this->volume_number_->publish_state((float)this->previous_volume_);
    }
    
    return true;
}

// =============================================================================
// Implementácia Number (Frequency)
// ... (zostáva bezo zmien)

void Si4703Frequency::setup() {
    if (!std::isnan(this->state)) { 
        this->publish_state(this->state);
        //ESP_LOGD(TAG, "Frequency setup: Obnovená frekvencia je %.1f MHz", this->state);
    }
}

void Si4703Frequency::control(float freq_mhz) {
    this->parent_->set_channel_from_float(freq_mhz);
}

// =============================================================================
// Implementácia Number (Volume)
// ... (zostáva bezo zmien)

void Si4703Volume::setup() {
    // 1. Priorita: Hodnota obnovená zo súboru (ak existuje)
    if (!std::isnan(this->state)) {
        this->publish_state(this->state);
        //ESP_LOGD(TAG, "Volume setup: Obnovená hlasitosť je %.0f", this->state);
    }
}
void Si4703Volume::control(float new_volume) {
    this->parent_->set_volume((uint8_t)std::round(new_volume));
}

// =============================================================================
// Implementácia Publikovania Stereo Statusu
// =============================================================================

void Si4703FM::publish_stereo_status() {
    // -------------------------------------------------------------------------
    // Stereo Indicator (Binary Sensor)
    // -------------------------------------------------------------------------
    if (this->stereo_indicator_sensor_ != nullptr) {
        // Bit 8 (ST) v 0x0A (STATUSRSSI) je 1 pre Stereo, 0 pre Mono
        bool is_stereo = this->registers_[STATUSRSSI] & 0x0100; // 0x0100 je Bit 8

        // OPRAVA: Robustnejšie publikovanie stavu (vrátane inicializácie z UNKNOWN)
        if (!this->stereo_indicator_sensor_->has_state() || this->stereo_indicator_sensor_->get_state() != is_stereo) {
            ESP_LOGD(TAG, "Stereo Indicator: Publikujem stav: %s (Stereo: %d)", ONOFF(is_stereo), is_stereo);
            this->stereo_indicator_sensor_->publish_state(is_stereo);
        }
    }
}

// =============================================================================
// Setup
// =============================================================================

void Si4703FM::setup() {
    // 1. I/O Pin setup
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    if (this->stc_int_pin_ != nullptr) this->stc_int_pin_->setup();
    this->reset_pin_->digital_write(true); 
    
    this->gpio2_last_state_ = false;
    this->gpio2_last_change_ = millis();
   
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
    
    ESP_LOGD(TAG, "Si4703 inicialized.");
}

// --- Helpers for volume, etc. ---

uint8_t Si4703FM::get_volume() { return this->registers_[SYSCONFIG2] & 0x0F; }

// =============================================================================
// Implementácia Si4703FM (Hlasitosť)
// =============================================================================
bool Si4703FM::set_volume(uint8_t volume) {
    if (volume > 15) {
        ESP_LOGE(TAG, "Neplatná hodnota hlasitosti: %d. Používam 15.", volume);
        volume = 15;
    }

    // NOVÉ: Ak je MUTE aktívne a mení sa hlasitosť, automaticky zruš MUTE.
    if (this->mute_switch_ != nullptr && this->mute_switch_->state) {
        ESP_LOGI(TAG, "Detekovaná zmena hlasitosti počas MUTE. Automatické UNMUTE.");
        
        // 1. Zruš Mute na HW (DMUTE=1). 
        // DÔLEŽITÉ: restore_volume=false, aby sa nepublikovala stará hlasitosť (zastavenie rekurzie).
        this->unmute(false); 
        
        // 2. Aktualizuj stav Mute Switchu v HA
        // Nastavíme lokálny stav prepínača na false PRED publikovaním, aby sa predišlo rekurzii.
        this->mute_switch_->state = false; 
        this->mute_switch_->publish_state(false); 
    }
    
    // 3. Nastav novú hlasitosť v HW
    if (!this->read_registers()) return false;

    // SYSCONFIG2 (0x05) | Volume: Bits 0-3
    this->registers_[SYSCONFIG2] &= ~0x000F; 
    this->registers_[SYSCONFIG2] |= (uint16_t)volume;
    if (!this->update_all_registers()) return false;

    this->volume_ = volume; 
    if (this->volume_number_ != nullptr) {
        this->volume_number_->publish_state((float)volume);
    }
    ESP_LOGD(TAG, "Hlasitosť nastavená na %d.", volume);
    return true;
}

float Si4703FM::get_rssi() {
    // RSSI je uložené v registri STATUSRSSI (REG 0x0A) v bitoch 0-5. Max 63.
    //uint8_t rssi_val = this->registers_[STATUSRSSI] & 0x3F; 
    uint8_t rssi_val = this->registers_[STATUSRSSI] & 0xFF; 
    return (float)rssi_val;
}

float Si4703FM::get_snr() {
    // SNR je uložené v registri STATUSRSSI (REG 0x0A) v bitoch 6-11.
    // Max 63 (6 bitov).
    // Posunieme doprava 6 bitov, aby sme sa dostali k SNR, a maskujeme 6 bitov (0x3F).
    uint8_t snr_val = (this->registers_[STATUSRSSI] >> 6) & 0x3F; 
    return (float)snr_val;
}

float Si4703FM::get_pty_code() {
    // PTY sa číta z Bloku B, ale malo by sa čítať len z platného Group 0A/0B paketu.
    // Túto funkciu by sme nemali volať priamo, slúži len ako fallback.
    uint8_t pty_val = (this->registers_[RDSB] >> 5) & 0x1F;
    return (float)pty_val;
}

// =============================================================================
// Loop
// =============================================================================
void Si4703FM::loop() {
    if (millis() - this->last_update_ > this->update_interval_) {
        this->last_update_ = millis();
        
        // Čítame registre iba raz za cyklus
        if (!this->read_registers()) {
            ESP_LOGW(TAG, "Loop: read register I2C failed.");
            return;
        }

        // 1. Aktualizácia frekvencie (ak nie je ovládaná)
        if (this->frequency_number_ != nullptr) {
            float freq = this->get_current_frequency_mhz();
            if (freq > 0 && this->frequency_number_->state != freq) {
                ESP_LOGV(TAG, "Loop: Aktualizácia frekvencie na %.1f MHz", freq);
                this->frequency_number_->publish_state(freq);
            }
        }
        
        // 2. Publikovanie RSSI
        if (this->rssi_sensor_ != nullptr) {
            // Použijeme dáta z read_registers(), ktoré sme už volali
            float rssi = this->get_rssi(); 
            ESP_LOGV(TAG, "Aktuálne namerané RSSI (REG 0x0A): %.0f dBµV", rssi);

            if (rssi > 0 && (this->rssi_sensor_->state != rssi || !this->rssi_sensor_->has_state())) { 
                ESP_LOGV(TAG, "Loop: Publikovanie RSSI: %.0f dBµV", rssi);
                this->rssi_sensor_->publish_state(rssi);
            }
        }

        // NOVÉ: Publikovanie SNR
        if (this->snr_sensor_ != nullptr) {
            float snr = this->get_snr();
            if (snr > 0 && (this->snr_sensor_->state != snr || !this->snr_sensor_->has_state())) {
                ESP_LOGV(TAG, "Loop: Publikovanie SNR: %.0f", snr);
                this->snr_sensor_->publish_state(snr);
            }
        }
        		
        // 3. Publikovanie Hlasitosti (ak nie je ovládaná)
        if (this->volume_number_ != nullptr) {
            uint8_t vol = this->get_volume();
            if (this->volume_number_->state != (float)vol || !this->volume_number_->has_state()) {
                ESP_LOGV(TAG, "Loop: Publikovanie Hlasitosti: %d", vol);
                this->volume_number_->publish_state((float)vol);
            }
        }

        // 4. Spracovanie RDS (NOVÉ)
        this->process_rds();

        // 5. Stereo Indicator (Binary Sensor) - aktualizácia len každých 300 ms
        if (millis() - this->last_stereo_update_ >= STEREO_UPDATE_INTERVAL) {
            this->publish_stereo_status();
            this->last_stereo_update_ = millis();
        }
    }

    // Pollovanie GPIO2 (hardware switch) – TERAZ už len cez I2C (nie ESP pin)
    this->poll_gpio2_input();
}

// =============================================================================
// Inicializácia čipu
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
    ESP_LOGD(TAG, "Si4703 find.");

    if (!this->read_registers()) return false;

    // 1. Povolíme oscilátor (TEST1 = 0x8100)
    this->registers_[TEST1] = 0x8100;
    if (!this->update_all_registers()) return false;
    delay(500); // Oscilátor potrebuje stabilizovať

    // 2. Povolíme čip pre konfiguráciu (POWERCFG = 0x4400)
    if (!this->read_registers()) return false;
    this->registers_[POWERCFG] = 0x4080; // DMUTE, MONO, BEZ ENABLE (na začiatku)
    
    // --- Povolenie RDS ---
    this->registers_[SYSCONFIG1] |= (1 << 12);  // správny bit pre RDS Enable [cite: 635]
    this->registers_[SYSCONFIG1] |= (1 << 10);  // AGCD enable
    this->registers_[SYSCONFIG1] &= ~(1 << 11); // DE=0 (Europe 50us)
	
    if (!this->update_all_registers()) return false;
    delay(500);

    // 3. Nastavenie pásma a hlasitosti (POUŽITIE OBNOVENÝCH HODNÔT)
    if (!this->read_registers()) return false;

    // --- Nastavenie kritických prahov pre Slabý Signál ---
    
    // A) SYSCONFIG1 (0x04):
    // Vyčistíme bity 0-3 (Stereo Blend Threshold - STC_STRENGTH) a Bit 11 (DE)
    this->registers_[SYSCONFIG1] &= ~0x080F; 
    this->registers_[SYSCONFIG1] &= ~0x00C0;
    this->registers_[SYSCONFIG1] |= 0x0080;  
    
    // OPRAVA: Nastavenie prahu BLNDADJ (Stereo Blend Level) pre stabilizáciu Stereo/Mono 
    // Používame BLNDADJ[1:0] (Bity 7:6) v SYSCONFIG1 (0x04)
    // Predvolená hodnota 00 = 31–49 dBµV 
    // Nastavíme na 10 = 19–37 dBµV, aby sa stabilizoval príjem 
    this->registers_[SYSCONFIG1] &= ~0x00C0; // Vyčistíme bity 7:6 (BLNDADJ)
    this->registers_[SYSCONFIG1] |= 0x0080;  // Nastavíme bity na 10b (0x80)
    
    // B) SYSCONFIG2 (0x05): Nastavenie RSSI Mute Threshold (RSMSSI) na 0
    // Vyčistíme bity 8-15 (RSMSSI)
    this->registers_[SYSCONFIG2] &= 0x00FF; 
    
    // Kanálový krok
    this->registers_[SYSCONFIG2] &= 0xFF0F; // Vymažeme bity 4-7 pre kanálový krok
    this->registers_[SYSCONFIG2] |= (1 << 4); // Nastavíme 100kHz krok (pre Európu)

    // --- Získanie počiatočnej frekvencie a hlasitosti z Number entít ---
    float initial_freq_mhz = 104.8f; 
    uint8_t initial_volume = 8;
    
    if (this->frequency_number_ != nullptr && !std::isnan(this->frequency_number_->state)) {
        initial_freq_mhz = this->frequency_number_->state;
    }
    if (this->volume_number_ != nullptr && !std::isnan(this->volume_number_->state)) {
        initial_volume = (uint8_t)std::round(this->volume_number_->state);
    }
    
    // Ochrana rozsahu hlasitosti 0-15
    if (initial_volume > 15) initial_volume = 15;
    
    // Nastavenie hlasitosti do registra SYSCONFIG2
    this->registers_[SYSCONFIG2] &= 0xFFF0;
    this->registers_[SYSCONFIG2] |= initial_volume;
	
    // Nastavme GPIO1/2 default LOW (00 = hi-z, 10 = low, 11 = high)
    this->registers_[SYSCONFIG1] &= ~0x000F; // clear GPIO1/2 bits
    // default LOW on both: 10 (0b10) -> bits 3:2 and 1:0
    this->registers_[SYSCONFIG1] |= (0b10 << 2); // GPIO2 low
    this->registers_[SYSCONFIG1] |= (0b10);      // GPIO1 low
	
    // SYSCONFIG3 left untouched (not used for GPIO config on SI4703)
    this->registers_[SYSCONFIG3] = 0x0000;
    if (this->stc_int_pin_ != nullptr) {
        this->registers_[SYSCONFIG3] |= (1 << 2); // STC interrupt enable
    }

    if (!this->update_all_registers()) return false;

    // 4. Nastavenie frekvencie (Tento krok musí zapnúť aj ENABLE bit!)
    uint16_t freq_x10 = (uint16_t)std::round(initial_freq_mhz * 10.0f);
    uint16_t initial_channel = (freq_x10 - 875); 

    ESP_LOGI(TAG, "Nastavujem počiatočnú/obnovenú frekvenciu %.1f MHz...", initial_freq_mhz);
    this->tune_to_channel(initial_channel);

    if (!this->read_registers()) return false;

    float freq = this->get_current_frequency_mhz();
    float rssi = this->get_rssi();

    ESP_LOGI(TAG, "Si4703 inicialized. Freq: %.1f MHz, RSSI: %.0f dBµV, Volume: %d", freq, rssi, initial_volume);
    
    if (this->frequency_number_ != nullptr) this->frequency_number_->publish_state(freq);
    if (this->rssi_sensor_ != nullptr) this->rssi_sensor_->publish_state(rssi);
    if (this->volume_number_ != nullptr) this->volume_number_->publish_state((float)initial_volume);

    return true;
}

// =============================================================================
// Tune / Seek
// =============================================================================

//void Si4703FM::set_channel_from_float(float freq_mhz) {
//    uint16_t freq_x10 = (uint16_t)std::round(freq_mhz * 10.0f);
//    uint16_t channel_val = (freq_x10 - 875); 
//    
//    ESP_LOGV(TAG, "Ladenie na %.1f MHz (Hodnota registra: %d)", freq_mhz, channel_val);
//    this->tune_to_channel(channel_val);
//}
//
//bool Si4703FM::tune_to_channel(uint16_t channel_val) {
//    if (!this->read_registers()) return false;
//    
//    // --- ZAPNUTIE ČIPU (ENABLE) ---
//    this->registers_[POWERCFG] |= 0x0001; 
//    
//    // Nastavenie kanálu a TUNE bitu
//    this->registers_[CHANNEL] &= 0xFE00; 
//    this->registers_[CHANNEL] |= channel_val; 
//    this->registers_[CHANNEL] |= TUNE; // TUNE je 1 << 15
//    
//    if (!this->update_all_registers()) return false;
//
//    // --- POŽADOVANÁ ÚPRAVA ---
//    // Ihneď po odoslaní príkazu na ladenie vyčistíme staré RDS dáta
//    // a zobrazíme "RDS Sync...". Tým sa splní požiadavka na reset pri zmene stanice.
//    ESP_LOGV(TAG, "Ladenie: Čistím RDS buffre a publikujem 'RDS Sync...'");

void Si4703FM::reset_rds_on_tune() {
    ESP_LOGV(TAG, "Ladenie/Seek: Čistím RDS buffre a publikujem 'RDS Sync...'");
    
    std::string sync_msg = "RDS Sync...";
    
    // Vyčistenie PS (Názov stanice)
    memset(this->rds_ps_buffer_, ' ', 8);
    this->rds_ps_buffer_[8] = '\0';
    if (this->rds_ps_sensor_ != nullptr && this->rds_ps_sensor_->state != sync_msg) {
        this->rds_ps_sensor_->publish_state(sync_msg);
    }

    // Vyčistenie RT (Text)
    memset(this->rds_rt_buffer_, ' ', 64);
    this->rds_rt_buffer_[64] = '\0';
    if (this->rds_text_sensor_ != nullptr && this->rds_text_sensor_->state != sync_msg) {
        this->rds_text_sensor_->publish_state(sync_msg);
    }
	
	// Vyčistenie CT (Čas)
    memset(this->rds_ct_buffer_, 0, sizeof(this->rds_ct_buffer_));
    if (this->rds_ct_sensor_ != nullptr && this->rds_ct_sensor_->state != sync_msg) {
        this->rds_ct_sensor_->publish_state(sync_msg);
    }
	
    // Vynulujeme aj A/B príznaky, aby sa vynútilo úplné načítanie
    this->rds_ps_last_ab_flag_ = 0;
    this->rds_rt_last_ab_flag_ = 0;

    // Vynulovanie TP/TA (vynútenie OFF pri ladení)
    if (this->tp_indicator_sensor_ != nullptr && this->tp_indicator_sensor_->has_state()) {
        this->tp_indicator_sensor_->publish_state(false);
    }
    if (this->ta_indicator_sensor_ != nullptr && this->ta_indicator_sensor_->has_state()) {
        this->ta_indicator_sensor_->publish_state(false);
    }
}


void Si4703FM::set_channel_from_float(float freq_mhz) {
    uint16_t freq_x10 = (uint16_t)std::round(freq_mhz * 10.0f);
    uint16_t channel_val = (freq_x10 - 875); 
    
    ESP_LOGV(TAG, "Ladenie na %.1f MHz (Hodnota registra: %d)", freq_mhz, channel_val);
    this->tune_to_channel(channel_val);
}

bool Si4703FM::tune_to_channel(uint16_t channel_val) {
    if (!this->read_registers()) return false;
    
    // --- ZAPNUTIE ČIPU (ENABLE) ---
    this->registers_[POWERCFG] |= 0x0001; 
    
    // Nastavenie kanálu a TUNE bitu
    this->registers_[CHANNEL] &= 0xFE00; 
    this->registers_[CHANNEL] |= channel_val; 
    this->registers_[CHANNEL] |= TUNE; // TUNE je 1 << 15
    
    if (!this->update_all_registers()) return false;

    // --- POŽADOVANÁ ÚPRAVA (Refaktorovaná) ---
    // Ihneď po odoslaní príkazu na ladenie vyčistíme staré RDS dáta
    this->reset_rds_on_tune();

    // Pevná pauza 200 ms
    //ESP_LOGD(TAG, "Cakam pevne 200ms na dokoncenie ladenia...");
    delay(200);

    // Vyčistíme TUNE bit (a opäť odosielame POWERCFG s ENABLE)
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
    // Aktualizujeme aj RSSI a Hlasitosť po ladení
    if (this->rssi_sensor_ != nullptr) {
        this->rssi_sensor_->publish_state(this->get_rssi());
    }
    if (this->volume_number_ != nullptr) {
        this->volume_number_->publish_state((float)this->get_volume());
    }

    return true;
}

void Si4703FM::seek_up() { this->seek_internal(true); }
void Si4703FM::seek_down() { this->seek_internal(false); }

void Si4703FM::seek_internal(bool seek_up) {
    ESP_LOGD(TAG, "Spúšťam Seek (Smer: %s)", seek_up ? "UP" : "DOWN");

    if (!this->read_registers()) return;

    // 1. Nastavenie bitov pre Seek v POWERCFG (0x02)
    // Bit 10 (SKMODE) = 1 (Stop at band limits - POŽIADAVKA)
    this->registers_[POWERCFG] |= (1 << 10); 
    
    // Bit 9 (SEEKUP) = 1 (Up) alebo 0 (Down)
    if (seek_up) {
        this->registers_[POWERCFG] |= (1 << 9);
    } else {
        this->registers_[POWERCFG] &= ~(1 << 9);
    }
    
    // Bit 8 (SEEK) = 1 (Začať Seek)
    this->registers_[POWERCFG] |= (1 << 8);

    if (!this->update_all_registers()) {
        ESP_LOGE(TAG, "Seek zlyhal (zápis POWERCFG).");
        return;
    }

    // 2. Čakanie na dokončenie (Polling bitu STC)
    uint32_t start_time = millis();
    bool seek_complete = false;
    
    while (millis() - start_time < 3000) { // Max 3 sekundy timeout
        if (this->read_registers()) {
            // Bit 14 (STC) v STATUSRSSI (0x0A)
            if (this->registers_[STATUSRSSI] & STC) { 
                seek_complete = true;
                break; // Seek je dokončený
            }
        }
        delay(50); // Krátka pauza pred ďalším čítaním
    }

    // 3. Vyčistenie Seek bitu
    if (!this->read_registers()) return; // Znovu načítame finálny stav
    
    this->registers_[POWERCFG] &= ~(1 << 8); // SEEK = 0 (Ukončiť Seek mód)
    if (!this->update_all_registers()) {
        ESP_LOGE(TAG, "Seek zlyhal (vyčistenie SEEK bitu).");
    }

    // 4. Spracovanie výsledkov
    if (seek_complete) {
        float new_freq = this->get_current_frequency_mhz();
        ESP_LOGD(TAG, "Seek dokončený. Nájdená frekvencia: %.1f MHz", new_freq);

        // Bit 13 (SFBL) - Ak je 1, seek zlyhal alebo dosiahol hranicu
        if (this->registers_[STATUSRSSI] & SFBL) {
             ESP_LOGD(TAG, "Seek dosiahol hranicu pásma (SFBL=1).");
        }

        // --- POŽIADAVKA: Aktualizácia 'number' entity ---
        if (this->frequency_number_ != nullptr) {
            this->frequency_number_->publish_state(new_freq);
        }
        
        // Aktualizujeme aj RSSI a Hlasitosť
        if (this->rssi_sensor_ != nullptr) {
            this->rssi_sensor_->publish_state(this->get_rssi());
        }
        if (this->volume_number_ != nullptr) {
            this->volume_number_->publish_state((float)this->get_volume());
        }
        
        // --- POŽIADAVKA: Resetovanie RDS ---
        this->reset_rds_on_tune();

    } else {
        ESP_LOGW(TAG, "Seek zlyhal (timeout).");
    }
}


float Si4703FM::get_current_frequency_mhz() {
    // Používame už načítané registre, nevoláme read_registers() znova
    uint16_t channel_val = this->registers_[READCHANNEL] & 0x03FF;
    float freq_mhz = ((float)channel_val * 0.1f) + 87.5f;
    return freq_mhz;
}

// =============================================================================
// RDS spracovanie (nezmenené, skrátené pre relevantné úpravy)
// =============================================================================

void Si4703FM::process_rds() {
    // 1. Pracujeme s dátami, ktoré sú už v this->registers_
    uint16_t status_word = this->registers_[STATUSRSSI];
    
    // RDSR (Bit 15 - RDS Ready), RDSS (Bit 11 - RDS Synchronized)
    bool rds_ready = status_word & RDSR; 
    bool rds_sync = status_word & RDSS;

    ESP_LOGV(TAG, "RDS Check: RDSR=%d RDSS=%d", rds_ready, rds_sync);

    // 2. Kontrola stavu
    if (rds_ready) {
        // Dáta sú pripravené (RDSR=1). 
        // Dekódujeme ich.
        this->decode_rds_data(
            this->registers_[RDSA],
            this->registers_[RDSB],
            this->registers_[RDSC],
            this->registers_[RDSD]
        );
        
    } else if (!rds_sync) {
        // --- POŽADOVANÁ ÚPRAVA ---
        // Dáta nie sú pripravené (RDSR=0) A nie je synchronizácia (RDSS=0).
        
        // Zobrazíme "RDS Sync..." IBA vtedy, ak senzor nemá žiadny stav (t.j. pri prvom spustení).
        // Tým sa zabráni preblikávaniu, keď už máme platný text.
        std::string sync_msg = "RDS Sync...";
        
        // Publikujeme "RDS Sync..." len ak senzor je úplne prázdny (po štarte)
        if (this->rds_text_sensor_ != nullptr && !this->rds_text_sensor_->has_state()) {
             this->rds_text_sensor_->publish_state(sync_msg);
        }
		
        // Podobná logika pre PS senzor (Názov stanice)
        if (this->rds_ps_sensor_ != nullptr && !this->rds_ps_sensor_->has_state()) {
            this->rds_ps_sensor_->publish_state(sync_msg);
        }
		
	    //  Reset CT senzora pri strate synchronizácie
        if (this->rds_ct_sensor_ != nullptr && !this->rds_ct_sensor_->has_state()) {
            this->rds_ct_sensor_->publish_state(sync_msg);
        }
    }
}

void Si4703FM::decode_rds_data(uint16_t block_a, uint16_t block_b, uint16_t block_c, uint16_t block_d) {
    // -------------------------------------------------------------------------
    // 0. Spracovanie TP (Traffic Programme) a TA (Traffic Announcement)
    // -------------------------------------------------------------------------
    //bool is_tp_enabled = (block_b & 0x0020);  // Bit 5 v Bloku B
    //bool is_ta_active  = (block_b & 0x0010);  // Bit 4 v Bloku B
	//
    //if (this->tp_indicator_sensor_ != nullptr) {
    //    if (!this->tp_indicator_sensor_->has_state() || this->tp_indicator_sensor_->get_state() != is_tp_enabled) {
    //        ESP_LOGD(TAG, "RDS TP: Publikujem stav: %s", ONOFF(is_tp_enabled));
    //        this->tp_indicator_sensor_->publish_state(is_tp_enabled);
    //    }
    //}
	//
    //if (this->ta_indicator_sensor_ != nullptr) {
    //    if (!this->ta_indicator_sensor_->has_state() || this->ta_indicator_sensor_->get_state() != is_ta_active) {
    //        ESP_LOGD(TAG, "RDS TA: Publikujem stav: %s", ONOFF(is_ta_active));
    //        this->ta_indicator_sensor_->publish_state(is_ta_active);
    //    }
    //}
		
	// Skupina (Group Type) je v Bloku B (B15-B12)
    uint8_t group_type = (block_b >> 12) & 0x0F;
    
    // -------------------------------------------------------------------------
    // 1. Spracovanie Program Service (PS) - Group Type 0A
    // -------------------------------------------------------------------------
    if (group_type == 0x00 && !((block_b >> 11) & 0x01)) { // Kontrola Group Type 0A (M bit = 0)

        // Spracovanie TP a TA (z Bloku B) ---
        // TP (Traffic Programme) je Bit 5 v Bloku B (Mask 0x0020)
        bool is_tp_enabled = (block_b & 0x0020); 
        if (this->tp_indicator_sensor_ != nullptr) {
            // Publikovanie musí byť robustné, aby sa preplo z UNKNOWN na OFF (false)
            if (!this->tp_indicator_sensor_->has_state() || this->tp_indicator_sensor_->get_state() != is_tp_enabled) {
                ESP_LOGD(TAG, "RDS TP: Publikujem stav: %s (TP bit 5: %d)", ONOFF(is_tp_enabled), is_tp_enabled);
                this->tp_indicator_sensor_->publish_state(is_tp_enabled);
            }
        }
        
        // TA (Traffic Announcement) je Bit 4 v Bloku B (Mask 0x0010)
        bool is_ta_active = (block_b & 0x0010);
        if (this->ta_indicator_sensor_ != nullptr) {
            // Publikovanie musí byť robustné, aby sa preplo z UNKNOWN na OFF (false)
            if (!this->ta_indicator_sensor_->has_state() || this->ta_indicator_sensor_->get_state() != is_ta_active) {
                ESP_LOGD(TAG, "RDS TA: Publikujem stav: %s", ONOFF(is_ta_active));
                this->ta_indicator_sensor_->publish_state(is_ta_active);
            }
        }

        // --- Spracovanie PTY (z Bloku B) ---
        // PTY (Programme Type) sú Bity 9:5 v Bloku B (Mask 0x03E0)
        float pty_code = (block_b >> 5) & 0x1F;
        if (this->pty_sensor_ != nullptr && this->pty_sensor_->state != pty_code) {
             ESP_LOGV(TAG, "RDS: Publikovanie PTY kódu: %.0f", pty_code);
             this->pty_sensor_->publish_state(pty_code);
        }

        // --- Spracovanie PS (Názov stanice) ---
        uint8_t ab_flag = (block_b >> 4) & 0x01;
        uint8_t ps_segment_address = block_b & 0x03; // Adresa segmentu PS (0, 1, 2, alebo 3)
              
        // Pozícia v 8-znakovom buffri (0, 2, 4, 6)
        uint8_t pos = ps_segment_address * 2; 
    
        // 1. Kontrola zmeny AB flagu
        if (ab_flag != this->rds_ps_last_ab_flag_) {
            this->rds_ps_last_ab_flag_ = ab_flag;
            memset(this->rds_ps_buffer_, ' ', 8); // Vynulovanie celého 8-znakového buffra
            this->rds_ps_buffer_[8] = '\0';
            ESP_LOGD(TAG, "RDS: Detekovaný nový PS (0A) - Reset buffra");

            // Pri detekcii nového PS môžeme tiež publikovať "načítavanie"
            if (this->rds_ps_sensor_ != nullptr) {
                this->rds_ps_sensor_->publish_state("RDS: Načítava sa...");
            }
        }
        
        // 2. Extrakcia 2 znakov a uloženie na správnu pozíciu
        if (pos < 8) {
            this->rds_ps_buffer_[pos]     = (char)((block_d >> 8) & 0xFF); // Prvý znak (vyšší byte z Bloku D)
            this->rds_ps_buffer_[pos + 1] = (char)(block_d & 0xFF);       // Druhý znak (nižší byte z Bloku D)
        }
        
        // 3. Publikovanie PS (Názov stanice)
        std::string current_ps(this->rds_ps_buffer_);
        
        size_t end = current_ps.find_last_not_of(' ');
        if (std::string::npos != end) {
            current_ps.resize(end + 1);
        }
        
        for (char &c : current_ps) {
            if (static_cast<unsigned char>(c) < 0x20 || static_cast<unsigned char>(c) > 0x7E) {
                c = ' ';
            }
        }
        
        if (this->rds_ps_sensor_ != nullptr && this->rds_ps_sensor_->state != current_ps) {
            ESP_LOGD(TAG, "RDS: Publikujem PS (Názov rádia): %s", current_ps.c_str());
            this->rds_ps_sensor_->publish_state(current_ps);
        }
    }


    // -------------------------------------------------------------------------
    // 2. Spracovanie Radio Text (RT) - Group Type 2A
    // -------------------------------------------------------------------------
    if (group_type == 0x02 && !((block_b >> 11) & 0x01)) { // Kontrola Group Type 2A (M bit = 0)
        
        uint8_t ab_flag = (block_b >> 4) & 0x01;        // A/B flag (B4)
        uint8_t text_segment_address = block_b & 0x0F;  // Adresa segmentu (B0-B3)

        // 1. Kontrola zmeny AB flagu (signál pre nový RT)
        if (ab_flag != this->rds_rt_last_ab_flag_) {
            this->rds_rt_last_ab_flag_ = ab_flag;
            memset(this->rds_rt_buffer_, ' ', 64); 
            this->rds_rt_buffer_[64] = '\0';
            ESP_LOGD(TAG, "RDS: Detekovaný nový RT (2A) - Reset buffra");

            // Keď sa zistí nový text, okamžite publikujeme "načítavanie",
            // aby sme vymazali starý text.
            if (this->rds_text_sensor_ != nullptr) {
                this->rds_text_sensor_->publish_state("RDS: Načítava sa...");
            }
        }
        
        // 2. Extrakcia 4 znakov
        char char1 = (char)((block_c >> 8) & 0xFF); // Znak 1 (C High)
        char char2 = (char)(block_c & 0xFF);        // Znak 2 (C Low)
        char char3 = (char)((block_d >> 8) & 0xFF); // Znak 3 (D High)
        char char4 = (char)(block_d & 0xFF);        // Znak 4 (D Low)

        // 3. Uloženie na správnu pozíciu
        uint8_t pos = text_segment_address * 4; 

        if (pos < 64) {
            this->rds_rt_buffer_[pos] = char1;
            if (pos + 1 < 64) {
                this->rds_rt_buffer_[pos + 1] = char2;
            }
            if (pos + 2 < 64) {
                this->rds_rt_buffer_[pos + 2] = char3;
            }
            if (pos + 3 < 64) {
                this->rds_rt_buffer_[pos + 3] = char4;
            }
            
            // 4. Publikovanie stavu
            std::string current_rt(this->rds_rt_buffer_);
            size_t end = current_rt.find_last_not_of(' ');
            if (std::string::npos != end) {
                current_rt.resize(end + 1);
            }
			
			for (char &c : current_rt) {
                // Filtrujeme znaky mimo rozsahu 0x20 (medzera) až 0x7E (~)
                if (static_cast<unsigned char>(c) < 0x20 || static_cast<unsigned char>(c) > 0x7E) {
                    c = ' ';
                }
            }
            
            if (this->rds_text_sensor_ != nullptr && this->rds_text_sensor_->state != current_rt) {
                ESP_LOGD(TAG, "RDS: Publikujem RT: %s", current_rt.c_str());
                this->rds_text_sensor_->publish_state(current_rt);
            }
        }
    }
	
	// -------------------------------------------------------------------------
    // 3. Spracovanie Clock Time (CT) - Group Type 4A
    // -------------------------------------------------------------------------
    if (group_type == 0x04 && !((block_b >> 11) & 0x01)) { // Kontrola Group Type 4A

        // Hodiny UTC (5 bitov): 
        // Bit 4 je v Bloku C (bit 0)
        // Bity 3-0 sú v Bloku D (bity 15-12)
        uint8_t hour_utc = ((block_c & 0x0001) << 4) | ((block_d & 0xF000) >> 12);
        
        // Minúty UTC (6 bitov): 
        // Bity 5-0 sú v Bloku D (bity 11-6)
        uint8_t minute_utc = (block_d & 0x0FC0) >> 6;
        
        // LTO (Local Time Offset) - 6 bitov v Bloku D:
        // Smer posunu (Summer Time) je bit 5 (0 = +, 1 = -)
        // Posun v polhodinových krokoch (5 bitov) je bity 4-0 (LTO)
        
        uint8_t sign = (block_d >> 5) & 0x01; // 0 = +, 1 = -
        uint8_t offset_half_hours = block_d & 0x1F; // 5 bitov pre posun (0-31)
        
        // Prevod posunu na minúty
        int16_t offset_minutes = offset_half_hours * 30;
        
        // Aplikácia znamienka
        if (sign == 1) { // Typicky používané pre Západnú hemisféru, ale v RDS je to pre GMT - LTO
            offset_minutes = -offset_minutes; // Teda ak je LTO napr. +1h, pre CEST je to +2h od UTC
        }
        
        // Aplikácia posunu
        int16_t total_minutes = (hour_utc * 60) + minute_utc + offset_minutes;
        
        // Normalizácia na 24h cyklus
        int local_hour = (total_minutes / 60) % 24;
        int local_minute = total_minutes % 60;

        // Oprava záporných hodnôt po modulovaní (ak je napr. -1:30)
        if (local_minute < 0) {
            local_minute += 60;
            local_hour--;
        }
        if (local_hour < 0) {
            local_hour += 24;
        }

        if (this->rds_ct_sensor_ != nullptr) {
            // Formátujeme čas ako "HH:MM"
            snprintf(this->rds_ct_buffer_, sizeof(this->rds_ct_buffer_), "%02d:%02d", local_hour, local_minute);
            
            std::string current_ct(this->rds_ct_buffer_);

            if (this->rds_ct_sensor_->state != current_ct) {
                ESP_LOGD(TAG, "RDS: Publikujem CT (Lokalizovaný Čas): %s (UTC: %02d:%02d, Offset: %d min.)", 
                    current_ct.c_str(), hour_utc, minute_utc, offset_minutes);
                this->rds_ct_sensor_->publish_state(current_ct);
            }
        }
    }
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

// -----------------------------------------------------------------------------
// GPIO1/2 helper methods – cez SYSCONFIG1 (Register 0x04)
// -----------------------------------------------------------------------------
bool Si4703FM::set_gpio1_state(bool on) {
    // GPIO1[1:0] (bits 1:0): 10 = Low, 11 = High
    if (!this->read_registers()) {
        ESP_LOGE(TAG, "set_gpio1_state: read_registers zlyhalo.");
        return false;
    }

    this->registers_[SYSCONFIG1] &= ~(0b11); // clear bits 1:0
    this->registers_[SYSCONFIG1] |= on ? 0b11 : 0b10;

    if (!this->update_all_registers()) {
        ESP_LOGE(TAG, "set_gpio1_state: update_all_registers zlyhalo.");
        return false;
    }

    this->amp_state_ = on;
    ESP_LOGD(TAG, "GPIO1 (SYSCONFIG1 bits 1:0) nastavené na: %s", ONOFF(on));
    return true;
}

bool Si4703FM::set_gpio2_state(bool on) {
    // GPIO2[1:0] (bits 3:2): 10 = Low, 11 = High
    if (!this->read_registers()) {
        ESP_LOGE(TAG, "set_gpio2_state: read_registers zlyhalo.");
        return false;
    }

    this->registers_[SYSCONFIG1] &= ~(0b11 << 2); // clear bits 3:2
    this->registers_[SYSCONFIG1] |= (on ? 0b11 : 0b10) << 2;

    if (!this->update_all_registers()) {
        ESP_LOGE(TAG, "set_gpio2_state: update_all_registers zlyhalo.");
        return false;
    }

    ESP_LOGD(TAG, "GPIO2 (SYSCONFIG1 bits 3:2) nastavené na: %s", ONOFF(on));
    return true;
}

void Si4703FM::poll_gpio2_input() {
    // If GPIO2 is configured as output (10/11), we skip input polling.
    // If user wired it as input (00 hi-z), STATUSRSSI has no bit for it; so do nothing.
    // Leaving this empty avoids false reads when GPIO2 is used as an output switch.
}

}  // namespace si4703_fm
}  // namespace esphome
