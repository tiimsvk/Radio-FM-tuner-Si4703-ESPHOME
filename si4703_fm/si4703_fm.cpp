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
    
    // --- NOVÉ: Inicializácia RDS RT buffra ---
    memset(this->rds_rt_buffer_, ' ', 64);
    this->rds_rt_buffer_[64] = '\0';
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
// NOVÉ: Implementácia Switch (Mute)
// =============================================================================

void Si4703MuteSwitch::write_state(bool state) {
    bool success;
    // state = true -> MUTE ON (Stlmiť)
    // state = false -> MUTE OFF (Zapnúť zvuk)
    if (state) {
        success = this->parent_->mute();
    } else {
        success = this->parent_->unmute();
    }
    
    if (success) {
        this->publish_state(state);
    }
}

// =============================================================================
// Implementácia Si4703FM (Power ON/OFF a Mute/Unmute)
// =============================================================================

bool Si4703FM::turn_on() {
    ESP_LOGD(TAG, "Zapínam rádio (Power ON).");
    
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

//---------------------------------------------------------------
// implementacia mute nefunguje spravne 
// NOVÉ: Softvérové MUTE (DMUTE = 0 & SMUTE = 1)
bool Si4703FM::mute() {
    // 1. Získame aktuálnu hlasitosť a uložíme ju
    this->previous_volume_ = this->get_volume();
    
    // 2. Nastavíme hlasitosť na 0
    if (!this->set_volume(0)) {
        ESP_LOGE(TAG, "MUTE: Nastavenie hlasitosti na 0 zlyhalo.");
        return false;
    }

    // Voliteľne aktualizujeme aj DMUTE, ale primárne stlmenie je Volume=0
    if (!this->read_registers()) return false;
    this->registers_[POWERCFG] &= ~0x0080; // DMUTE OFF
    this->registers_[POWERCFG] |= 0x0400;  // SMUTE ON
    this->update_all_registers(); 

    ESP_LOGI(TAG, "Mute ZAPNUTÉ (Volume=0). Predchádzajúca hlasitosť: %d", this->previous_volume_);
    
    // Zabezpečíme, že Home Assistant vidí Volume=0
    if (this->volume_number_ != nullptr) {
        this->volume_number_->publish_state(0.0f);
    }
    
    return true;
}

// NOVÉ: Softvérové UNMUTE (Obnoví Volume)
bool Si4703FM::unmute() {
    // 1. Nastavíme hlasitosť na uloženú hodnotu
    if (!this->set_volume(this->previous_volume_)) {
        ESP_LOGE(TAG, "UNMUTE: Obnovenie hlasitosti na %d zlyhalo.", this->previous_volume_);
        return false;
    }

    // Voliteľne aktualizujeme DMUTE, ale primárne zapnutie je Volume>0
    if (!this->read_registers()) return false;
    this->registers_[POWERCFG] |= 0x0080;  // DMUTE ON
    this->registers_[POWERCFG] &= ~0x0400; // SMUTE OFF
    this->update_all_registers(); 

    ESP_LOGI(TAG, "Mute VYPNUTÉ (Volume obnovená na %d).", this->previous_volume_);
    
    // Zabezpečíme, že Home Assistant vidí obnovenú hlasitosť
    if (this->volume_number_ != nullptr) {
        this->volume_number_->publish_state((float)this->previous_volume_);
    }
    
    return true;
}

// =============================================================================
// NOVÉ: Implementácia Switch (Stereo/Mono)
// =============================================================================

void Si4703StereoMonoSwitch::write_state(bool state) {
    // state = true -> MONO ON
    // state = false -> MONO OFF (Stereo ON)
    bool success = this->parent_->set_mono_mode(state);
    
    if (success) {
        this->publish_state(state);
    }
}

// =============================================================================
// Implementácia Si4703FM (Stereo/Mono)
// =============================================================================

// NOVÉ: Nastaví rádio do Mono módu (alebo Stereo, ak je mono_on=false)
bool Si4703FM::set_mono_mode(bool mono_on) {
    if (!this->read_registers()) {
        ESP_LOGE(TAG, "STEREO/MONO: Čítanie registrov zlyhalo.");
        return false; 
    }
    
    // MONO bit je Bit 14 (0x4000) v registri POWERCFG (0x02)
    if (mono_on) {
        this->registers_[POWERCFG] |= 0x4000; // Nastav MONO bit
        ESP_LOGD(TAG, "Režim nastavený na MONO (0x4000 pridaný).");
    } else {
        this->registers_[POWERCFG] &= ~0x4000; // Vymaž MONO bit (Stereo)
        ESP_LOGD(TAG, "Režim nastavený na STEREO (0x4000 odstránený).");
    }

    // Aktualizujeme stav v rámci loopu (až na ďalšom cykle)
    return this->update_all_registers();
}

// NOVÉ: Zistenie stavu (Mono/Stereo)
bool Si4703FM::is_mono_mode() {
    // Čistíme všetky bity okrem Bit 14
    return (this->registers_[POWERCFG] & 0x4000) != 0;
}





//----------------------------------------------------------------------------

// =============================================================================
// Implementácia Number (Frequency)
// ... (zostáva bezo zmien)

void Si4703Frequency::setup() {
    if (!std::isnan(this->state)) { 
        this->publish_state(this->state);
        ESP_LOGD(TAG, "Frequency setup: Obnovená frekvencia je %.1f MHz", this->state);
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
        ESP_LOGD(TAG, "Volume setup: Obnovená hlasitosť je %.0f", this->state);
    }
}
void Si4703Volume::control(float new_volume) {
    this->parent_->set_volume((uint8_t)std::round(new_volume));
}

// =============================================================================
// Implementácia hlavnej triedy Si4703FM (Hub)
// ... (zostáva bezo zmien, okrem si4703_init a tune_to_channel, ktoré sa vrátia k verzii bez anti-POP logiky)

void Si4703FM::setup() {
    // 1. I/O Pin setup
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    
    if (this->stc_int_pin_ != nullptr) {
        this->stc_int_pin_->setup();
    }
    
    this->reset_pin_->digital_write(true); 
    
   
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

float Si4703FM::get_snr() {
    // SNR je uložené v registri STATUSRSSI (REG 0x0A) v bitoch 6-11.
    // Max 63 (6 bitov).
    // Posunieme doprava 6 bitov, aby sme sa dostali k SNR, a maskujeme 6 bitov (0x3F).
    uint8_t snr_val = (this->registers_[STATUSRSSI] >> 6) & 0x3F; 
    return (float)snr_val;
}

float Si4703FM::get_bler_a() {
    // BLER A (Block Error Rate A) je uložený v Bloku A (REG 0x0C) v bitoch 0-1.
    // 0 = 0 chýb, 1 = 1-2 chyby, 2 = 3-5 chýb, 3 = 6+ chýb (Nespoľahlivé)
    uint8_t bler_a_val = this->registers_[RDSA] & 0x03; 
    return (float)bler_a_val;
}

float Si4703FM::get_bler_d() {
    // BLER D (Block Error Rate D) je uložený v Bloku D (REG 0x0F) v bitoch 0-1.
    uint8_t bler_d_val = this->registers_[RDSD] & 0x03; 
    return (float)bler_d_val;
}

// =============================================================================
// RDS Data Processing (NOVÁ VYLEPŠENÁ VERZIA)
// =============================================================================

void Si4703FM::process_rds_data() {
    // Táto funkcia je volaná z loop(), keď je nastavený RDSR bit (dáta pripravené).
    // Registre 0x0C (RDSA), 0x0D (RDSB), 0x0E (RDSC), 0x0F (RDSD)
    // sú už načítané v this->registers_
    
    uint16_t rds_block_a = this->registers_[0x0C];
    uint16_t rds_block_b = this->registers_[0x0D];
    uint16_t rds_block_c = this->registers_[0x0E];
    uint16_t rds_block_d = this->registers_[0x0F];

    // Získame Group Type (Bity 12-15) a Verziu (Bit 11) z Bloku B
    uint8_t group_type = (rds_block_b >> 12) & 0x0F;
    uint8_t version_b = (rds_block_b >> 11) & 0x01;

    // ESP_LOGV(TAG, "RDS Dáta prijaté: GT=%d, B=%04X C=%04X D=%04X", group_type, rds_block_b, rds_block_c, rds_block_d);

    // --- POKROČILÉ PARSOVANIE: Radio Text (RT) ---
    // Radio Text sa posiela v skupinách 2A (version_b=0) alebo 2B (version_b=1)
    
    if (group_type == 2) {
        // Bity 0-3 v Bloku B sú adresa segmentu (0-15)
        uint8_t segment = rds_block_b & 0x0F;
        // Bit 4 v Bloku B je A/B flag, ktorý sa mení pri novej správe
        uint8_t ab_flag = (rds_block_b >> 4) & 0x01;

        // 1. Detekcia novej RT správy (zmena A/B flagu)
        if (ab_flag != this->rds_rt_last_ab_flag_) {
            ESP_LOGD(TAG, "RDS: Detekovaná nová RT správa (A/B flag sa zmenil). Čistím buffer.");
            this->rds_rt_last_ab_flag_ = ab_flag;
            // Vyčistíme buffer (naplníme medzerami)
            memset(this->rds_rt_buffer_, ' ', 64);
            this->rds_rt_buffer_[64] = '\0';
        }

        // 2. Naplnenie buffra podľa verzie skupiny
        if (version_b == 0) { // Group 2A (4 znaky)
            // segment 0-15, každý nesie 4 znaky
            if (segment < 16) {
                uint8_t offset = segment * 4;
                this->rds_rt_buffer_[offset + 0] = (rds_block_c >> 8) & 0xFF; // C (High)
                this->rds_rt_buffer_[offset + 1] = rds_block_c & 0xFF;        // C (Low)
                this->rds_rt_buffer_[offset + 2] = (rds_block_d >> 8) & 0xFF; // D (High)
                this->rds_rt_buffer_[offset + 3] = rds_block_d & 0xFF;        // D (Low)
                ESP_LOGV(TAG, "RDS RT (2A) Segment %d: %c%c%c%c", segment, 
                         this->rds_rt_buffer_[offset + 0], this->rds_rt_buffer_[offset + 1],
                         this->rds_rt_buffer_[offset + 2], this->rds_rt_buffer_[offset + 3]);
            }
        } else { // Group 2B (2 znaky)
            // segment 0-15, každý nesie 2 znaky
            if (segment < 16) {
                uint8_t offset = segment * 2;
                this->rds_rt_buffer_[offset + 0] = (rds_block_d >> 8) & 0xFF; // D (High)
                this->rds_rt_buffer_[offset + 1] = rds_block_d & 0xFF;        // D (Low)
                ESP_LOGV(TAG, "RDS RT (2B) Segment %d: %c%c", segment, 
                         this->rds_rt_buffer_[offset + 0], this->rds_rt_buffer_[offset + 1]);
            }
        }

        // 3. Publikovanie aktuálneho stavu RT buffra
        // Prevedieme C-style string na std::string
        std::string rt_string(this->rds_rt_buffer_);
        
        // Odstránime koncové medzery pre čistejší výstup
        size_t endpos = rt_string.find_last_not_of(' ');
        if (std::string::npos != endpos) {
            rt_string = rt_string.substr(0, endpos + 1);
        }
        
        // Publikujeme, iba ak sa text líši od predchádzajúceho
        if (this->rds_text_sensor_->state != rt_string) {
            ESP_LOGD(TAG, "RDS: Publikujem RT: %s", rt_string.c_str());
            this->rds_text_sensor_->publish_state(rt_string);
        }
        
    } // koniec if (group_type == 2)
}

// =============================================================================
// --- Hlavná slučka (Loop) ---
// =============================================================================
void Si4703FM::loop() {
    if (millis() - this->last_update_ > this->update_interval_) {
        this->last_update_ = millis();
        
        // Čítame registre iba raz za cyklus
        if (!this->read_registers()) {
            ESP_LOGW(TAG, "Loop: Čítanie registrov z I2C zlyhalo. Dáta sa neaktualizujú!");
            return;
        }
        ESP_LOGD(TAG, "RDSR=%d RDSS=%d", 
           (this->registers_[STATUSRSSI] & RDSR) != 0,
           (this->registers_[STATUSRSSI] & RDSS) != 0);
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

        // NOVÉ: Publikovanie SNR
        if (this->snr_sensor_ != nullptr) {
            float snr = this->get_snr();
            if (snr > 0 && (this->snr_sensor_->state != snr || !this->snr_sensor_->has_state())) {
                ESP_LOGD(TAG, "Loop: Publikovanie SNR: %.0f", snr);
                this->snr_sensor_->publish_state(snr);
            }
        }
        
        // NOVÉ: Publikovanie BLER A
        if (this->bler_a_sensor_ != nullptr) {
            float bler_a = this->get_bler_a();
            if (this->bler_a_sensor_->state != bler_a || !this->bler_a_sensor_->has_state()) {
                ESP_LOGD(TAG, "Loop: Publikovanie BLER A: %.0f", bler_a);
                this->bler_a_sensor_->publish_state(bler_a);
            }
        }
        
        // NOVÉ: Publikovanie BLER D
        if (this->bler_d_sensor_ != nullptr) {
            float bler_d = this->get_bler_d();
            if (this->bler_d_sensor_->state != bler_d || !this->bler_d_sensor_->has_state()) {
                ESP_LOGD(TAG, "Loop: Publikovanie BLER D: %.0f", bler_d);
                this->bler_d_sensor_->publish_state(bler_d);
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
        // 4. Publikovanie stavu Stereo/Mono (NOVÉ)
        if (this->stereo_mono_switch_ != nullptr) {
            bool mono_mode_on = this->is_mono_mode();
            if (this->stereo_mono_switch_->state != mono_mode_on) {
                ESP_LOGD(TAG, "Loop: Publikovanie stavu Stereo/Mono: %s", mono_mode_on ? "MONO" : "STEREO");
                this->stereo_mono_switch_->publish_state(mono_mode_on);
            }
        }

        // 5. Spracovanie RDS (NOVÉ)
        this->process_rds();
        //if (this->rds_text_sensor_ != nullptr) {
        //    // Skontrolujeme, či je RDSR (RDS Ready) bit nastavený (Bit 15 v 0x0A)
        //    // a či je RDSS (RDS Synchronized) bit nastavený (Bit 10 v 0x0A)
        //    //if ((this->registers_[STATUSRSSI] & RDSR) && (this->registers_[STATUSRSSI] & RDSS)) {
        //    if (this->registers_[STATUSRSSI] & RDSR) {
        //        // Máme nové dáta a sme synchronizovaní, parsujeme ich
        //        this->process_rds_data();
		//    } else {
        //        // publish status
        //        this->rds_text_sensor_->publish_state("RDS: Waiting...");
        //    }
        //}
		//if (millis() - this->last_rds_check_ >= RDS_CHECK_INTERVAL) {
        //    this->process_rds();
        //    this->last_rds_check_ = millis();
        //}
    }
}

// =============================================================================
// --- Inicializácia čipu (Pôvodná verzia bez anti-POP) ---
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

    // 2. Povolíme čip pre konfiguráciu (POWERCFG = 0x4400)
    if (!this->read_registers()) return false;
    this->registers_[POWERCFG] = 0x4080; // DMUTE, MONO, BEZ ENABLE (na začiatku)
    // --- NOVÉ: Povolenie RDS ---
    // Bit 3 v SYSCONFIG1 (0x04) povoľuje RDS
    ESP_LOGD(TAG, "SYSCONFIG1 pred zapnutím RDS: 0x%04X", this->registers_[SYSCONFIG1]);
    this->registers_[SYSCONFIG1] |= (1 << 12);  // správny bit pre RDS Enable
    ESP_LOGD(TAG, "SYSCONFIG1 po zapnutí RDS: 0x%04X", this->registers_[SYSCONFIG1]);
	
	this->registers_[SYSCONFIG1] |= (1 << 10);  // AGCD enable
    this->registers_[SYSCONFIG1] &= ~(1 << 11); // DE=0 (Europe)
	
    if (!this->update_all_registers()) return false;
	
    delay(500); // Power-up delay

    // 3. Nastavenie pásma a hlasitosti (POUŽITIE OBNOVENÝCH HODNÔT)
    if (!this->read_registers()) return false;
    //this->registers_[SYSCONFIG1] |= (1 << 11); // DE (De-emphasis) = 50us (Európa)
    //this->registers_[SYSCONFIG2] &= 0xFF0F; // Vymažeme bity 4-7 pre kanálový krok
    //this->registers_[SYSCONFIG2] |= (1 << 4); // Nastavíme 100kHz krok (pre Európu)

    // --- Nastavenie kritických prahov pre Slabý Signál ---
    
    // A) SYSCONFIG1 (0x04): Nastavenie Stereo Blend Threshold (STC_STRENGTH) na 0
    // Vyčistíme bity 0-3 (Stereo Blend Threshold) a Bit 11 (DE, ak bol nastavený na 75us)
    this->registers_[SYSCONFIG1] &= ~0x080F; 
    
    // DE (De-emphasis) 50us (Európa) - Bit 11 je 0 (Default)
    // Pridáme len užívateľom definovanú DE-emphasis (ak je 75us, inak 50us zostáva 0)
    // this->registers_[SYSCONFIG1] |= (1 << 11); // Ak by sme chceli 75us (USA)
    
    // B) SYSCONFIG2 (0x05): Nastavenie RSSI Mute Threshold (RSMSSI) na 0
    // Vyčistíme bity 8-15 (RSMSSI)
    this->registers_[SYSCONFIG2] &= 0x00FF; 
    
    // Vaša pôvodná logika pre De-emphasis a kanálový krok:
    //this->registers_[SYSCONFIG1] |= (1 << 11); // Pôvodná hodnota (pravdepodobne 75us)
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
    this->registers_[SYSCONFIG2] &= 0xFFF0; // Vyčistíme staré bity hlasitosti
    this->registers_[SYSCONFIG2] |= initial_volume; // Nastavíme obnovenú hlasitosť
    
    if (this->stc_int_pin_ != nullptr) {
        this->registers_[SYSCONFIG3] |= (1 << 2); // Povolíme STC prerušenie
    }

    if (!this->update_all_registers()) return false; // Pošleme konfiguráciu bez ladenia/zapnutia.

    // 4. Nastavenie frekvencie (Tento krok musí zapnúť aj ENABLE bit!)
    uint16_t freq_x10 = (uint16_t)std::round(initial_freq_mhz * 10.0f);
    uint16_t initial_channel = (freq_x10 - 875); 

    ESP_LOGI(TAG, "Nastavujem počiatočnú/obnovenú frekvenciu %.1f MHz...", initial_freq_mhz);
    this->tune_to_channel(initial_channel); // TUNE_TO_CHANNEL zapne ENABLE bit

    // Finálne čítanie na potvrdenie stavu
    if (!this->read_registers()) return false;

    float freq = this->get_current_frequency_mhz();
    float rssi = this->get_rssi();

    ESP_LOGI(TAG, "Si4703 inicializácia dokončená. Aktuálna frekvencia: %.1f MHz, RSSI: %.0f dBµV, Hlasitosť: %d", freq, rssi, initial_volume);
    
    // Počiatočné publikovanie stavu (už len pre istotu)
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
    
    // --- ZAPNUTIE ČIPU (ENABLE) ---
    this->registers_[POWERCFG] |= 0x0001; 
    
    // Nastavenie kanálu a TUNE bitu
    this->registers_[CHANNEL] &= 0xFE00; 
    this->registers_[CHANNEL] |= channel_val; 
    this->registers_[CHANNEL] |= TUNE; // TUNE je 1 << 15
    
    if (!this->update_all_registers()) return false;

    // --- POŽADOVANÁ ÚPRAVA ---
    // Ihneď po odoslaní príkazu na ladenie vyčistíme staré RDS dáta
    // a zobrazíme "RDS Sync...". Tým sa splní požiadavka na reset pri zmene stanice.
    ESP_LOGD(TAG, "Ladenie: Čistím RDS buffre a publikujem 'RDS Sync...'");
    
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
    // Vynulujeme aj A/B príznaky, aby sa vynútilo úplné načítanie
    this->rds_ps_last_ab_flag_ = 0;
    this->rds_rt_last_ab_flag_ = 0;
    // --- KONIEC ÚPRAVY ---

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

// =============================================================================
// Implementácia RDS spracovania (nové)
// =============================================================================

void Si4703FM::process_rds() {
    // 1. Pracujeme s dátami, ktoré sú už v this->registers_
    uint16_t status_word = this->registers_[STATUSRSSI];
    
    // RDSR (Bit 15 - RDS Ready), RDSS (Bit 11 - RDS Synchronized)
    bool rds_ready = status_word & RDSR; 
    bool rds_sync = status_word & RDSS;

    // Zobrazíme aktuálnu chybovosť pre bloky A a D
    float bler_a = this->get_bler_a();
    float bler_d = this->get_bler_d();
	
    ESP_LOGD(TAG, "RDS Check: RDSR=%d RDSS=%d | BLERA=%.0f, BLERD=%.0f", rds_ready, rds_sync, bler_a, bler_d);

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
    }
}

void Si4703FM::decode_rds_data(uint16_t block_a, uint16_t block_b, uint16_t block_c, uint16_t block_d) {
    // Skupina (Group Type) je v Bloku B (B15-B12)
    uint8_t group_type = (block_b >> 12) & 0x0F;
    
    // -------------------------------------------------------------------------
    // 1. Spracovanie Program Service (PS) - Group Type 0A
    // -------------------------------------------------------------------------
    if (group_type == 0x00 && !((block_b >> 11) & 0x01)) { // Kontrola Group Type 0A

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

            // --- POŽADOVANÁ ÚPRAVA ---
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

            // --- POŽADOVANÁ ÚPRAVA ---
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
            
            if (this->rds_text_sensor_ != nullptr && this->rds_text_sensor_->state != current_rt) {
                ESP_LOGD(TAG, "RDS: Publikujem RT: %s", current_rt.c_str());
                this->rds_text_sensor_->publish_state(current_rt);
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

}  // namespace si4703_fm
}  // namespace esphome
