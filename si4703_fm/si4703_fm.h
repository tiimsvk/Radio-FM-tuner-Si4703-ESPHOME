#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/number/number.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/sensor/sensor.h" 
#include "esphome/components/text_sensor/text_sensor.h"
#include <string> // Pridané pre std::string a RDS

namespace esphome {
namespace si4703_fm {

// I2C adresa
static const uint8_t SI4703_I2C_ADDR = 0x10;

// RDS
static const uint8_t RDSA = 0x0C;
static const uint8_t RDSB = 0x0D;
static const uint8_t RDSC = 0x0E;
static const uint8_t RDSD = 0x0F;

// Definície registrov
static const uint8_t POWERCFG = 0x02;
static const uint8_t CHANNEL = 0x03;
static const uint8_t SYSCONFIG1 = 0x04;
static const uint8_t SYSCONFIG2 = 0x05;
static const uint8_t SYSCONFIG3 = 0x06;
static const uint8_t TEST1 = 0x07;
static const uint8_t STATUSRSSI = 0x0A;
static const uint8_t READCHANNEL = 0x0B;

// Bity, ktoré by mali byť v STATUSRSSI (0x0A)
static const uint16_t RDSR = (1 << 15);  // Bit 15 - RDS Ready
static const uint16_t RDSS = (1 << 11);  // Bit 11 - RDS Synchronized (Kľúčová oprava) 

// Ostatné bity z iných registrov:
static const uint16_t RDY = (1 << 15); // Ready bit v STATUSRSSI
static const uint16_t RDS = (1 << 12);    // Bit v SYSCONFIG1 (0x04) na zapnutie RDS

// Bity
static const uint16_t TUNE = (1 << 15);
static const uint16_t STC = (1 << 14); // Seek/Tune Complete

class Si4703FM; // Predbežná deklarácia

// -----------------------------------------------------------------------------
// Switch triedy
// -----------------------------------------------------------------------------

class Si4703PowerSwitch : public switch_::Switch {
public:
    Si4703PowerSwitch(Si4703FM *parent) : parent_(parent) {}
    // Plná deklarácia write_state
    void write_state(bool state) override; 

protected:
    Si4703FM *parent_;
};

class Si4703MuteSwitch : public switch_::Switch {
public:
    Si4703MuteSwitch(Si4703FM *parent) : parent_(parent) {}
    // Plná deklarácia write_state
    void write_state(bool state) override; 

protected:
    Si4703FM *parent_;
};

class Si4703StereoMonoSwitch : public switch_::Switch {
public:
    Si4703StereoMonoSwitch(Si4703FM *parent) : parent_(parent) {}
    // Plná deklarácia write_state
    void write_state(bool state) override; 

protected:
    Si4703FM *parent_;
};

// -----------------------------------------------------------------------------
// Number triedy
// -----------------------------------------------------------------------------

class Si4703Frequency : public number::Number, public Component { 
public:
    Si4703Frequency(Si4703FM *parent) : parent_(parent) {}
    // Plná deklarácia setup a control
    void setup() override; 
    void control(float value) override; 
protected:
    Si4703FM *parent_;
};

class Si4703Volume : public number::Number, public Component { 
public:
    Si4703Volume(Si4703FM *parent) : parent_(parent) {}
    // Plná deklarácia setup a control
    void setup() override; 
    void control(float value) override; 
protected:
    Si4703FM *parent_;
};


// -----------------------------------------------------------------------------
// Hlavná trieda (Hub)
// -----------------------------------------------------------------------------

class Si4703FM : public Component, public i2c::I2CDevice {
public:
  Si4703FM(i2c::I2CBus *parent_bus, GPIOPin *reset_pin, GPIOPin *stc_int_pin);
  
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

  // Settery pre prepojenie entít
  void set_frequency_number(Si4703Frequency *freq_num) { this->frequency_number_ = freq_num; }
  void set_rssi_sensor(sensor::Sensor *rssi_sensor) { this->rssi_sensor_ = rssi_sensor; }
  void set_snr_sensor(sensor::Sensor *snr_sensor) { this->snr_sensor_ = snr_sensor; } // NOVÉ
  void set_bler_a_sensor(sensor::Sensor *bler_a_sensor) { this->bler_a_sensor_ = bler_a_sensor; } // NOVÉ
  void set_bler_d_sensor(sensor::Sensor *bler_d_sensor) { this->bler_d_sensor_ = bler_d_sensor; } // NOVÉ
  void set_volume_number(Si4703Volume *vol_num) { this->volume_number_ = vol_num; }
  void set_power_switch(Si4703PowerSwitch *power_sw) { this->power_switch_ = power_sw; }
  void set_mute_switch(Si4703MuteSwitch *mute_sw) { this->mute_switch_ = mute_sw; } 
  void set_stereo_mono_switch(Si4703StereoMonoSwitch *st_mo_sw) { this->stereo_mono_switch_ = st_mo_sw; } 
  void set_rds_text_sensor(text_sensor::TextSensor *rds_text_sensor) { this->rds_text_sensor_ = rds_text_sensor; } 
  void set_rds_ps_sensor(text_sensor::TextSensor *rds_ps_sensor) { this->rds_ps_sensor_ = rds_ps_sensor; }

  // Interval aktualizácie
  void set_update_interval(uint32_t update_interval) { this->update_interval_ = update_interval; }

  // Verejné funkcie pre entitu
  void set_channel_from_float(float freq_mhz);
  float get_current_frequency_mhz();
  float get_rssi();
  float get_snr(); // NOVÉ
  float get_bler_a(); // NOVÉ
  float get_bler_d(); // NOVÉ
  bool set_volume(uint8_t volume); 
  uint8_t get_volume(); 
  
  bool turn_on();
  bool turn_off();
  bool mute();   
  bool unmute(); 
  
  bool set_mono_mode(bool mono_on); 
  bool is_mono_mode();             

  bool si4703_init();
  bool update_all_registers();
  
  bool read_registers();

protected:
  GPIOPin *reset_pin_;
  GPIOPin *stc_int_pin_;

  uint16_t registers_[16] = {0};
  uint8_t previous_volume_ = 8;

  // --- Premenné pre RDS Program Service (PS) ---
  char rds_ps_buffer_[9] = {'\0'}; // PS má max 8 znakov + '\0'
  uint8_t rds_ps_last_ab_flag_ = 0;
  // --- Premenné pre RDS Radio Text (RT) ---
  char rds_rt_buffer_[65]; 
  uint8_t rds_rt_last_ab_flag_ = 0; 

  // Časovač pre RDS polling (100ms je dobrá hodnota)
  uint32_t last_rds_check_ = 0;
  static const uint32_t RDS_CHECK_INTERVAL = 50;
  
  // Pointers na entity
  Si4703Frequency *frequency_number_ = nullptr;
  sensor::Sensor *rssi_sensor_{nullptr};
  sensor::Sensor *snr_sensor_{nullptr}; // NOVÉ: SNR
  sensor::Sensor *bler_a_sensor_{nullptr}; // NOVÉ: BLER A
  sensor::Sensor *bler_d_sensor_{nullptr}; // NOVÉ: BLER D
  Si4703Volume *volume_number_ = nullptr; 
  Si4703PowerSwitch *power_switch_ = nullptr;
  Si4703MuteSwitch *mute_switch_ = nullptr; 
  Si4703StereoMonoSwitch *stereo_mono_switch_ = nullptr; 
  text_sensor::TextSensor *rds_text_sensor_{nullptr};
  text_sensor::TextSensor *rds_ps_sensor_{nullptr};
  
  uint32_t update_interval_ = 50; 
  uint32_t last_update_ = 0;

  bool tune_to_channel(uint16_t channel_val);
  uint16_t get_channel_value();
  void process_rds_data(); 
  
  void process_rds();
  void decode_rds_data(uint16_t block_a, uint16_t block_b, uint16_t block_c, uint16_t block_d);
  
};

}  // namespace si4703_fm
}  // namespace esphome
