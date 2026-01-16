#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/number/number.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/sensor/sensor.h" 
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include <string>

namespace esphome {
namespace si4703_fm {

// I2C adresa
static const uint8_t SI4703_I2C_ADDR = 0x10;

// RDS
static const uint8_t RDSA = 0x0C;
static const uint8_t RDSB = 0x0D;
static const uint8_t RDSC = 0x0E;
static const uint8_t RDSD = 0x0F;

static const uint8_t POWERCFG   = 0x02;
static const uint8_t CHANNEL    = 0x03;
static const uint8_t SYSCONFIG1 = 0x04;
static const uint8_t SYSCONFIG2 = 0x05;
static const uint8_t SYSCONFIG3 = 0x06;
static const uint8_t TEST1      = 0x07;
static const uint8_t STATUSRSSI = 0x0A;
static const uint8_t READCHANNEL= 0x0B;

// Bity, ktoré by mali byť v STATUSRSSI (0x0A)
static const uint16_t RDSR = (1 << 15);  // Bit 15 - RDS Ready
static const uint16_t RDSS = (1 << 11);  // Bit 11 - RDS Synchronized
static const uint16_t STC  = (1 << 14);  // Seek/Tune Complete
static const uint16_t SFBL = (1 << 13);  // Seek/Tune Fail/Band Limit

// Ostatné bity z iných registrov:
static const uint16_t RDY   = (1 << 15); // Ready bit v STATUSRSSI
static const uint16_t RDS   = (1 << 12); // Bit v SYSCONFIG1 (0x04) na zapnutie RDS
static const uint16_t DMUTE = (1 << 14); // Bit v POWERCFG (0x02) - Mute Disable

// Bity
static const uint16_t TUNE = (1 << 15);

// -----------------------------------------------------------------------------
// Forward deklarácia
// -----------------------------------------------------------------------------
class Si4703FM;

// -----------------------------------------------------------------------------
// Switch triedy
// -----------------------------------------------------------------------------

class Si4703PowerSwitch : public switch_::Switch {
public:
    Si4703PowerSwitch(Si4703FM *parent) : parent_(parent) {}
    void write_state(bool state) override; 
protected:
    Si4703FM *parent_;
};

class Si4703MuteSwitch : public switch_::Switch {
public:
    Si4703MuteSwitch(Si4703FM *parent) : parent_(parent) {}
    void write_state(bool state) override; 
protected:
    Si4703FM *parent_;
};

class Si4703AmpSwitch : public Component, public switch_::Switch {
 public:
  Si4703AmpSwitch(Si4703FM *parent) : parent_(parent) {}
 protected:
  void write_state(bool state) override;
  Si4703FM *parent_;
};

class Si4703Gpio2Switch : public Component, public switch_::Switch {
 public:
  Si4703Gpio2Switch(Si4703FM *parent) : parent_(parent) {}
 protected:
  void write_state(bool state) override;
  Si4703FM *parent_;
};

// -----------------------------------------------------------------------------
// Number triedy
// -----------------------------------------------------------------------------

class Si4703Frequency : public number::Number, public Component { 
public:
    Si4703Frequency(Si4703FM *parent) : parent_(parent) {}
    void setup() override; 
    void control(float value) override; 
protected:
    Si4703FM *parent_;
};

class Si4703Volume : public number::Number, public Component { 
public:
    Si4703Volume(Si4703FM *parent) : parent_(parent) {}
    void setup() override; 
    void control(float value) override; 
protected:
    Si4703FM *parent_;
};

// =============================================================================
// Binary Sensor triedy
// =============================================================================
class Si4703StereoIndicator : public binary_sensor::BinarySensor, public Component {
 public:
  Si4703StereoIndicator(Si4703FM *parent) : parent_(parent) {}
 protected:
  Si4703FM *parent_{nullptr};
};

// Trieda pre TP Indicator (Traffic Programme)
class Si4703TPIndicator : public binary_sensor::BinarySensor, public Component {
 public:
  Si4703TPIndicator(Si4703FM *parent) : parent_(parent) {}
 protected:
  Si4703FM *parent_{nullptr};
};

// Trieda pre TA Indicator (Traffic Announcement)
class Si4703TAIndicator : public binary_sensor::BinarySensor, public Component {
 public:
  Si4703TAIndicator(Si4703FM *parent) : parent_(parent) {}
 protected:
  Si4703FM *parent_{nullptr};
};

// NOVÉ: GPIO2 ako binárny vstup (hardware vypínač)
class Si4703Gpio2Sensor : public binary_sensor::BinarySensor, public Component {
 public:
  Si4703Gpio2Sensor(Si4703FM *parent) : parent_(parent) {}
 protected:
  Si4703FM *parent_;
};

// =============================================================================
// Button triedy
// =============================================================================
class Si4703SeekUpButton : public button::Button, public Component {
 public:
  Si4703SeekUpButton(Si4703FM *parent) : parent_(parent) {}
  void press_action() override;
 protected:
  Si4703FM *parent_{nullptr};
};

class Si4703SeekDownButton : public button::Button, public Component {
 public:
  Si4703SeekDownButton(Si4703FM *parent) : parent_(parent) {}
  void press_action() override;
 protected:
  Si4703FM *parent_{nullptr};
};

// -----------------------------------------------------------------------------
// Hlavná trieda (Hub)
// -----------------------------------------------------------------------------

class Si4703FM : public Component, public i2c::I2CDevice {
public:
  Si4703FM(i2c::I2CBus *parent_bus, GPIOPin *reset_pin, GPIOPin *stc_int_pin = nullptr, GPIOPin *gpio1_pin = nullptr, GPIOPin *gpio2_pin = nullptr);
  
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

  // Settery pre prepojenie entít
  void set_frequency_number(Si4703Frequency *freq_num) { this->frequency_number_ = freq_num; }
  void set_rssi_sensor(sensor::Sensor *rssi_sensor) { this->rssi_sensor_ = rssi_sensor; }
  void set_snr_sensor(sensor::Sensor *snr_sensor) { this->snr_sensor_ = snr_sensor; } //dev sensor
  //void set_bler_a_sensor(sensor::Sensor *bler_a_sensor) { this->bler_a_sensor_ = bler_a_sensor; }   //dev sensor 
  //void set_bler_d_sensor(sensor::Sensor *bler_d_sensor) { this->bler_d_sensor_ = bler_d_sensor; }   //dev sensor 
  void set_pty_sensor(sensor::Sensor *pty_sensor) { this->pty_sensor_ = pty_sensor; }
  void set_volume_number(Si4703Volume *vol_num) { this->volume_number_ = vol_num; }
  void set_power_switch(Si4703PowerSwitch *power_sw) { this->power_switch_ = power_sw; }
  void set_mute_switch(Si4703MuteSwitch *mute_sw) { this->mute_switch_ = mute_sw; } 
  void set_amp_switch(Si4703AmpSwitch *amp_sw) { this->amp_switch_ = amp_sw; }
  void set_gpio2_switch(Si4703Gpio2Switch *sw) { this->gpio2_switch_ = sw; }
  void set_rds_text_sensor(text_sensor::TextSensor *rds_text_sensor) { this->rds_text_sensor_ = rds_text_sensor; } 
  void set_rds_ps_sensor(text_sensor::TextSensor *rds_ps_sensor) { this->rds_ps_sensor_ = rds_ps_sensor; }
  void set_rds_ct_sensor(text_sensor::TextSensor *rds_ct_sensor) { this->rds_ct_sensor_ = rds_ct_sensor; }
  void set_stereo_indicator_sensor(binary_sensor::BinarySensor *s) { this->stereo_indicator_sensor_ = s; }
  void set_tp_indicator_sensor(binary_sensor::BinarySensor *s) { this->tp_indicator_sensor_ = s; }
  void set_ta_indicator_sensor(binary_sensor::BinarySensor *s) { this->ta_indicator_sensor_ = s; }
  void set_amp(bool on);   // GPIO1 Control (wrapper)
  void set_gpio2(bool on); // GPIO2 Control (wrapper)
  void set_seek_up_button(button::Button *btn) { this->seek_up_button_ = btn; }
  void set_seek_down_button(button::Button *btn) { this->seek_down_button_ = btn; }
  
  void publish_stereo_status();
  void publish_tp_ta_status();

  // Interval aktualizácie
  void set_update_interval(uint32_t update_interval) { this->update_interval_ = update_interval; }

  // Verejné funkcie pre entitu
  void set_channel_from_float(float freq_mhz);
  float get_current_frequency_mhz();
  float get_rssi();
  float get_snr(); // dev sensor
  //float get_bler_a(); // dev sensor
  //float get_bler_d(); // dev sensor
  float get_pty_code();
  bool set_volume(uint8_t volume); 
  uint8_t get_volume(); 
  
  bool turn_on();
  bool turn_off();
  bool mute();   
  bool unmute(bool restore_volume = true);
  void seek_up();
  void seek_down();
  
  bool set_mono_mode(bool mono_on); 
  bool is_mono_mode();             

  bool si4703_init();
  bool update_all_registers();
  bool read_registers();

  // GPIO1/2 cez I2C
  bool set_gpio1_state(bool on); // nastaví GPIO1 výstup v registroch (SYSCONFIG1 bits 1:0)
  bool set_gpio2_state(bool on); // nastaví GPIO2 výstup v registroch (SYSCONFIG1 bits 3:2)
  bool get_amp_pin_state() const { return this->amp_state_; }

protected:
  GPIOPin *reset_pin_;
  GPIOPin *stc_int_pin_;
  GPIOPin *gpio1_pin_{nullptr};
  GPIOPin *gpio2_pin_{nullptr};

  uint16_t registers_[16] = {0};
  int volume_ = 8;
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
  
  // --- NOVÉ: Premenné pre RDS Clock Time (CT) ---
  char rds_ct_buffer_[20]; // Buffer pre formátovaný čas, napr. "HH:MM"
  
    // --- NOVÉ: Premenné pre stabilizáciu PTY ---
  uint8_t last_pty_code_ = 255; // Posledný prijatý PTY kód pre overenie
  uint8_t pty_validation_count_ = 0; // Počet po sebe idúcich rovnakých PTY kódov
  static const uint8_t PTY_VALIDATION_THRESHOLD = 2; // Koľkokrát musí byť PTY rovnaký, než sa publikuje
  
  // Pointers na entity
  Si4703Frequency *frequency_number_ = nullptr;
  sensor::Sensor *rssi_sensor_{nullptr};
  sensor::Sensor *snr_sensor_{nullptr}; // SNR dev sensor
  //sensor::Sensor *bler_a_sensor_{nullptr}; // BLER A dev sensor
  //sensor::Sensor *bler_d_sensor_{nullptr}; // BLER D dev sensor
  sensor::Sensor *pty_sensor_{nullptr};
  Si4703Volume *volume_number_ = nullptr; 
  Si4703PowerSwitch *power_switch_ = nullptr;
  Si4703MuteSwitch *mute_switch_ = nullptr; 
  Si4703AmpSwitch *amp_switch_ = nullptr;
  Si4703Gpio2Switch *gpio2_switch_ = nullptr;
  binary_sensor::BinarySensor *stereo_indicator_sensor_{nullptr};
  binary_sensor::BinarySensor *tp_indicator_sensor_{nullptr};
  binary_sensor::BinarySensor *ta_indicator_sensor_{nullptr};
  binary_sensor::BinarySensor *gpio2_input_sensor_{nullptr};
  text_sensor::TextSensor *rds_text_sensor_{nullptr};
  text_sensor::TextSensor *rds_ps_sensor_{nullptr};
  text_sensor::TextSensor *rds_ct_sensor_{nullptr};
  button::Button *seek_up_button_{nullptr};
  button::Button *seek_down_button_{nullptr};
  
  uint32_t update_interval_ = 50; 
  uint32_t last_update_ = 0;
  uint32_t last_stereo_update_ = 0;
  static const uint32_t STEREO_UPDATE_INTERVAL = 300;

  bool gpio2_last_state_ = false;
  uint32_t gpio2_last_change_ = 0;
  static const uint32_t GPIO2_DEBOUNCE_MS = 50;

  bool amp_state_ = false;

  bool tune_to_channel(uint16_t channel_val);
  uint16_t get_channel_value();
  void process_rds_data(); 
  void process_rds();
  void decode_rds_data(uint16_t block_a, uint16_t block_b, uint16_t block_c, uint16_t block_d);
  
  void seek_internal(bool seek_up);
  void reset_rds_on_tune();
  
  void poll_gpio2_input();
};

}  // namespace si4703_fm
}  // namespace esphome
