#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/number/number.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/sensor/sensor.h" 

namespace esphome {
namespace si4703_fm {

// I2C adresa
static const uint8_t SI4703_I2C_ADDR = 0x10;

// Definície registrov
static const uint8_t POWERCFG = 0x02;
static const uint8_t CHANNEL = 0x03;
static const uint8_t SYSCONFIG1 = 0x04;
static const uint8_t SYSCONFIG2 = 0x05;
static const uint8_t SYSCONFIG3 = 0x06;
static const uint8_t TEST1 = 0x07;
static const uint8_t STATUSRSSI = 0x0A;
static const uint8_t READCHANNEL = 0x0B;

// Bity
static const uint16_t TUNE = (1 << 15);
static const uint16_t STC = (1 << 14); // Seek/Tune Complete

class Si4703FM; // Predbežná deklarácia

class Si4703PowerSwitch : public switch_::Switch, public Component {
public:
  Si4703PowerSwitch(Si4703FM *parent) : parent_(parent) {}
  void write_state(bool state) override;
  // Neimplementujeme setup ani loop, spoliehame sa na parenta
  
protected:
  Si4703FM *parent_;
};

// --- Trieda pre Number (Frequency) ---
class Si4703Frequency : public number::Number, public Component {
public:
  Si4703Frequency(Si4703FM *parent) : parent_(parent) {}
  void setup() override;
  float get_setup_priority() const override { return setup_priority::LATE; }
protected:
  void control(float value) override;
  Si4703FM *parent_;
};

// --- NOVÁ: Trieda pre Number (Volume) ---
class Si4703Volume : public number::Number, public Component {
public:
  Si4703Volume(Si4703FM *parent) : parent_(parent) {}
  void setup() override;
  float get_setup_priority() const override { return setup_priority::LATE; }
protected:
  void control(float value) override;
  Si4703FM *parent_;
};


// --- Hlavná trieda komponentu (Hub) ---
class Si4703FM : public Component, public i2c::I2CDevice {
public:
  Si4703FM(i2c::I2CBus *parent_bus, GPIOPin *reset_pin, GPIOPin *stc_int_pin);
  
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

  // Settery pre prepojenie entít
  void set_frequency_number(Si4703Frequency *freq_num) { this->frequency_number_ = freq_num; }
  void set_rssi_sensor(sensor::Sensor *rssi_sensor) { this->rssi_sensor_ = rssi_sensor; }
  void set_volume_number(Si4703Volume *vol_num) { this->volume_number_ = vol_num; }
  void set_power_switch(Si4703PowerSwitch *power_sw) { this->power_switch_ = power_sw; }
  
  // Interval aktualizácie
  void set_update_interval(uint32_t update_interval) { this->update_interval_ = update_interval; }

  // Verejné funkcie pre entity
  void set_channel_from_float(float freq_mhz);
  float get_current_frequency_mhz();
  float get_rssi();
  bool set_volume(uint8_t volume); // Nové
  uint8_t get_volume(); // Nové
  
  bool turn_on();
  bool turn_off();

  bool read_registers();

protected:
  GPIOPin *reset_pin_;
  GPIOPin *stc_int_pin_;

  uint16_t registers_[16] = {0};

  // Pointers na entity
  Si4703Frequency *frequency_number_ = nullptr;
  sensor::Sensor *rssi_sensor_{nullptr}; 
  Si4703Volume *volume_number_ = nullptr; // Nové
  Si4703PowerSwitch *power_switch_ = nullptr;
  
  uint32_t update_interval_ = 2000; // Predvolená hodnota
  uint32_t last_update_ = 0;

  // Pomocné funkcie
  bool si4703_init();
  
  bool update_all_registers();
  bool tune_to_channel(uint16_t channel_val);
  uint16_t get_channel_value();
  // wait_for_stc() už nepoužívame, spoliehame sa na pevný delay
};

}  // namespace si4703_fm
}  // namespace esphome
