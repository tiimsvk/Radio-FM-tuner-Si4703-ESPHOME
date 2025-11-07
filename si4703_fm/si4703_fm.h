#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h" 

// Aby sme predišli viacnásobnej inklúzii
#ifndef ESPHOME_SI4703_FM_H
#define ESPHOME_SI4703_FM_H

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

// --- Trieda pre Number (Frequency) ---
// Musí dediť od Component, aby mala setup()
class Si4703Frequency : public number::Number, public Component {
public:
  Si4703Frequency(Si4703FM *parent) : parent_(parent) {}
  void setup() override; // Na nastavenie počiatočnej hodnoty
  float get_setup_priority() const override { return setup_priority::LATE; }
protected:
  void control(float value) override; // Volá sa pri zmene z HA
  Si4703FM *parent_;
};


// --- Hlavná trieda komponentu (Hub) ---
class Si4703FM : public Component, public i2c::I2CDevice {
public:
  Si4703FM(i2c::I2CBus *parent_bus, GPIOPin *reset_pin, GPIOPin *stc_int_pin);
  
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

  // Funkcia na prepojenie s Number entitou
  void set_frequency_number(Si4703Frequency *freq_num) { this->frequency_number_ = freq_num; }

  // NOVÉ: Funkcia na prepojenie so Sensor entitou (RSSI)
  void set_rssi_sensor(sensor::Sensor *rssi_sensor) { this->rssi_sensor_ = rssi_sensor; }

  // Verejné funkcie volané z Number entity
  void set_channel_from_float(float freq_mhz);
  float get_current_frequency_mhz();
  
  // NOVÉ: Funkcia na čítanie RSSI
  float get_rssi();

protected:
  GPIOPin *reset_pin_;
  GPIOPin *stc_int_pin_;

  uint16_t registers_[16] = {0};

  Si4703Frequency *frequency_number_ = nullptr;
  // NOVÉ: Pointer na RSSI senzor
  sensor::Sensor *rssi_sensor_{nullptr}; 
  
  uint32_t last_update_ = 0;
  static const uint32_t UPDATE_INTERVAL_MS = 2000;

  // Pomocné funkcie
  bool si4703_init();
  bool read_registers();
  bool update_all_registers(); // Zmenené z update_register
  bool tune_to_channel(uint16_t channel_val);
  uint16_t get_channel_value();
  void wait_for_stc();
};

}  // namespace si4703_fm
}  // namespace esphome

#endif // ESPHOME_SI4703_FM_H