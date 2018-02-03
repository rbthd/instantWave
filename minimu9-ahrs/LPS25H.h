#pragma once

#include <i2c_bus.h>
#include <cstdint>

namespace lps25h
{
  enum device_type
  {
    LPS25H = 0xBC,
  };

  enum i2c_addr
  {
    SA0_LOW_ADDR = 5D,
  };

  enum reg_addr
  {
    WHO_AM_I    = 0x0F,

    REF_P_XL    = 0x08, //Reference factory-calibrated pressure
    REF_P_XL    = 0x09, // Shouldn't be modified manually
    REF_P_H     = 0x0A, // except in CTRL_REG2

    RES_CONF    = 0x10,

    CTRL_REG1   = 0x20,
    CTRL_REG2   = 0x21,
    CTRL_REG3   = 0x22,
    CTRL_REG4   = 0x23,

    INT_CFG     = 0x24,
    INT_SOURCE  = 0x25,

    STATUS_REG  = 0x27,

    PRESS_POUT_XL = 0x28,
    PRESS_OUT_L   = 0x29,
    PRESS_OUT_H   = 0x2A,

    TEMP_OUT_L    = 0x2B,
    TEMP_OUT_H    = 0x2C,

    FIFO_CTRL     = 0x2E,
    FIFO_STATUS   = 0x2F,

    THS_P_L       = 0x30,
    THS_P_H       = 0x31,

    RPDS_L        = 0x39,
    RPDS_H        = 0x3A,
  };

  struct comm_config
  {
    bool use_sensor = false;
    device_type device;
    std::string i2c_bus_name;
    i2c_addr i2c_address;
  };

  class handle
  {
  public:
    void open(const comm_config &);
    void enable();
    void write_reg(reg_addr, uint8_t value);
    void read();

    int32_t p;

  protected:
    i2c_bus i2c;
    comm_config config;
  };

};
