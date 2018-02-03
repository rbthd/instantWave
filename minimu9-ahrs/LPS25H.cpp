#include "LPS25H.h"
#include <stdexcept>

void lps25h::handle::open(const comm_config & config)
{
  if (!config.use_sensor)
  {
    throw std::runtime_error("LPS25H configuration is null.");
  }
  this->config = config;
  i2c.open(config.i2c_bus_name);
}

void lps25h::handle::write_reg(reg_addr addr, uint8_t value)
{
  i2c.write_two_bytes(config.i2c_address, addr, value);
}

void lps25h::handle::enable()
{
  if(config.device == LPS25H)
  {
    //Activation of sensor
    // ODR = 000 (one shot)
    write_reg(CTRL_REG1, 0b10010000);

    //FIFO_EN = 0 --> if enabled, allows ...
    //
    //write_reg(CTRL_REG2, 0b00000000)
  }
}

void lps25h::read()
{
uint8_t ppxl;
uint8_t pol;
uint8_t poh;
i2c.write_byte_and_read(config.i2c_address, PRESS_POUT_XL, ppxl, sizeof(ppxl));
i2c.write_byte_and_read(config.i2c_address, PRESS_POUT_L, pol, sizeof(pol));
i2c.write_byte_and_read(config.i2c_address, PRESS_POUT_H, poh, sizeof(poh));

uint32_t p_tmp = poh << 16 | pol << 8  | ppxl;
float p = p_tmp/4096.0;
}
