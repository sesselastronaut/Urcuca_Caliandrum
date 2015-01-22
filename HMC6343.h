/*------------------------------------------------
Arduino Header HMC6343
from http://www.sparkfun.com/commerce/product_info.php?products_id=8656
------------------------------------------------
*/
#ifndef HMC6343_h
#define HMC6343_h
/**
* analog input 5 - I2C SCL
* analog input 4 - I2C SDA
**/
class ReadAngleHMC6343
{
  private:
  // I2C Client Address of the sensor
  const static int i2cAddress= (0x32 >> 1);
  ReadAngleHMC6343(){};
  public:
  static int getAngle();
};
#endif 
