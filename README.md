# Alternative BNO055 Driver derived from Adafruit work #
PROOF OF CONCEPT

Expose an additional SetAccRange() method to modify the applicable range on the accelerometer with values BNO055_ACC_CONFIG_2G , BNO055_ACC_CONFIG_4G (the defaut), BNO055_ACC_CONFIG_8G, BNO055_ACC_CONFIG_16G  

This should present himself as "Adafruit BNO055 (alternate)" in the list of libraries.

Please note the following line in the setup() function... it may be necessary to switch to full ACCELEROMETER mode to avoids auto-ranging of parameters (AKA the FUSION mode, still to be confirmed).

```
void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* May be necessary to switch to Accelerometer mode to avoids FUSION auto-scalling */
  // bno.setMode( bno.OPERATION_MODE_ACCONLY );
 
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  if(!bno.setAccRange( bno.BNO055_ACC_CONFIG_8G ))
  {
     Serial.print("Ooops, changing Acc range seems to be refsued!");
  }
  delay(1000);

  ...
}
```

# Adafruit Unified BNO055 Driver (AHRS/Orientation)  #

This driver is for the Adafruit BNO055 Breakout (http://www.adafruit.com/products/2472),
and is based on Adafruit's Unified Sensor Library (Adafruit_Sensor).

To work with the Arduino Zero, the BNO055's ADR pin must be high, and an "#error" must be removed from the .h file.

## What is the Adafruit Unified Sensor Library? ##

The Adafruit Unified Sensor Library ([Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor)) provides a common interface and data type for any supported sensor.  It defines some basic information about the sensor (sensor limits, etc.), and returns standard SI units of a specific type and scale for each supported sensor type.

It provides a simple abstraction layer between your application and the actual sensor HW, allowing you to drop in any comparable sensor with only one or two lines of code to change in your project (essentially the constructor since the functions to read sensor data and get information about the sensor are defined in the base Adafruit_Sensor class).

This is imporant useful for two reasons:

1.) You can use the data right away because it's already converted to SI units that you understand and can compare, rather than meaningless values like 0..1023.

2.) Because SI units are standardised in the sensor library, you can also do quick sanity checks when working with new sensors, or drop in any comparable sensor if you need better sensitivity or if a lower cost unit becomes available, etc. 

Light sensors will always report units in lux, gyroscopes will always report units in rad/s, etc. ... freeing you up to focus on the data, rather than digging through the datasheet to understand what the sensor's raw numbers really mean.

## About this Driver ##

Adafruit invests time and resources providing this open source code.  Please support Adafruit and open-source hardware by purchasing products from Adafruit!

Written by Kevin (KTOWN) Townsend for Adafruit Industries.
