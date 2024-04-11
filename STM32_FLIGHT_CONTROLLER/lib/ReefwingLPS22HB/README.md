![version](https://img.shields.io/github/v/tag/Reefwing-Software/Reefwing-LPS22HB) ![license](https://img.shields.io/badge/license-MIT-green) ![release](https://img.shields.io/github/release-date/Reefwing-Software/Reefwing-LPS22HB?color="red") ![open source](https://badgen.net/badge/open/source/blue?icon=github)

# Reefwing LPS22HB
 
 This is a Library for the LPS22HB Pressure Sensor, found in the Arduino Nano 33 BLE Sense Revisions 1 and 2. This library differs from the ArduinoLPS22HB library in the following ways:
 
 - It provides altitude calculations for QNE, QNH and QFE pressure references. 
 - It enables Block Data Update (`BDU`) which ensures that the content of the output registers is not updated until the last register is read, avoiding the reading of values related to different samples. This is important if you set the sampling rate (ODR) to anything other than one-shot.
 - It includes 2's complement conversion for the temperature reading allowing negative values to be returned.

 For additional details, please refer to our Medium article: [Reefwing LPS22HB Library for the Nano 33 BLE Sense](https://reefwing.medium.com/reefwing-lps22hb-library-for-the-nano-33-ble-sense-44839caa34e4).
 
 The LPS22HB is a compact piezoresistive absolute pressure sensor which functions as a digital barometer. The device comprises a sensing element and an interface which communicates using I2C or SPI. The Nano 33 BLE Sense is connected via I2C on Wire 1, and is factory calibrated.

 The LPS22HB has a 260 to 1260 hPa absolute pressure range, and can be sampled at between 1 and 75 Hz (Output Data Rate = ODR = 1, 10, 25, 50 or 75). Within the range 800 - 1100 hPa, the relative pressure accuracy is ±0.1 hPa.

 You can use this library in one-shot mode, which is the default, or set the acquisition data rate frequency using the `setODR(Rate: rate)` function.

 ```
 Note that hectopascals (hPa) and millibars (mbar) are equivalent.

 1 hPa = 100 Pascals = 1 mbar = 0.1 kPa

Following the adoption of the Pascal as the SI unit of pressure, 
meteorologists chose the hectopascal as the international unit for 
measuring atmospheric pressure. 

The millibar is still often used in weather reports and forecasts.
 ```

 The temperature sensor operates between -40 to 85°C and has an absolute accuracy of ±1.5°C. ODR for the temperature sensor is the same as for the pressure sensor.

 This library was originally contained within the [Reefwing AHRS](https://github.com/Reefwing-Software/Reefwing-AHRS) library but was broken out due to the release of Rev. 2 of the Nano 33 BLE Sense. This new version of the Sense board had updated sensors which were different to Rev. 1.

 ## Register Mapping

 The device contains a set of 8-bit registers which are used to control its behavior and to retrieve pressure and temperature data. The register address, made up of 7 bits, is used to identify them and to read/write the data through the serial interface.

 The registers that we use are shown in the following table, which are an extract of Table 16 in the data sheet.

| Name         | Type | Address | Function            |
|--------------|------|---------|---------------------|
|   WHO_AM_I   |   R  |   0x0F  |       Chip ID       |
|   CTRL_REG1  |  R/W |   0x10  |   Control Register  |
|   CTRL_REG2  |  R/W |   0x11  |   Control Register  |
|   CTRL_REG3  |  R/W |   0x12  |   Control Register  |
|    STATUS    |   R  |   0x27  |   Status Register   |
| PRESS_OUT_XL |   R  |   0x28  |   Pressure Reading  |
|  PRESS_OUT_L |   R  |   0x29  |   Pressure Reading  |
|  PRESS_OUT_H |   R  |   0x2A  |   Pressure Reading  |
|  TEMP_OUT_L  |   R  |   0x2B  | Temperature Reading |
|  TEMP_OUT_H  |   R  |   0x2C  | Temperature Reading |

## Who Am I?

Most chips include a register which we can read to positively identify the device on the I2C bus. For the LPS22HB, this is the `WHO_AM_I` register at address `0x0F`. The value contained in this register is `0xB1` = `1011 0001`.

| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| 1 | 0 | 1 | 1 | 0 | 0 | 0 | 1 |

We have a function which reads the `WHO_AM_I` register and `connected()` which confirms that the value in the register = `LPS22HB_WHO_AM_I_VALUE` = `0xB1`.

```c++
byte ReefwingLPS22HB::whoAmI() {
  Wire1.beginTransmission(_address);
  Wire1.write(LPS22HB_WHO_AM_I);
  Wire1.endTransmission();
  Wire1.requestFrom(_address, 1);
  return Wire1.read();
}

bool ReefwingLPS22HB::connected() {
  return (whoAmI() == LPS22HB_WHO_AM_I_VALUE);
}
```

## One-Shot Sampling & Control Register 2 (0x11)

The 8-bit control register 2 (`CTRL_REG2`), is used to set the data acquisition mode of the barometer. 

|   7  |    6    |      5      |      4     |    3    |    2    | 1 |     0    |
|:----:|:-------:|:-----------:|:----------:|:-------:|:-------:|:-:|:--------:|
| BOOT | FIFO_EN | STOP_ON_FTH | IF_ADD_INC | I2C_DIS | SWRESET | 0 | ONE_SHOT |

To request a new dataset, all bits are set to 0 apart from bit 0 (`ONE_SHOT`), which is set to 1.

```c++
    write(LPS22HB_CTRL_REG2, 0x01);
```

If the `ONE_SHOT` bit in `CTRL_REG2` (0x11) is set to '1', one-shot mode is triggered and a
new acquisition starts. Enabling this mode is possible only if the device was previously in power-down mode (ODR bits in `CTRL_REG1` set to '000'). Once the acquisition is completed and the output registers updated, the device automatically enters power-down mode and the `ONE_SHOT` bit clears itself. 

## Continuous Sampling & Control Register 1 (0x10)

| 7 |   6  |   5  |   4  |    3    |     2    |  1  |  0  |
|:-:|:----:|:----:|:----:|:-------:|:--------:|:---:|:---:|
| 0 | ODR2 | ODR1 | ODR0 | EN_LPFP | LPFP_CFG | BDU | SIM |

There are four important bits in control register 1 (`CTRL_REG1`), which are used for output data rate selection and block data update, these are:

- `ODR2`: Bit 6
- `ODR1`: Bit 5
- `ODR0`: Bit 4
- `BDU` : Bit 1

The value of these bits are used to determine the sensor acquisition rate as shown in the following table.

| ODR2 | ODR1 | ODR0 | Sample Rate |
|------|------|------|-------------|
|   0  |   0  |   0  |   One-Shot  |
|   0  |   0  |   1  |     1 Hz    |
|   0  |   1  |   0  |    10 Hz    |
|   0  |   1  |   1  |    25 Hz    |
|   1  |   0  |   0  |    50 Hz    |
|   1  |   0  |   1  |    75 Hz    |

When the ODR bits are set to '000', the device is in Power-down / One-Shot mode (See previous section). When the device is in power-down mode, almost all the internal functional blocks of the device are switched off to minimize power consumption. The I2C interface is still active to allow communication.

When the ODR bits are set to a value different than '000', the device is in Continuous
mode and automatically acquires a set of data (pressure and temperature) at the frequency
selected through the `ODR[2:0]` bits. 

The `BDU` bit in `CTRL_REG1` is used to inhibit the update of the output registers between the reading of upper, middle and lower register parts. In default mode (`BDU` = ‘0’), the lower and upper register parts are updated continuously. When the `BDU` is activated (`BDU` = ‘1’), the content of the output registers is not updated until `PRESS_OUT_H` (`0x2A`) is read, avoiding the reading of values related to different samples. This is why we read the `PRESS_OUT_H` register last.

The LPS22HB has 32 slots of 40-bit FIFO data to store the pressure and temperature output values. We are not using this and have the FIFO mode set to Bypass.

## Interpreting Pressure Readings

The pressure data is stored in 3 registers: `PRESS_OUT_H` (0x2A), `PRESS_OUT_L` (0x29),
and `PRESS_OUT_XL` (0x28). The value is expressed as a 2’s complement. To obtain the pressure in hPa, take the two’s complement of the complete word and then divide by 4096 LSB/hPa.

```c++
    uint8_t pressOutXL = read(LPS22HB_PRES_OUT_XL);
    uint8_t pressOutL = read(LPS22HB_PRES_OUT_L);
    uint8_t pressOutH = read(LPS22HB_PRES_OUT_H);

    long val = ( ((long)pressOutH << 16) | ((long)pressOutL << 8) | (long)pressOutXL );
  
    return val/4096.0f;
```

## The STATUS Register (0x27)

| 7 | 6 |   5  |   4  | 3 | 2 |   1  |   0  |
|:-:|:-:|:----:|:----:|:-:|:-:|:----:|:----:|
| - | - | T_OR | P_OR | - | - | T_DA | P_DA |

The LPS22HB 8-bit `STATUS` Register, located at address `0x27` contains two bits that we are interested in, `P_DA` (BIT 0) and `T_DA` (BIT 1). This register is updated every ODR cycle.

- `P_DA`: Is the Pressure Data Available bit, it is 1 when new pressure data is available and 0 otherwise.
- `T_DA`: Is the Temperature Data Available bit, it is 1 when new temperature data is available and 0 otherwise.

For one-shot readings we can just check that the `ONE_SHOT` bit in `CTRL_REG2` (0x11) has cleared.

## Determing Altitude from Air Pressure

You can't obtain an absolute height using air pressure. What you can do is get a relative height based on two different air pressures. At low altitudes, the pressure decreases by about 1.2 kPa (12 hPa) for every 100 metres gained. This decrease in air pressure with increased altitude is due to the reducing air density and weight of air above. The relationship is not linear though and weather conditions, including temperature and humidity, also affect the atmospheric pressure. Pressure is proportional to temperature and inversely proportional to humidity.

The [barometric formula](https://en.wikipedia.org/wiki/Barometric_formula) is used to model how air pressure changes with altitude. The barometric formula is derived from the [ideal gas law](https://en.wikipedia.org/wiki/Ideal_gas_law). If we assume that the temperature lapse rate is zero (i.e., temperature doesn't change with altitude, which is a simplification), then the barometric formula for altitudes up to 86 kms is:

```
P = Pr exp [-gM(h - hr)  / (R * Tr) ]

Where:

- Pr = reference pressure
- Tr = reference temperature (K)
- h  = height at which pressure (P) is calculated (m)
- hr = height of reference level b (m)
- R  = universal gas constant: 8.3144598 J/(mol·K)
- g  = gravitational acceleration: 9.80665 m/s^2
- M  = molar mass of Earth's air: 0.0289644 kg/mol

If we include the standard temperature lapse rate (Lr = -0.0065 [K/m]), 
the barometeric formula becomes:

P = Pr[(Tr + (h - hr)Lr) / Tr] exp [-gM / (R * Lr)]

Which if we rearrange to get height is:

h = hr + Tr/Lr {((P/Pr) ^ [-(R * Lr) / gM]) - 1}
```

A common simplification in sensor libraries (e.g., the official [Arduino LPS22HB Library](https://github.com/arduino-libraries/Arduino_LPS22HB/blob/master/src/BARO.cpp)) is to use the formula:

```
h = 44330 * [1 - (P/Pr) ^ (1/5.255)]
```

This formula makes a number of assumptions (Tr = 15°C, Lr = -0.0065 [K/m]) about the parameters in the equation above. Whether these assumptions are appropriate will depend on your application. Also, based on our re-arrangement of the original equation, the correct simplification should be:

```
h = 44330 * [(P/Pr) ^ (1/5.255) - 1]
```

Nevertheless, we will include Tr and Pr in our calculations, and use our form of the derived equation. The full barometric formula for height:

```
h = hr + Tr/Lr [(P/Pr) exp [-(R * Lr) / gM] - 1]
```

can be simplified by pre-calculating the constant expressions in the formula, in particular, 

- hr = 0 as our height will be relative to hr.
- The exponent can be simplified to: -(R * Lr) / gM = 0.190266435663732

Then the formula becomes:

```
h = (Tr/0.0065) * [(P/Pr) ^ (0.190266) - 1]
```

## QNH, QFE, and QNE

Pilots have been concerned about measuring altitude for some time, and it makes sense for us to use the commonly accepted pressure references (`Pr` in the formulas above). 

The reference often used in barometer libararies is `QNE = 1013.25 hPa = 1 atm`, the standard pressure reference, standard atmosphere, or average sea-level pressure. This reference has the advantage of being a constant, but you need to remember that the altitude derived is the vertical distance between your sensor and the height where atmospheric pressure is 1013.25 hPa. In aviation this distance is referred to as a Flight Level (FL), and these are used to maintain aircraft separation above the Transition Level (which varies by location as indicated on aviation charts).

Below the Transition Level, aircraft use QNH as the reference pressure. `QNH is the Pressure at Mean Sea Level (MSL)` and it varies by location. The sensor will read the altitude above mean sea level in the vicinity of the airfield from which the QNH is obtained. These standards are defined so that everyone is using the same reference pressure and thus comparable altitudes, thereby avoiding mid air collisions.

The third standard reference pressure is QFE. `QFE is the pressure of a location on the ground`, normally a runway threshold. If your sensor library sets Pr = QFE, then the altitude will be zero at that location. As your sensor goes up, it will give a height Above Ground Level (AGL). Obviously terrain is not flat, so the height AGL will only be with reference to the defined location.

Our library allows you to use any of these three pressure reference standards.

## Using the Library

Looking at the examples is the easiest way to learn how to use this Library. The first thing you need to do is include the library and instantiate the LPS22HB class in your sketch.

```c++
#include <ReefwingLPS22HB.h>

ReefwingLPS22HB LPS22HB;
```

In `setup()` you need to call `LPS22HB.begin()` to hook up the I2C connection and initialise the sensor. You can then use `LPS22HB.connected()` to check whether the sensor is available for pressure and temperature readings.

```c++
LPS22HB.begin();
while (!Serial);

if (LPS22HB.connected()) {
    Serial.println("LPS22HB Pressure Sensor found.");
}
```

In `loop()` you can obtain data, process it as required and print it out to the serial console.

```c++
void loop() {
    // Sensor Reading Loop
    Serial.print("Pressure: "); Serial.print(LPS22HB.readPressure()); Serial.print(" hPa, ");
    Serial.print("Temperature: "); Serial.print(LPS22HB.readTemperature()); Serial.print(" C, ");
    Serial.print("Altitude: "); Serial.print(LPS22HB.readAltitude()); Serial.println(" m");
    delay(1000);
}
```

## Library Public Methods

The ReefwingLPS22HB Library contains the following public methods to control your sensor.

```c++
  ReefwingLPS22HB();

  void begin();
  void reset();
  bool connected();

  uint8_t whoAmI();
  void setODR(Rate rate);
  void setQNH(float q);
  float getQNH();
  void clearQNH();
  float readTemperature(Scales scales = Scales::CELSIUS);
  float readPressure(Units units = Units::HECTOPASCAL);
  float readAltitude(PressureReference Pr = PressureReference::QNE);
  uint32_t readPressureRAW();
```



