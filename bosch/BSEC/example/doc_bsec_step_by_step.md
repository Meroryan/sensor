# BSEC Step-by-step Example {#exStepByStep}

Temperature, humidity, and the presence of certain gases all influence the quality of the air we are breathing. In this walk-through, we will see how to use Bosch Sensortec BME680 sensors together with the BSEC software package to measure indoor-air-quality (IAQ).

# Prerequisites {#exPrereq}

First of all, you will need a BME680 sensor that is connected to a microcontoller (MCU). The MCU will be used to control the operation of the sensor and to process the sensor signals to derive indoor-air-quality in the end. Of course, you will also need an development environment for the MCU of your choice. In this example, I will use the Arduino-based [Octopus](http://fab-lab.eu/octopus/) board with BME680 already included on-board. If you plan on following this tutorial with the same hardware, you can find instructions on how to setup Arduino IDE for this board [here](https://learn.adafruit.com/adafruit-feather-huzzah-esp8266/using-arduino-ide). If you have trouble downloading the board support package via the board manager, you can manually get the required files from [GitHub](https://github.com/esp8266/Arduino) directly.

Once we are set with our hardware and development environment, we need two pieces of software to get the most out of our BME680 sensor:

1. BME680 API ([available on GitHub](https://github.com/BoschSensortec/BME680_driver/releases/tag/v2.2.0)) deals with the low-level communication and basic compensation of sensor data. It saves us from having to fiddle with individual registers on the BME680 ourselves.
2. BSEC ([available from Bosch Sensortec](https://www.bosch-sensortec.com/bst/products/all_products/BSEC)) will be used by us to control the sensor operation and it will provide us with an indoor-air-quality output as well as compensated temperature and relative humidity data. For this it interfaces with the BME680 API we just downloaded. In case you are wondering, BSEC stands for "Bosch Software Environmental Cluster".

While the above might sound somewhat intimidating, we are lucky that BSEC comes with an ready made example code that only requires a small number of modifications to get it running on a new platform.

# Setting Everything Up {#exSetup}

To get started, we will first see which files from the packages we just downloaded need to be added to our project. From the BME680 API, we need to add all included .h and .c files. In case of BSEC, we need to add the BSEC interface headers as well the the example that we want to extend:

* `inc`
    * `bsec_interface.h`
    * `bsec_datatypes.h`
* `examples`
    * `bsec_integration.h`
    * `bsec_integration.c`
    * `bsec_iot_example.c`

As BSEC is made available as a pre-compiled binary, we should also get the correct .a file. Since the Octopus board uses an ESP8266 MCU, we need to use the library file found in `binaries/staticlib/ESP8266/libalgobsec.a` of the BSEC release package.

To use our code in an Arduino sketch, we should copy all the above mentioned files into a folder named `bsec_iot_example`.

# The Example Code {#exExampleCode}

Once we are set up, let's have a look at `bsec_iot_example.c` which is the only file we will have to modify to get our project up and running. You will see that it contains a `main()` function as shown below.

```C
int main()
{
    /* Call to the function which initializes the BSEC library 
     * Switch on low-power mode and provide no temperature offset */
    bsec_iot_init(BSEC_SAMPLE_RATE_LP, 0.0f, bus_write, bus_read, sleep);
    
    /* Call to endless loop function which reads and processes data based on sensor settings */
    bsec_iot_loop(sleep, get_timestamp_us, output_ready);
    
    return 0;
}
```

Here, we first initialize both the API and the BSEC library. For this purpose, we provide 3 function pointers `bus_write`, `bus_read`, and `sleep` to `bsec_iot_init()`. These pointers are used by BSEC and the BME680 API to communicate with the sensor and to put the system to sleep to control timings. Moreover, we provide the desired operation mode, in this case, we use low-power (LP) mode. The numerical argument allows us to subtract a temperature offset from the temperature reading and correct the humidity accordingly. More on this later.

Next, `bsec_iot_loop()` is called to enter an endless loop which periodically reads out sensor data, processes the signals, and calls the provided function pointer `outputs_ready` whenever new data is available. Additionally, we provide a function pointer `get_timestamp_us` that is used to get the system time stamps in microseconds.

Inside `bsec_iot_example.c`, we already find empty implementations of these five functions pointers. All we have to do to get our basic example up and running is to fill in some code into these functions and add a little bit of MCU initalization to the beginning of `main()`.

# Hello "Indoor-Air-Quality" {#exHelloIaq}

Since we want to use the example with Arduino IDE, we will have to convert the example C code into an Arduino-compatible sketch file. For this, we change the file name of `bsec_iot_example.c` to `bsec_iot_example.ino` and rename the `int main()` function into `void setup()`. At the top of the function, we add 2 lines to initalize the I2C port used to talk with our sensor as well as the serial line we want to use to report IAQ values back to our host PC. We also add an empty `loop()` to create a complete Arduino sketch.

```C
void setup()
{
    /* Init I2C and serial communication */
    Wire.begin();
    Serial.begin(115200);
  
    /* Call to the function which initializes the BSEC library 
     * Switch on low-power mode and provide no temperature offset */
    bsec_iot_init(BSEC_SAMPLE_RATE_LP, 0.0f, bus_write, bus_read, sleep);
    
    /* Call to endless loop function which reads and processes data based on sensor settings */
    bsec_iot_loop(sleep, get_timestamp_us, output_ready);
}

void loop()
{
    /* We do not need to put anything here as we enter our on loop function in setup() */
}
```

To be able to compile to above code, we also need to add the following include at the top of the file.

```C
#include <Wire.h>
```

With the setup done, we can now move to implementing the communication with the sensor. The write function takes an array of bytes as well as a register address to start writing to.

```C
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint8_t data_len)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);    /* Set register address to start writing to */

    /* Write the data */
    for (int index = 0; index < data_len; index++) {
        Wire.write(reg_data_ptr[index]);
    }

    return (int8_t)Wire.endTransmission();
}
```

In case of the read function, we burst read from the BME680 register map.

```C
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint8_t data_len)
{
    int8_t comResult = 0;
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);                    /* Set register address to start reading from */
    comResult = Wire.endTransmission();

    delayMicroseconds(150);                 /* Precautionary response delay */
    Wire.requestFrom(dev_addr, data_len);    /* Request data */

    int index = 0;
    while (Wire.available())  /* The slave device may send less than requested (burst read) */
    {
        reg_data_ptr[index] = Wire.read();
        index++;
    }

    return comResult;
}
```

To implement sleep functionality, we make use of the Arduino `delay()` function.

```C
void sleep(uint32_t t_ms)
{
    delay(t_ms);
}
```

Getting a timestamp is just as easy. Here, we use the `millis()` function to get a timestamp in milliseconds and multiply by 1000 to get the required microsecond resolution.

```C
int64_t get_timestamp_us()
{
    return (int64_t) millis() * 1000;
}
```

Finally, we need to do something with the measurement data we get. The simplest thing is to print out the something on the serial interface and look at it on your host machine. In this case, we print out the temperature, humidity, and IAQ values returned by BSEC. In the future, we could even return the pressure as it is also measured.

```C
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature,
    float pressure, float humidity, bsec_library_return_t bsec_status)
{
    Serial.print("[");
    Serial.print(timestamp/1e6);
    Serial.print("] T: ");
    Serial.print(temperature);
    Serial.print(" | rH: ");
    Serial.print(humidity);
    Serial.print(" | IAQ: ");
    Serial.print(iaq);
    Serial.print(" (");
    Serial.print(iaq_accuracy);
    Serial.println(")");
}
```

The last step we need to do is to ensure that the pre-build `libalgobsec.a` library is linked when we compile our project. Unfortunately, this process is somewhat tricky when it comes to Arduino IDE. We first need to find where the board support package for our board is installed. On Windows, this could be for example in `<USER_HOME>\AppData\Local\Arduino15\packages\esp8266\hardware` or in `<ARDUINO_ROOT>\hardware`. Once we found the location, we need to perform the following steps. Please keep in mind that the target paths might differ slightly depending on the ESP8266 package version you are using.

1. We need to copy the file `binaries\staticlib\ESP8266\libalgobsec.a` from the BSEC package into the `hardware\esp8266\2.3.0\tools\sdk\lib` folder.
2. The linker file found at `hardware\esp8266\2.3.0\tools\sdk\ld\eagle.app.v6.common.ld` needs to be modifed by inserting the line `*libalgobsec.a:(.literal .text .literal.* .text.*)` after the line `*libm.a:(.literal .text .literal.* .text.*)`.
3. Finally, we need to change the linker argument, telling the linker to include BSEC. This is achieved by adding the argument `-lalgobsec` to the line `compiler.c.elf.libs=-lm -lgcc ...` found in `hardware\esp8266\2.3.0\platform.txt`.

If we now run our project and check with the Serial Monitor (found under Tools menu in Arduino IDE), we can see a new measurement come in every 3 seconds as shown below.

```
...
[300345.00] T: 33.30| rH: 22.73| IAQ: 25.00 (3)
[303346.00] T: 33.30| rH: 22.76| IAQ: 26.30 (3)
[306346.00] T: 33.26| rH: 22.81| IAQ: 27.90 (3)
[309346.00] T: 33.26| rH: 22.84| IAQ: 19.72 (3)
[312346.00] T: 33.20| rH: 22.93| IAQ: 25.02 (3)
[315346.00] T: 33.19| rH: 22.94| IAQ: 20.70 (3)
[318346.00] T: 33.14| rH: 22.97| IAQ: 28.80 (3)
...
```

Success!

# Reducing power consumption {#exDataRate}

You will notice that we now get IAQ, temperature and humidity data once every 3 seconds. This is because the example code is pre-configured to use what is called low-power (LP) mode.

For certain applications, we may want to reduce the power consumption (and data rate) and use ultra-low-power mode. Since it only takes around 0.08 mA current on average, this mode is ideal for long-term battery powered operation. But let's see if we can change the code to lower the data rate.

In order to make this change, we can simply change the first argument we pass to `bsec_iot_init()` to ULP mode:

```C
/* Call to the function which initializes the BSEC library 
 * Switch on ultra-low-power mode and provide no temperature offset */
bsec_iot_init(BSEC_SAMPLE_RATE_ULP, 0.0f, bus_write, bus_read, sleep);
```

When we run our project now again, we can see the data coming in much slower and with greatly reduced current consumption.

```
...
[1200329.00] T: 30.97 | rH: 25.95 | IAQ: 25.00 (3)
[1500329.00] T: 31.35 | rH: 25.98 | IAQ: 97.96 (3)
[1800330.00] T: 30.86 | rH: 26.72 | IAQ: 131.54 (3)
[2100330.00] T: 30.80 | rH: 26.73 | IAQ: 124.95 (3)
...
```

# Temperature offsets due to heatsources on the board {#exSelfHeating}

Last but not least, let's have a look at the temperature and humidity values we are receiving from the board. A temperature of over thirty degrees and such a low relative humidity seems off. Looking at a reference thermometer, we can see that our temperature is indeed a few degrees to high. Does that mean the temperature sensor inside the BME680 is inaccurate?

Actually no, it very accurately measures the temperatue exactly where it is located on the board. But there also is the issue: our board as most devices contains some heatsources (e.g., MCU, WiFi chip, display, ...). This means the temperature on our board is actually higher than the ambient temperature. Since the absolute amount of water in the air is approximately constant, this also causes the relative humidity to be lower on our board than elsewhere in the room.

As BSEC cannot know in which kind of device the sensor is integrated, we have provide some information to the algorithm to enable it to compensate this offset. In the simplest case, we have to deal with an embedded device with a constant workload and approximately constant self-heating. In such a case, we can simply supply a temperature offset to BSEC which will be subtracted from the temperature and will be used to correct the humidity reading as well.

To achieve this, we simply provide a non-zero temperature offset as the second argument to `bsec_iot_init()` as shown below. Here, we subtract a 5 degrees offset, for example.

```C
/* Call to the function which initializes the BSEC library 
 * Switch on ultra-low-power mode and subtract a 5 degrees temperature offset */
bsec_iot_init(BSEC_SAMPLE_RATE_ULP, 5.0f, bus_write, bus_read, sleep);
```

# Conclusion {#exConclusion}

As you can see, it is easy to easily BSEC and BME680 API into an Arduino platform to measure indoor-air-quality, temperature, and humidity. We also did some modifications to the existing example code in order to change the sampling rate to ULP and to subtract a temperature offset.
