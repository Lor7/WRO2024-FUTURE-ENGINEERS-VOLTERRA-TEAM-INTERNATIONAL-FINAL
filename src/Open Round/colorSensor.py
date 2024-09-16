from smbus2 import SMBus

# Initialize I2C bus on bus 1 (default for Raspberry Pi)
bus = SMBus(1)

class ColorSensor():

    # I2C address for the TCS34725 color sensor
    TCS34725_DEFAULT_ADDRESS = 0x29

    # TCS34725 register set
    TCS34725_COMMAND_BIT = 0x80  # Command bit to indicate a command for the sensor
    TCS34725_REG_ENABLE = 0x00   # Enable states and interrupts
    TCS34725_REG_ATIME = 0x01    # Integration time for color measurement
    TCS34725_REG_WTIME = 0x03    # Wait time register
    TCS34725_REG_CONFIG = 0x0D   # Configuration register
    TCS34725_REG_CONTROL = 0x0F  # Control register for setting gain
    TCS34725_REG_CDATAL = 0x14   # Low byte of the clear (IR) channel data
    TCS34725_REG_CDATAH = 0x15   # High byte of the clear (IR) channel data
    TCS34725_REG_RDATAL = 0x16   # Low byte of red channel data
    TCS34725_REG_RDATAH = 0x17   # High byte of red channel data
    TCS34725_REG_GDATAL = 0x18   # Low byte of green channel data
    TCS34725_REG_GDATAH = 0x19   # High byte of green channel data
    TCS34725_REG_BDATAL = 0x1A   # Low byte of blue channel data
    TCS34725_REG_BDATAH = 0x1B   # High byte of blue channel data

    # TCS34725 ENABLE register configuration flags
    TCS34725_REG_ENABLE_SAI = 0x40  # Sleep after interrupt
    TCS34725_REG_ENABLE_AIEN = 0x10 # ALS (ambient light sensing) interrupt enable
    TCS34725_REG_ENABLE_WEN = 0x08  # Wait enable
    TCS34725_REG_ENABLE_AEN = 0x02  # ADC enable (activates RGBC function)
    TCS34725_REG_ENABLE_PON = 0x01  # Power on

    # ATIME register configuration for integration time
    TCS34725_REG_ATIME_2_4 = 0xFF  # 2.4 ms integration time, 1 cycle
    TCS34725_REG_ATIME_24 = 0xF6   # 24 ms integration time, 10 cycles
    TCS34725_REG_ATIME_101 = 0xDB  # 101 ms integration time, 42 cycles
    TCS34725_REG_ATIME_154 = 0xC0  # 154 ms integration time, 64 cycles
    TCS34725_REG_ATIME_700 = 0x00  # 700 ms integration time, 256 cycles
    TCS34725_REG_WTIME_2_4 = 0xFF  # 2.4 ms wait time
    TCS34725_REG_WTIME_204 = 0xAB  # 204 ms wait time
    TCS34725_REG_WTIME_614 = 0x00  # 614 ms wait time

    # Gain configuration for the CONTROL register
    TCS34725_REG_CONTROL_AGAIN_1 = 0x00  # 1x gain
    TCS34725_REG_CONTROL_AGAIN_4 = 0x01  # 4x gain
    TCS34725_REG_CONTROL_AGAIN_16 = 0x02 # 16x gain
    TCS34725_REG_CONTROL_AGAIN_60 = 0x03 # 60x gain

    def __init__(self):
        # Initialize the sensor by configuring the ENABLE, TIME, and GAIN registers
        self.enable_selection()  # Set power on and enable ADC
        self.time_selection()    # Set integration and wait time
        self.gain_selection()    # Set gain level for the sensor
        self._stop = False       # Flag for stopping continuous data read
        self.colorDict = None    # Dictionary to store color data (red, green, blue)

    def enable_selection(self):
        """Configure the ENABLE register to power on and enable ADC for the color sensor."""
        ENABLE_CONFIGURATION = (self.TCS34725_REG_ENABLE_AEN | self.TCS34725_REG_ENABLE_PON)
        # Write the configuration to the ENABLE register
        bus.write_byte_data(self.TCS34725_DEFAULT_ADDRESS, self.TCS34725_REG_ENABLE | self.TCS34725_COMMAND_BIT, ENABLE_CONFIGURATION)
    
    def time_selection(self):
        """Configure the ATIME and WTIME registers for integration and wait times."""
        # Set ATIME for integration time (2.4 ms in this case)
        bus.write_byte_data(self.TCS34725_DEFAULT_ADDRESS, self.TCS34725_REG_ATIME | self.TCS34725_COMMAND_BIT, self.TCS34725_REG_WTIME_2_4)
        # Set WTIME for wait time (2.4 ms in this case)
        bus.write_byte_data(self.TCS34725_DEFAULT_ADDRESS, self.TCS34725_REG_WTIME | self.TCS34725_COMMAND_BIT, self.TCS34725_REG_WTIME_2_4)
        
    def gain_selection(self):
        """Configure the gain control register."""
        # Set the gain to 1x
        bus.write_byte_data(self.TCS34725_DEFAULT_ADDRESS, self.TCS34725_REG_CONTROL | self.TCS34725_COMMAND_BIT, self.TCS34725_REG_CONTROL_AGAIN_1)
    
    def readData(self):
        """Read color data from the sensor and return normalized RGB values and luminance."""
        # Read 8 bytes of data starting from the clear data register
        data = bus.read_i2c_block_data(self.TCS34725_DEFAULT_ADDRESS, self.TCS34725_REG_CDATAL | self.TCS34725_COMMAND_BIT, 8)
        
        # Convert raw data into 16-bit values for clear, red, green, and blue channels
        cData = data[1] * 256 + data[0]  # Clear/IR data
        red = data[3] * 256 + data[2]    # Red data
        green = data[5] * 256 + data[4]  # Green data
        blue = data[7] * 256 + data[6]   # Blue data
        
        # Calculate luminance using a weighted sum of red, green, and blue channels
        luminance = (-0.32466 * red) + (1.57837 * green) + (-0.73191 * blue)
        
        # If clear channel data is zero, return black
        if cData == 0:
            return {'black': 0}
        
        # Normalize RGB values based on the clear channel data
        red = red / cData * 255
        green = green / cData * 255
        blue = blue / cData * 255

        # Return a dictionary with color and luminance data
        return {'sum' : cData, 'red' : red, 'green' : green, 'blue' : blue, 'luminance' : luminance}
    
    def readDataContinuously(self):
        """Continuously read color data and update the color dictionary."""
        while not(self._stop):
            try:
                # Continuously read data until stopped
                data = bus.read_i2c_block_data(self.TCS34725_DEFAULT_ADDRESS, self.TCS34725_REG_CDATAL | self.TCS34725_COMMAND_BIT, 8)
                
                # Convert the data into clear, red, green, and blue values
                cData = data[1] * 256 + data[0]
                red = data[3] * 256 + data[2]
                green = data[5] * 256 + data[4]
                blue = data[7] * 256 + data[6]
                
                # Skip iteration if clear data is zero
                if cData == 0:
                    continue
                
                # Normalize RGB values based on the clear channel
                red = red / cData * 255
                green = green / cData * 255
                blue = blue / cData * 255

                # Update the color dictionary with the RGB values
                self.colorDict = {'red' : red, 'green' : green, 'blue' : blue}
            except Exception as e:
                # Print any exception that occurs during continuous reading
                print(str(e))

    @property
    def stop(self):
        """Getter for the stop flag."""
        return self._stop
    
    @stop.setter
    def stop(self, value):
        """Setter for the stop flag, allowing continuous reading to be stopped."""
        self._stop = value

    @property
    def datas(self):
        """Getter for the latest color data dictionary."""
        return self.colorDict
