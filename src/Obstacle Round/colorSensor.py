from smbus2 import SMBus
from time import sleep
from threading import Thread

# Initialize the I2C bus on bus number 1
bus = SMBus(1)

class ColorSensor():
    """
    Class to interface with the TCS34725 color sensor via I2C.
    """

    # I2C Address of the TCS34725 color sensor
    TCS34725_DEFAULT_ADDRESS = 0x29

    # TCS34725 Register Addresses
    TCS34725_COMMAND_BIT = 0x80
    TCS34725_REG_ENABLE = 0x00 # Enables states and interrupts
    TCS34725_REG_ATIME = 0x01 # RGBC integration time
    TCS34725_REG_WTIME = 0x03 # Wait time
    TCS34725_REG_CONFIG = 0x0D # Configuration register
    TCS34725_REG_CONTROL = 0x0F # Control register
    TCS34725_REG_CDATAL = 0x14 # Clear/IR channel low data register
    TCS34725_REG_CDATAH = 0x15 # Clear/IR channel high data register
    TCS34725_REG_RDATAL = 0x16 # Red ADC low data register
    TCS34725_REG_RDATAH = 0x17 # Red ADC high data register
    TCS34725_REG_GDATAL = 0x18 # Green ADC low data register
    TCS34725_REG_GDATAH = 0x19 # Green ADC high data register
    TCS34725_REG_BDATAL = 0x1A # Blue ADC low data register
    TCS34725_REG_BDATAH = 0x1B # Blue ADC high data register

    # TCS34725 Enable Register Configuration
    TCS34725_REG_ENABLE_SAI = 0x40 # Sleep After Interrupt
    TCS34725_REG_ENABLE_AIEN = 0x10 # ALS Interrupt Enable
    TCS34725_REG_ENABLE_WEN = 0x08 # Wait Enable
    TCS34725_REG_ENABLE_AEN = 0x02 # ADC Enable
    TCS34725_REG_ENABLE_PON = 0x01 # Power ON

    # TCS34725 Time Register Configuration
    TCS34725_REG_ATIME_2_4 = 0xFF # Atime = 2.4 ms, Cycles = 1
    TCS34725_REG_ATIME_24 = 0xF6 # Atime = 24 ms, Cycles = 10
    TCS34725_REG_ATIME_101 = 0xDB # Atime = 101 ms, Cycles = 42
    TCS34725_REG_ATIME_154 = 0xC0 # Atime = 154 ms, Cycles = 64
    TCS34725_REG_ATIME_700 = 0x00 # Atime = 700 ms, Cycles = 256
    TCS34725_REG_WTIME_2_4 = 0xFF # Wtime = 2.4 ms
    TCS34725_REG_WTIME_204 = 0xAB # Wtime = 204 ms
    TCS34725_REG_WTIME_614 = 0x00 # Wtime = 614 ms

    # TCS34725 Gain Configuration
    TCS34725_REG_CONTROL_AGAIN_1 = 0x00 # 1x Gain
    TCS34725_REG_CONTROL_AGAIN_4 = 0x01 # 4x Gain
    TCS34725_REG_CONTROL_AGAIN_16 = 0x02 # 16x Gain
    TCS34725_REG_CONTROL_AGAIN_60 = 0x03 # 60x Gain

    def __init__(self):
        """
        Initialize the color sensor with default settings.
        """
        self.enable_selection()  # Set up the ENABLE register
        self.time_selection()    # Set up the ATIME and WTIME registers
        self.gain_selection()    # Set up the CONTROL register for gain
        self._stop = False       # Flag to control the continuous reading thread
        self.colorDict = None    # Dictionary to hold the color values

    def enable_selection(self):
        """
        Configure the ENABLE register to power on the sensor and enable ADC.
        """
        ENABLE_CONFIGURATION = (self.TCS34725_REG_ENABLE_AEN | self.TCS34725_REG_ENABLE_PON)
        bus.write_byte_data(self.TCS34725_DEFAULT_ADDRESS, self.TCS34725_REG_ENABLE | self.TCS34725_COMMAND_BIT, ENABLE_CONFIGURATION)
    
    def time_selection(self):
        """
        Configure the integration time (ATIME) and wait time (WTIME) registers.
        """
        bus.write_byte_data(self.TCS34725_DEFAULT_ADDRESS, self.TCS34725_REG_ATIME | self.TCS34725_COMMAND_BIT, self.TCS34725_REG_ATIME_2_4)
        bus.write_byte_data(self.TCS34725_DEFAULT_ADDRESS, self.TCS34725_REG_WTIME | self.TCS34725_COMMAND_BIT, self.TCS34725_REG_WTIME_2_4)
        
    def gain_selection(self):
        """
        Configure the gain settings in the CONTROL register.
        """
        bus.write_byte_data(self.TCS34725_DEFAULT_ADDRESS, self.TCS34725_REG_CONTROL | self.TCS34725_COMMAND_BIT, self.TCS34725_REG_CONTROL_AGAIN_1)
    
    def readData(self):
        """
        Read the color data from the sensor and calculate RGB values and luminance.
        
        Returns:
            dict: Contains the raw data (sum, red, green, blue) and luminance.
        """
        data = bus.read_i2c_block_data(self.TCS34725_DEFAULT_ADDRESS, self.TCS34725_REG_CDATAL | self.TCS34725_COMMAND_BIT, 8)
        
        # Convert the data from raw sensor readings
        cData = data[1] * 256 + data[0]
        red = data[3] * 256 + data[2]
        green = data[5] * 256 + data[4]
        blue = data[7] * 256 + data[6]
        
        # Calculate luminance
        luminance = (-0.32466 * red) + (1.57837 * green) + (-0.73191 * blue)
        
        if cData == 0:
            return {'black': 0}
        
        # Normalize RGB values based on clear data
        red = red / cData * 255
        green = green / cData * 255
        blue = blue / cData * 255
        
        return {'sum': cData, 'red': red, 'green': green, 'blue': blue, 'luminance': luminance}

    def standby(self):
        """
        Put the sensor in standby mode.
        """
        self.stop = True
        self.colorDict = {'red': 0, 'green': 0, 'blue': 0}

    def resume(self):
        """
        Resume sensor operation and start continuous reading in a separate thread.
        """
        self.stop = False
        self.colorDict = {'red': 0, 'green': 0, 'blue': 0}
        colorThread = Thread(target=self.readDataContinuously)
        colorThread.start()
                    
    def readDataContinuously(self):
        """
        Continuously read color data from the sensor and update the color dictionary.
        """
        while not self._stop:
            try:
                data = bus.read_i2c_block_data(self.TCS34725_DEFAULT_ADDRESS, self.TCS34725_REG_CDATAL | self.TCS34725_COMMAND_BIT, 8)
                
                # Convert the data from raw sensor readings
                cData = data[1] * 256 + data[0]
                if cData == 0:
                    continue
                red = data[3] * 256 + data[2]
                green = data[5] * 256 + data[4]
                blue = data[7] * 256 + data[6]
                
                # Normalize RGB values based on clear data
                red = red / cData * 255
                green = green / cData * 255
                blue = blue / cData * 255
                
                # Update the color dictionary with the new values
                self.colorDict = {'red': red, 'green': green, 'blue': blue}
                sleep(0.002)  # Delay between readings
            except Exception as e:
                pass  # Handle potential errors silently
        
    @property
    def stop(self):
        """
        Property to get the stop flag.
        """
        return self._stop
    
    @stop.setter
    def stop(self, value):
        """
        Property to set the stop flag.
        """
        self._stop = value

    @property
    def datas(self):
        """
        Property to get the current color data.
        """
        return self.colorDict

