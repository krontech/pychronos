# JEDEC Serial Presence Detect
import os, fcntl

def spdRead(id=0, bus="/dev/i2c-1"):
    """Attempt to read the JEDEC Serial Presence Detect data.

    Parameters
    ----------
    id : `int`, optional
        The slot identifier to be read (default: zero)
    bus : `str`, optional
        The path to the I2C bus to read from (default: "/dev/i2c-1")
    
    Returns
    -------
    `spd` : Class containing the JEDEC SPD data.
    """
    ## From linux/i2c-dev.h to set slave address.
    I2C_SLAVE = 0x0703

    # Write zero to set the byte offset for EEPROM readout.
    wbuf = bytearray([0])
    fd = os.open(bus, os.O_RDWR)
    fcntl.ioctl(fd, I2C_SLAVE, (0x50 + id))

    # Set offset and read SPD data.
    try:
        os.write(fd, bytearray([0]))
        data = os.read(fd, 128)
    except:
        os.close(fd)
        return None
    os.close(fd)
    return spd(data)

class spd:
    ## SPD Constants and enumerations
    SPD_TYPE_SDRAM = 4
    SPD_TYPE_DDR = 7
    SPD_TYPE_DDR2 = 8
    SPD_TYPE_DDR3 = 11

    SPD_MODULE_UDIMM = 2
    SPD_MODULE_SODIMM = 3
    SPD_MODULE_LRDIMM = 11

    def __init__(self, data):
        self.data = data
    
    def __bignum(self, number, units):
        bits = number.bit_length() // 10
        suffix = {
            0: "",
            1: "ki",
            2: "Mi",
            3: "Gi",
            4: "Ti",
            5: "OMGi"
        }
        return "%s %s%s" % (number >> (bits * 10), suffix[bits], units)

    def __str__(self):
        typeNames = {
            self.SPD_TYPE_SDRAM: "SDRAM",
            self.SPD_TYPE_DDR: "DDR SDRAM",
            self.SPD_TYPE_DDR2: "DDR2",
            self.SPD_TYPE_DDR3: "DDR3",
        }
        modNames = {
            self.SPD_MODULE_UDIMM: "UDIMM",
            self.SPD_MODULE_SODIMM: "SODIMM",
            self.SPD_MODULE_LRDIMM: "LRDIMM",
        }

        s = "JEDEC SPD Version: %s" % (self.version)
        s += "\nMemory Size: %s" % (self.__bignum(self.size, "B"))
        s += "\nMemory Type: %s" % (typeNames[self.memoryType] if self.memoryType in typeNames else "Unknown")
        s += "\nModule Type: %s" % (modNames[self.module] if self.module in modNames else "Unknown")
        s += "\nOperating Voltages: %s" % (self.voltages)
        s += "\nMemory Chip Size: %s" % (self.__bignum(self.chipSize, "b"))
        s += "\nAddress Bits: %s" % (self.bankBits + self.rowBits + self.colBits)
        s += "\nRanks: %s" % (self.ranks)

        s += "\nMemory Width: %s bits" % (self.dataBits)
        if (self.eccBits != 0):
            s += " with ECC"
        s += " (%sx%s bits)" % (self.dataBits // self.ioBits, self.ioBits)
        
        s += "\nManufacture ID: %s" % (hex(self.mfrId))
        s += "\nManufacture Date: %s%s" % (self.mfrYear, self.mfrWeek)
        s += "\nSerial Number: %s" % (self.serial)
        return s

    def nsec(self, time):
        """Convert a timing parameter in the medium time base into nanoseconds"""
        mul, div = self.mtb
        return (time * mul) / div

    @property
    def size(self):
        """Total module size in bytes"""
        if (self.memoryType != self.SPD_TYPE_DDR3):
            raise NotImplementedError("SPD parsing only supported for DDR")
        
        wordSize = self.dataBits // 8
        addrBits = self.bankBits + self.rowBits + self.colBits
        return (wordSize << addrBits) * self.ranks

    @property
    def version(self):
        """JEDEC SPD version number"""
        return "%s.%s" % ((self.data[1] & 0xf0) >> 4, self.data[1] & 0x0f)
    
    @property
    def memoryType(self):
        """Type of RAM chips"""
        return self.data[2]
    
    @property
    def module(self):
        """Type of module"""
        return self.data[3]
    
    @property
    def bankBits(self):
        """Bank address bits"""
        return ((self.data[4] & 0x70) >> 4) + 3
    
    @property
    def chipSize(self):
        """Bits per chip"""
        return 1 << (((self.data[4]) & 0x0f) + 28)

    @property
    def rowBits(self):
        """Row address bits"""
        return ((self.data[5] & 0x38) >> 3) + 12
    
    @property
    def colBits(self):
        """Column address bits"""
        return ((self.data[5] & 0x07) >> 0) + 9
    
    @property
    def voltages(self):
        """Module voltages supported"""
        volts = []
        if ((self.data[6] & 0x01) == 0): volts.append(1.5)
        if ((self.data[6] & 0x02) != 0): volts.append(1.35)
        if ((self.data[6] & 0x04) != 0): volts.append(1.25)
        return volts

    @property
    def ioBits(self):
        """IO bits per chip"""
        return 1 << ((self.data[7] & 0x7) + 2)
    
    @property
    def ranks(self):
        """Ranks per module"""
        return ((self.data[7] & 0x38) >> 3) + 1
    
    @property
    def eccBits(self):
        """ECC bits"""
        return (self.data[8] & 0x18) >> 3
    
    @property
    def dataBits(self):
        """Data bits per module"""
        return 1 << ((self.data[8] & 0x7) + 3)
    
    @property
    def ftb(self):
        """Fine time base dividend/divisor"""
        return ( (self.data[9] & 0xf0) >> 4, (self.data[9] & 0x0f) >> 0 )
    
    @property
    def mtb(self):
        """Medium time base dividend/divisor"""
        return ( self.data[10], self.data[11] )
    
    @property
    def tCkMin(self):
        """Minimum cycle time"""
        return self.data[12]
    
    @property
    def casLatencies(self):
        """CAS latencies supported""" 
        cl = []
        for i in range(0, 8):
            if (self.data[14] & (1 << i)):
                cl.append(i + 4)
        for i in range(0, 8):
            if (self.data[15] & (1 << i)):
                cl.append(i + 12)
        return cl
    
    @property
    def tAAmin(self):
        """Minimum CAS latency time"""
        return self.data[16]
    
    @property
    def tWRmin(self):
        """Minimum write recovery time"""
        return self.data[17]
    
    @property
    def tRCDmin(self):
        """Minimum RAS to CAS delay time"""
        return self.data[18]
    
    @property
    def tRRDmin(self):
        """Minimum row to row active delay time"""
        return self.data[19]
    
    @property
    def tRPmin(self):
        """Minimum row precharge time"""
        return self.data[20]
    
    @property
    def tRASmin(self):
        """Minimum active to time"""
        return ((self.data[21] & 0x0f) << 8) + self.data[22]
    
    @property
    def tRCmin(self):
        """Minimum active to active/refresh"""
        return ((self.data[21] & 0xf0) << 4) + self.data[23]
    
    @property
    def tRFCmin(self):
        """Minimum refresh recovery delay"""
        return (self.data[25] << 8) + self.data[24]
    
    @property
    def tWTRmin(self):
        """Minimum internal write to read delay"""
        return self.data[26]
    
    @property
    def tRTPmin(self):
        """Minimum internal read to precharge delay"""
        return self.data[27]
    
    @property
    def tFAWmin(self):
        """Minimum four active window delay"""
        return ((self.data[28] & 0x0f) << 8) + self.data[29]

    @property
    def options(self):
        """SDRAM optional features support"""
        opt = []
        if (self.data[30] & 0x01): opt.append("RZQ/6")
        if (self.data[30] & 0x02): opt.append("RZQ/7")
        if (self.data[30] & 0x80): opt.append("DLL-off")
        if (self.data[31] & 0x01): opt.append("ETR 95c")
        if (self.data[31] & 0x02): opt.append("ETR 1x")
        if (self.data[31] & 0x04): opt.append("ASR")
        if (self.data[31] & 0x08): opt.append("ODTS")
        if (self.data[31] & 0x80): opt.append("PASR")
        return opt

    ## TODO: DIMM thermal sensor and non-standard SDRAM type

    @property
    def height(self):
        """Module nominal height"""
        return (self.data[60] & 0x1f)
    
    @property
    def backThickness(self):
        """Module back thickness"""
        return (self.data[61] & 0xf0) >> 4
    
    @property
    def frontThickness(self):
        """Module front thickness"""
        return (self.data[61] & 0x0f) >> 0
    
    @property
    def mfrId(self):
        """Manufacturer ID"""
        return (self.data[118] << 8) | self.data[117]
    
    @property
    def mfrYear(self):
        """Manufacturing year"""
        return ((self.data[120] & 0xf0) >> 4) * 10 + (self.data[120] & 0x0f)
    
    @property
    def mfrWeek(self):
        """Manufacturing week"""
        return ((self.data[121] & 0xf0) >> 4) * 10 + (self.data[121] & 0x0f)
    
    @property
    def serial(self):
        """Module serial number"""
        ## Assuming to be little-endian.
        serial = self.data[122]
        serial += (self.data[123] << 8)
        serial += (self.data[124] << 16)
        serial += (self.data[125] << 24)
        return serial
