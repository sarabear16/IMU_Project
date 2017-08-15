import ctypes.wintypes
import ctypes
import os
import time
from helpers import Singleton, AppError

STRING_BUFFER_SIZE = 128

#
# CP2110 signal pin assignments
#
CP2110_GIO0_CLK = 0
CP2110_GIO1_RTS = 1
CP2110_GIO2_CTS = 2
CP2110_GIO3_RS485 = 3
CP2110_GIO4_TX_TOGGLE = 4
CP2110_GIO5_RX_TOGGLE = 5
CP2110_GIO6 = 6
CP2110_GIO7 = 7
CP2110_GIO8_RESET = 8
CP2110_GIO9_BACKDOOR_EN = 9
CP2110_TX = 10
CP2110_SUSPEND = 11
CP2110_SUSPEND_BAR = 12
CP2110_VID = 0x10C4
CP2110_PID = 0xEA80
CP2110_READ_TIMEOUT = 20
CP2110_WRITE_TIMEOUT = 50

#
# Determines if deviceString contains a vendor ID string, product ID string, serial
# string, device path string, manufacturer string, or product string.
#
HID_UART_GET_VID_STR = 0x01
HID_UART_GET_PID_STR = 0x02
HID_UART_GET_PATH_STR = 0x03
HID_UART_GET_SERIAL_STR = 0x04
HID_UART_GET_MANUFACTURER_STR = 0x05
HID_UART_GET_PRODUCT_STR = 0x06

STRING_LENGTH = {
    HID_UART_GET_VID_STR: 5,
    HID_UART_GET_PID_STR: 5,
    HID_UART_GET_PATH_STR: 260,
    HID_UART_GET_SERIAL_STR: 256,
    HID_UART_GET_MANUFACTURER_STR: 256,
    HID_UART_GET_PRODUCT_STR: 256
}

DECODE_LENGTH = {
    HID_UART_GET_VID_STR: 4,
    HID_UART_GET_PID_STR: 4,
    HID_UART_GET_PATH_STR: 82,
    HID_UART_GET_SERIAL_STR: 6,
    HID_UART_GET_MANUFACTURER_STR: 20,
    HID_UART_GET_PRODUCT_STR: 29
}

#
# CP2110 pin mask
#
CP2110_MASK_GPIO_0_CLK = 0x0001
CP2110_MASK_GPIO_1_RTS = 0x0002
CP2110_MASK_GPIO_2_CTS = 0x0004
CP2110_MASK_GPIO_3_RS485 = 0x0008
CP2110_MASK_TX = 0x0010
CP2110_MASK_RX = 0x0020
CP2110_MASK_GPIO_4_TX_TOGGLE = 0x0040
CP2110_MASK_GPIO_5_RX_TOGGLE = 0x0080
CP2110_MASK_SUSPEND_BAR = 0x0100
CP2110_MASK_GPIO_6 = 0x0400
CP2110_MASK_GPIO_7 = 0x0800
CP2110_MASK_GPIO_8 = 0x1000
CP2110_MASK_GPIO_9 = 0x2000
CP2110_MASK_SUSPEND = 0x4000

PIN_MASK_DICT = {
    CP2110_GIO0_CLK: CP2110_MASK_GPIO_0_CLK,
    CP2110_GIO1_RTS: CP2110_MASK_GPIO_1_RTS,
    CP2110_GIO2_CTS: CP2110_MASK_GPIO_2_CTS,
    CP2110_GIO3_RS485: CP2110_MASK_GPIO_3_RS485,
    CP2110_GIO4_TX_TOGGLE: CP2110_MASK_GPIO_4_TX_TOGGLE,
    CP2110_GIO5_RX_TOGGLE: CP2110_MASK_GPIO_5_RX_TOGGLE,
    CP2110_GIO6: CP2110_MASK_GPIO_6,
    CP2110_GIO7: CP2110_MASK_GPIO_7,
    CP2110_GIO8_RESET: CP2110_MASK_GPIO_8,
    CP2110_GIO9_BACKDOOR_EN: CP2110_MASK_GPIO_9,
    CP2110_TX: CP2110_MASK_TX,
    CP2110_SUSPEND: CP2110_MASK_SUSPEND,
    CP2110_SUSPEND_BAR: CP2110_MASK_SUSPEND_BAR
}

#
# CP2110 error code
#

HID_UART_SUCCESS = 0x00
HID_UART_DEVICE_NOT_FOUND = 0x01
HID_UART_INVALID_HANDLE = 0x02
HID_UART_INVALID_DEVICE_OBJECT = 0x03
HID_UART_INVALID_PARAMETER = 0x04
HID_UART_INVALID_REQUEST_LENGTH = 0x05
HID_UART_READ_ERROR = 0x10
HID_UART_WRITE_ERROR = 0x11
HID_UART_READ_TIMED_OUT = 0x12
HID_UART_WRITE_TIMED_OUT = 0x13
HID_UART_DEVICE_IO_FAILED = 0x14
HID_UART_DEVICE_ACCESS_ERROR = 0x15
HID_UART_DEVICE_NOT_SUPPORTED = 0x16
HID_UART_UNKNOWN_ERROR = 0xFF

HID_UART_ERR_MSG = {
    HID_UART_DEVICE_NOT_FOUND: "HID_UART_DEVICE_NOT_FOUND",
    HID_UART_INVALID_HANDLE: "HID_UART_INVALID_HANDLE",
    HID_UART_INVALID_DEVICE_OBJECT: "HID_UART_INVALID_DEVICE_OBJECT",
    HID_UART_INVALID_PARAMETER: "HID_UART_INVALID_PARAMETER",
    HID_UART_INVALID_REQUEST_LENGTH: "HID_UART_INVALID_REQUEST_LENGTH",
    HID_UART_READ_ERROR: "HID_UART_READ_ERROR",
    HID_UART_WRITE_ERROR: "HID_UART_WRITE_ERROR",
    HID_UART_READ_TIMED_OUT: "HID_UART_READ_TIMED_OUT",
    HID_UART_WRITE_TIMED_OUT: "HID_UART_WRITE_TIMED_OUT",
    HID_UART_DEVICE_IO_FAILED: "HID_UART_DEVICE_IO_FAILED",
    HID_UART_DEVICE_ACCESS_ERROR: "HID_UART_DEVICE_ACCESS_ERROR",
    HID_UART_DEVICE_NOT_SUPPORTED: "HID_UART_DEVICE_NOT_SUPPORTED",
    HID_UART_UNKNOWN_ERROR: "HID_UART_UNKNOWN_ERROR"
}

class SLABHIDDevice():

    def __init__(self):
        self.hidLib = self.__getLib()
        self.device = ctypes.wintypes.LPVOID()

    def __getLib(self):
        _environ = dict(os.environ)
        _DIRNAME = os.path.dirname(os.path.abspath(__file__))
        try:
            dll_path = os.path.join(_DIRNAME, "dll","x64")
            os.environ['PATH'] += os.pathsep + dll_path
            _dll = ctypes.windll.LoadLibrary("SLABHIDtoUART.dll")
            os.environ.clear()
            os.environ.update(_environ)
            
            #print (dll_path)
            return _dll

        except OSError as e:
            dll_path = os.path.join(_DIRNAME, "dll","x86")
            os.environ['PATH'] += os.pathsep + dll_path
            _dll = ctypes.windll.LoadLibrary("SLABHIDtoUART.dll")
            os.environ.clear()
            os.environ.update(_environ)
            #print (dll_path)
            return _dll

       
        

    def _convert_str(self, buffer, length):
        return ''.join(map(chr, buffer[:length]))

    def get_num_devices(self):
        value = ctypes.wintypes.DWORD()
        try:
            status = self.hidLib.HidUart_GetNumDevices(ctypes.byref(value), CP2110_VID, CP2110_PID)
            if status == HID_UART_SUCCESS:  # success
                return value.value
            else:
                raise SLABHIDError(status)
        finally:
            del value

    def open(self, device_num):
        status = self.hidLib.HidUart_Open(ctypes.byref(self.device),
                                          device_num,
                                          CP2110_VID,
                                          CP2110_PID)
        if status != HID_UART_SUCCESS:
            raise SLABHIDError(status)


    def close(self):
        status = self.hidLib.HidUart_Close(self.device)
        if status != HID_UART_SUCCESS:
            raise SLABHIDError(status)

    def flush_buffers(self, clear_tx, clear_rx):  #clear_tx : True if flush TX buffer, #clear_rx True if flush RX buffer
        status = self.hidLib. HidUart_FlushBuffers(self.device, clear_tx, clear_rx)
        if status != HID_UART_SUCCESS:
            raise SLABHIDError(status)

    def reset(self):
        status = self.hidLib.HidUart_Reset(self.device)
        if status != HID_UART_SUCCESS:
            raise SLABHIDError(status)
    
    
    def is_opened(self):
        if self.device.value is None:
            return 0
        value = ctypes.wintypes.BOOL()
        try:
            status = self.hidLib.HidUart_IsOpened(self.device, ctypes.byref(value))
            if status == HID_UART_SUCCESS:
                return value.value
            else:
                raise SLABHIDError(status)
        finally:
            del value

    def read_latch(self):
        value = ctypes.wintypes.WORD()
        try:
            status = self.hidLib.HidUart_ReadLatch(self.device, ctypes.byref(value))
            if status == HID_UART_SUCCESS:
                return value.value
            else:
                raise SLABHIDError(status)
        finally:
            del value

    def write_latch(self, value, mask):
        status = self.hidLib.HidUart_WriteLatch(self.device, value, mask)
        if status != HID_UART_SUCCESS:
            raise SLABHIDError(status)

    def write(self, buffer):
        strlen = ctypes.wintypes.DWORD()
        status = self.hidLib.HidUart_Write(self.device, buffer.encode(), len(buffer), ctypes.byref(strlen))
        try:
            if status == HID_UART_SUCCESS:
                return strlen.value
            else:
                raise SLABHIDError(status)
        finally:
            del strlen

    def read(self, length):
        buffer_type = ctypes.c_ubyte * length
        buffer = buffer_type()
        strlen = ctypes.wintypes.DWORD()
        try:
            status = self.hidLib.HidUart_Read(self.device, buffer, length, ctypes.byref(strlen))
            if status == HID_UART_SUCCESS or status == HID_UART_READ_TIMED_OUT:
                # return bytes(buffer)[:(strlen.value].decode('ascii')
                return ctypes.string_at(buffer, strlen.value)
            else:
                raise SLABHIDError(status)
        finally:
            del buffer
            del buffer_type
            del strlen

    def get_product_string(self):
        buffer_type = ctypes.c_char * STRING_BUFFER_SIZE
        buffer = buffer_type()
        strlen = ctypes.wintypes.BYTE()
        try:
            status = self.hidLib.HidUart_GetProductString(self.device, buffer, ctypes.byref(strlen))
            if status == HID_UART_SUCCESS:  # success
                return ctypes.string_at(buffer, strlen.value)
            else:
                raise SLABHIDError(status)
        finally:
            del buffer
            del buffer_type
            del strlen

    def get_serial_string(self):
        buffer_type = ctypes.c_char * STRING_BUFFER_SIZE
        buffer = buffer_type()
        strlen = ctypes.wintypes.BYTE()
        try:
            status = self.hidLib.HidUart_GetSerialString(self.device, buffer, ctypes.byref(strlen))
            if status == 0:
                return ctypes.string_at(buffer, strlen.value)
            else:
                raise SLABHIDError(status)
        finally:
            del buffer
            del buffer_type
            del strlen

    def get_receive_fifo_size(self):
        transmitFifoSize = ctypes.c_ushort()
        receiveFifoSize = ctypes.c_ushort()
        errorStatus = ctypes.c_ubyte()
        lineBreakStatus = ctypes.c_ubyte()
        status = self.hidLib.HidUart_GetUartStatus(self.device,
                                                   ctypes.byref(transmitFifoSize),
                                                   ctypes.byref(receiveFifoSize),
                                                   ctypes.byref(errorStatus),
                                                   ctypes.byref(lineBreakStatus))
        try:
            if status == 0:
                return transmitFifoSize.value, \
                       receiveFifoSize.value, \
                       errorStatus.value, \
                       lineBreakStatus.value
            else:
                raise SLABHIDError(status)
        finally:
            del transmitFifoSize
            del receiveFifoSize
            del errorStatus
            del lineBreakStatus

    def set_timeouts(self, readTimeout, writeTimeout):
        status = self.hidLib.HidUart_SetTimeouts(self.device, readTimeout, writeTimeout)
        if status != HID_UART_SUCCESS:
            raise SLABHIDError(status)

    def get_timeouts(self):

        readTimeout = ctypes.wintypes.DWORD()
        writeTimeout = ctypes.wintypes.DWORD()

        status = self.hidLib.HidUart_GetTimeouts(self.device,
                                                 ctypes.byref(readTimeout),
                                                 ctypes.byref(writeTimeout))

        if status == HID_UART_SUCCESS:
            return readTimeout.value, writeTimeout.value
        else:
            raise SLABHIDError(status)

    def get_string(self, device_num, vid, pid, options):
        buffer_type = ctypes.c_char * 260
        buf = buffer_type()
        try:
            status = self.hidLib.HidUart_GetString(device_num,
                                                   vid,
                                                   pid,
                                                   buf,
                                                   options)

            if status == HID_UART_SUCCESS:
                return ctypes.string_at(buf, DECODE_LENGTH[options])
            else:
                raise SLABHIDError(status)
        finally:
            del buf
            del buffer_type

    def CP2110_Set_Pin(self, pin, activeLevel):
        if pin >= 0 and pin <= 12:
            if activeLevel:
                latch_value = 0xFFFF
            else:
                latch_value = activeLevel
            return self.write_latch(latch_value, PIN_MASK_DICT[pin])

        else:
            raise SLABHIDError(HID_UART_INVALID_PARAMETER)

    def CP2110_TogglePin(self, pin, activeLevel):
        self.CP2110_Set_Pin(pin, not activeLevel)
        time.sleep(0.1)

        self.CP2110_Set_Pin(pin, activeLevel)
        time.sleep(0.25)

        self.CP2110_Set_Pin(pin, not activeLevel)
        time.sleep(0.1)

    def SetUartConfig(self,baudrate,bytesize,parity,stopbits,rtscts):
        _baudrate = ctypes.wintypes.DWORD(baudrate)
        _bytesize = ctypes.wintypes.BYTE(bytesize)
        _parity = ctypes.wintypes.BYTE(parity)
        _stopbits = ctypes.wintypes.BYTE(stopbits)
        _rtscts = ctypes.wintypes.BYTE(rtscts)
        try:
            status = self.hidLib.HidUart_SetUartConfig(self.device,_baudrate,_bytesize,_parity,_stopbits,_rtscts)
            if status != HID_UART_SUCCESS:
                raise SLABHIDError(status)
        finally:
            del _baudrate 
            del _bytesize
            del _parity
            del _stopbits
            del _rtscts
class SLABHIDError(AppError):
    def __init__(self, error_code):
        if error_code > HID_UART_DEVICE_NOT_SUPPORTED:
            self.error_code = HID_UART_UNKNOWN_ERROR
        else:
            self.error_code = error_code

    def __str__(self):
        return "<hid device error: {}>".format(HID_UART_ERR_MSG[self.error_code])

#---------------------------------------------------------------------
    
class Serial:
    
    
    
    def __init__(self,port=None,baudrate=115200,bytesize=3,parity=0,stopbits=0,timeout=None,xonxoff=False,rtscts=0,write_timeout=1000,read_timeout=1000,dsrdtr=False):
        self.ser = SLABHIDDevice();   
        self.port = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout
        self.xonxoff = xonxoff
        self.rtscts = rtscts
        self.write_timeout = write_timeout
        self.read_timeout = read_timeout
        self.dsrdtr = dsrdtr

       
        #ser.WriteLatch(port)
        
      
    def open(self,device_num = 0):
        self.ser.open(device_num)
        self.ser.set_timeouts(self.read_timeout,self.write_timeout)
        self.ser.SetUartConfig(self.baudrate,self.bytesize,self.parity,self.stopbits,self.rtscts)

    def close(self):
        self.ser.close()

    def read(self,length):
        return self.ser.read(length)

    def write(self,buffer):
        return self.ser.write(buffer)

    def reset(self):
        self.ser.reset()

    def setPin(self, pin, onOff):
        self.ser.CP2110_Set_Pin(pin, onOff)

#----------------------------------------------------------------------

