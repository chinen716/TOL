# This example finds and connects to a peripheral running the
# UART service (e.g. ble_simple_peripheral.py).
import bluetooth
import random
import struct
import time
import micropython
import machine, neopixel
import _thread
from machine import Pin, I2C
import mpu6050
import profile

from ble_advertising import decode_services, decode_name
from ble_advertising import advertising_payload
from micropython import const
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)
_IRQ_GATTS_READ_REQUEST = const(4)
_IRQ_SCAN_RESULT = const(5)
_IRQ_SCAN_DONE = const(6)
_IRQ_PERIPHERAL_CONNECT = const(7)
_IRQ_PERIPHERAL_DISCONNECT = const(8)
_IRQ_GATTC_SERVICE_RESULT = const(9)
_IRQ_GATTC_SERVICE_DONE = const(10)
_IRQ_GATTC_CHARACTERISTIC_RESULT = const(11)
_IRQ_GATTC_CHARACTERISTIC_DONE = const(12)
_IRQ_GATTC_DESCRIPTOR_RESULT = const(13)
_IRQ_GATTC_DESCRIPTOR_DONE = const(14)
_IRQ_GATTC_READ_RESULT = const(15)
_IRQ_GATTC_READ_DONE = const(16)
_IRQ_GATTC_WRITE_DONE = const(17)
_IRQ_GATTC_NOTIFY = const(18)
_IRQ_GATTC_INDICATE = const(19)


######################################
i2c = I2C(scl=Pin(22), sda=Pin(21))     #initializing the I2C method for ESP32
#i2c = I2C(scl=Pin(5), sda=Pin(4))       #initializing the I2C method for ESP8266
mpu= mpu6050.accel(i2c)
######################################
_FLAG_READ = const(0x0002)
_FLAG_WRITE_NO_RESPONSE = const(0x0004)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)
######################################
_ADV_IND = const(0x00)
_ADV_DIRECT_IND = const(0x01)
_ADV_SCAN_IND = const(0x02)
_ADV_NONCONN_IND = const(0x03)
_ENV_SENSE_UUID = [
    bluetooth.UUID(0x001A),
    bluetooth.UUID(0x002A),
    bluetooth.UUID(0x003A),
    bluetooth.UUID(0x004A),
    bluetooth.UUID(0x005A),
    bluetooth.UUID(0x006A),
    bluetooth.UUID(0x007A),
    bluetooth.UUID(0x008A),
    bluetooth.UUID(0x009A),
    bluetooth.UUID(0x010A),
    bluetooth.UUID(0x011A),
    bluetooth.UUID(0x012A),
    bluetooth.UUID(0x013A),
    bluetooth.UUID(0x014A),
    bluetooth.UUID(0x015A),
    ]
####################################################################
_UART_UUID = profile.my_UUID()
_UART_TX = (
    bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9F"),
    _FLAG_READ | _FLAG_NOTIFY,
)
_UART_RX = (
    bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9F"),
    _FLAG_WRITE | _FLAG_WRITE_NO_RESPONSE,
)
_UART_SERVICE = (
    _UART_UUID,
    (_UART_TX, _UART_RX),
)
####################################################################

np = neopixel.NeoPixel(machine.Pin(15), 8)
class BLESimpleCentral:
    def __init__(self, ble,name="001"):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        self._reset()
        self.r = None
        self.g = None
        self.b = None
        self.y = None
        ((self._handle_tx, self._handle_rx),) = self._ble.gatts_register_services((_UART_SERVICE,))
        self._connections = set()
        self._write_callback = None
        self._payload = advertising_payload(name=name, services=[_UART_UUID])
        self._advertise()
    def _reset(self):
        # Cached name and address from a successful scan.
        self._name = None
        self._addr_type = None
        self._addr = None
        self._rssi = None
        # Callbacks for completion of various operations.
        # These reset back to None after being invoked.
        self._scan_callback = None
        self._conn_callback = None
        self._read_callback = None
        # Persistent callback for when new data is notified from the device.
        self._notify_callback = None
        # Connected device.
        self._conn_handle = None
        self._start_handle = None
        self._end_handle = None
        self._tx_handle = None
        self._rx_handle = None
    def _irq(self, event, data):
        type_list = []
        rssi_list = []
        
        if event == _IRQ_SCAN_RESULT:
            addr_type, addr, adv_type, rssi, adv_data = data
            #print("Received adv_data:", adv_data) 
            if adv_type in (_ADV_IND, _ADV_DIRECT_IND):
                type_list = decode_services(adv_data)
                #rssi_list =[]
                for sensed_uuid in _ENV_SENSE_UUID:
                    if sensed_uuid in type_list: # 登録されたデバイスのuuidと一致しているか判定
                        # Found a potential device, remember it and stop scanning.
                        self._addr_type = addr_type
                        self._addr = bytes(
                            addr
                        )  # Note: addr buffer is owned by caller so need to copy it.
                        self._name = decode_name(adv_data) or "?"
                        self._ble.gap_scan(None)
                        #########重要##########
                        #print("rssi",rssi,"uuid",self._name)
                        self._rssi = None
                        RSSI.append(rssi)
                        #########重要##########
                        
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            print("New connection", conn_handle)
            self._connections.add(conn_handle)
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            print("Disconnected", conn_handle)
            self._connections.remove(conn_handle)
            # Start advertising again to allow a new connection.
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            value = self._ble.gatts_read(value_handle)
            if value_handle == self._handle_rx and self._write_callback:
                self._write_callback(value)

#         def add(a,b):
#             return a+b
#
#         _colList = np[5] # 5じゃなくても良い。今の光のいろを聞くために5人目に相談している
#         print(_colList)
#         r = _colList[0]
#         g = _colList[1]
#         b = _colList[2]
#         y = 0
#         _step = 2
#         if rssi > -60:
#             r1 = 255
#             g1 = 0
#             b1 = 0
#             y1 = 0
#         else:
#             r1 = 0
#             g1 = 0
#             b1 = 255
#             y1 = 0
#
#         _baseColDif = [(r1+(-r))/_step,(g1+(-g))/_step,(b1+(-b))/_step,(y1+(-y))/_step]# 今の光の色と変化させたい色の差分
#         #print(_baseColDif)
#         for j in range(_step-1):
#             _colDif = [int(_baseColDif[0])*(j+1)+r,int(_baseColDif[1]*(j+1)+g),int(_baseColDif[2]*(j+1)+b),int(_baseColDif[3]*(j+1)+y)]
#             #print(_colDif)
#             r2,g2,b2,y2 = _colDif[0],_colDif[1],_colDif[2],_colDif[3]
#             for i in range(16):
#                 np[i] = (r2,g2,b2,y2)
#                 np.write()#最終的な色の出力
#
#         time.sleep_ms(3)
#         return r2,g2,b2,y2
    # Returns true if we've successfully connected and discovered characteristics.
    def is_connected(self):
        return (
            self._conn_handle is not None
            and self._tx_handle is not None
            and self._rx_handle is not None
        )
    # Find a device advertising the environmental sensor service.
    def scan(self, callback=None):
        self._addr_type = None
        self._addr = None
        self._scan_callback = callback
        self._ble.gap_scan(0, 30000, 30000)
    # Connect to the specified device (otherwise use cached address from a scan).
    def connect(self, addr_type=None, addr=None, callback=None):
        self._addr_type = addr_type or self._addr_type
        self._addr = addr or self._addr
        self._conn_callback = callback
        if self._addr_type is None or self._addr is None:
            return False
        self._ble.gap_connect(self._addr_type, self._addr)
        return True
    # Disconnect from current device.
    def disconnect(self):
        if self._conn_handle is None:
            return
        self._ble.gap_disconnect(self._conn_handle)
        self._reset()
    # Send data over the UART
    def write(self, v, response=False):
        if not self.is_connected():
            return
        self._ble.gattc_write(self._conn_handle, self._rx_handle, v, 1 if response else 0)
    # Set handler for when data is received over the UART.
    def on_notify(self, callback):
        self._notify_callback = callback
    def connect_and_discover(self, addr_type=None, addr=None, callback=None):
        # ...
        self._ble.gap_connect(self._addr_type, self._addr)
########################
    def send(self, data):
        for conn_handle in self._connections:
            self._ble.gatts_notify(conn_handle, self._handle_tx, data)
    def is_connected(self):
        return len(self._connections) > 0
    def _advertise(self, interval_us=500000):
        print("Starting advertising")
        self._ble.gap_advertise(interval_us, adv_data=self._payload)
    def on_write(self, callback):
        self._write_callback = callback
#######################
RSSI = []
finalRSSI = [-100]
Acc = []
dif = [0]
thread_exit_flag = False

# ボタンのコールバック関数
def button_callback(p):
    global thread_exit_flag
    thread_exit_flag = True
    
# デフォルトのボタンをGPIO0に接続（通常、BOOTボタンがこのピンに接続されている）
button = Pin(0, Pin.IN, Pin.PULL_UP)
button.irq(trigger=Pin.IRQ_FALLING, handler=button_callback)


def thread1():
    global RSSI
    global finalRSSI
    global lock
    global thread_exit_flag
    while not thread_exit_flag:
            
        def on_scan(addr_type, addr, name,rssi):
                    if addr_type is not None:
                        print("Found peripheral:", addr_type, addr, name,rssi)
                    else:
                        nonlocal not_found
                        not_found = True
                        print("No peripheral found.")        
        def on_rx(v):
            print("RX", v)


        while True:
            #print("/----Connected-----/")
            not_found = False
            central.scan(callback=on_scan)
            time.sleep_ms(1000)
            with_response = False
            ble.gap_scan(None)
            #print("/---Disconnected---/")
            if len(RSSI) >3 :
                finalRSSI =[finalRSSI[-1]]
                finalRSSI.append(max(RSSI))
                RSSI = []
            #time.sleep_ms(500)
    _thread.exit()

def thread2():
    global RSSI
    global finalRSSI
    global lock
    global dif
    global thread_exit_flag

    while not thread_exit_flag:
        
        def lighting(rssi,fluct):

            n = np.n
            for i in range(0, 4 * 256,  15): # 一番右の数字で点滅の周期を制御できる
                #sss = random.randint(30,60)
                for j in range(n):
                    if (0 <= i <  5) or (507 <= i < 512) or (512 <= i < 517) or (1019 <= i <  1024):
                        if (rssi > -70 or fluct >1000):
                                np[j] = (1, 0, 0)
                        elif (rssi < -70 or fluct <1000):
                                np[j] = (0, 0, 1)
                    else:
                        if (i // 256) % 2 == 0:
                            val = i & 0xff
                            if (rssi >= -70 or fluct >= 1000):
                                np[j] = (val,  0,  0)
                            elif (rssi < -70 or fluct < 1000):
                                np[j] = (0, 0, val)
                        else:
                            val = 255 - (i & 0xff)
                            if (rssi >= -70 or fluct >= 2000):
                                np[j] = (val, 0, 0)
                            elif (rssi < -70 or fluct < 2000):
                                np[j] = (0, 0, val)
                np.write()
            # 消灯
            for i in range(n):
                np[i] = (0, 0, 0)
            np.write()

        while True:
            lighting(finalRSSI[-1],dif[-1])
            time.sleep_ms(100)
    _thread.exit()
  
def thread3():
    global dif
    global Acc
    global thread_exit_flag

    while not thread_exit_flag:
        while True:
            dif = []
            if len(Acc) > 3:
                del Acc[0]
            Acc.append(mpu.get_values())
            if len(Acc) >2:
                dif.append(abs(Acc[1]-Acc[0]))
            else:
                dif = [0]
            time.sleep(1)
    _thread.exit()

 ####################################
    #p.on_write(on_rx)
######################################
lock = _thread.allocate_lock()

if __name__ == "__main__":
    ble = bluetooth.BLE()
    central = BLESimpleCentral(ble)

    _thread.start_new_thread(thread1, ())  # SendThreadの起動
    _thread.start_new_thread(thread2, ())  # RecvThreadの起動
    _thread.start_new_thread(thread3, ())  # RecvThreadの起動
