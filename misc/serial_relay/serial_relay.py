# keyboard==0.13.5
# pyserial==3.5

import datetime
import keyboard
import serial
import serial.tools.list_ports
import time

def log(*args):
  print(datetime.datetime.now().isoformat(), *args)

sel_serial = None

last_down = [False] * 4

while True:
  try:
    for port, desc, hwid in serial.tools.list_ports.comports():
      log(port, desc)
      # Windows: port="COMxx" desc="USB 串行设备 (COMxx)"
      # macOS: port="/dev/cu.usbmodemxxxxxx" desc="DAPLink CMSIS-DAP - mbed Serial Port"
      # macOS: port="/dev/cu.usbmodemxxxxxx" desc="Debugprobe on Pico (CMSIS-DAP)"
      if 'USB' in desc or 'CMSIS' in desc:
        sel_serial = serial.Serial(port, 115200)
        break

    if sel_serial == None:
      raise Exception('No suitable serial device found')

    log(sel_serial)

    while True:
      cur_down = [keyboard.is_pressed(c) for c in ['q', 'w', 'e', 'r']]
      for last, cur, c in zip(last_down, cur_down, ['q', 'w', 'e', 'r']):
        if last != cur:
          data = '~AICHO+' + (c.upper() if cur else c.lower())
          sel_serial.write(data.encode())
      last_down = cur_down
      time.sleep(0.01)

  except Exception as e:
    log(e)
    if sel_serial is not None:
      sel_serial.close()
      sel_serial = None
    time.sleep(1)
