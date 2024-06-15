import datetime
import serial
import serial.tools.list_ports
import time

def log(*args):
  print(datetime.datetime.now().isoformat(), *args)

sel_serial = None

while True:
  try:
    for port, desc, hwid in serial.tools.list_ports.comports():
      log(port, desc)
      # Windows: port="COMxx" desc="USB 串行设备 (COMxx)"
      # macOS: port="/dev/cu.usbmodemxxxxxx" desc="DAPLink CMSIS-DAP - mbed Serial Port"
      if 'USB' in desc or 'DAPLink' in desc:
        sel_serial = serial.Serial(port, 115200)
        break

    if sel_serial == None:
      raise Exception('No suitable serial device found')

    log(sel_serial)

    while True:
      data = '~AICHO+Q'
      sel_serial.write(data.encode())
      time.sleep(1)
      data = '~AICHO+q'
      sel_serial.write(data.encode())
      time.sleep(1)
  except Exception as e:
    log(e)
    if sel_serial is not None:
      sel_serial.close()
      sel_serial = None
    time.sleep(1)
