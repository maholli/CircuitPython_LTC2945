import board, time
import ltc2945

i2c = board.I2C()
pwr = ltc2945.LTC2945(i2c, addr=0x67, rsense=0.035)

while True:
    print(f'Voltage: {pwr.read_vin():.3f}V\nCurrent: {pwr.read_current():.3f}A\n')
    time.sleep(3)
