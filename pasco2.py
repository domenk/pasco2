import time
import smbus

SENSOR_I2C_ADDR = 0x28

REG_PROD_ID     = 0x00
REG_SENS_STS    = 0x01
REG_MEAS_RATE_H = 0x02
REG_MEAS_RATE_L = 0x03
REG_MEAS_CFG    = 0x04
REG_CO2PPM_H    = 0x05
REG_CO2PPM_L    = 0x06
REG_MEAS_STS    = 0x07
REG_INT_CFG     = 0x08
REG_ALARM_TH_H  = 0x09
REG_ALARM_TH_L  = 0x0A
REG_PRESS_REF_H = 0x0B
REG_PRESS_REF_L = 0x0C
REG_CALIB_REF_H = 0x0D
REG_CALIB_REF_L = 0x0E
REG_SCRATCH_PAD = 0x0F
REG_SENS_RST    = 0x10

REG_MEAS_CFG_PWM_OUTEN = {0b0: "software disabled", 0b1: "software enabled"}
REG_MEAS_CFG_PWM_MODE = {0b0: "single-pulse", 0b1: "pulse-train"}
REG_MEAS_CFG_BOC_CFG = {0b00: "ABOC disabled", 0b01: "ABOC enabled", 0b10: "forced compensation", 0b11: "reserved"}
REG_MEAS_CFG_OP_MODE = {0b00: "idle", 0b01: "single-shot", 0b10: "continuous", 0b11: "reserved"}


def read_value(register):
	value = bus.read_i2c_block_data(SENSOR_I2C_ADDR, register, 1)
	return value[0]

def read_value_double(register):
	value = bus.read_i2c_block_data(SENSOR_I2C_ADDR, register, 2)
	return (value[0] << 8) | value[1]

def write_value(register, value):
	bus.write_i2c_block_data(SENSOR_I2C_ADDR, register, [value])
	time.sleep(0.1)

def write_value_double(register, value):
	bus.write_i2c_block_data(SENSOR_I2C_ADDR, register, [value >> 8, value & 0b11111111])
	time.sleep(0.1)

def sensor_set_measurement_rate(rate): # rate = seconds
	write_value(REG_MEAS_CFG, read_value(REG_MEAS_CFG) & 0b11111100) # set mode to idle
	time.sleep(1)

	write_value_double(REG_MEAS_RATE_H, rate)

	write_value(REG_MEAS_CFG, (read_value(REG_MEAS_CFG) & 0b11111100) | 0b10) # set mode to continuous
	time.sleep(1)


bus = smbus.SMBus(1)

sensor_prod_id = read_value(REG_PROD_ID)
print("--- Product and revision ID ---")
print("Product ID: {}".format((sensor_prod_id >> 5) & 0b111))
print("Revision ID: {}".format(sensor_prod_id & 0b11111))
print("")

sensor_status = read_value(REG_SENS_STS)
print("--- Sensor status ---")
print("Sensor ready: {}".format((sensor_status >> 7) & 0b1))
print("PWM_DIS pin status: {}".format((sensor_status >> 6) & 0b1))
print("OOR temp error: {}".format((sensor_status >> 5) & 0b1))
print("OOR VDD12V error: {}".format((sensor_status >> 4) & 0b1))
print("Comm error: {}".format((sensor_status >> 3) & 0b1))
print("")

sensor_meas_rate = read_value_double(REG_MEAS_RATE_H)
sensor_meas_sts = read_value(REG_MEAS_STS) # reading MEAS_CFG resets DRDY in MEAS_STS, so we read MEAS_STS first
sensor_meas_cfg = read_value(REG_MEAS_CFG)
print("--- Measurement ---")
print("Measurement period: {} s".format(sensor_meas_rate))
print("PWM output: {}".format(REG_MEAS_CFG_PWM_OUTEN[(sensor_meas_cfg >> 5) & 0b1]))
print("PWM mode: {}".format(REG_MEAS_CFG_PWM_MODE[(sensor_meas_cfg >> 4) & 0b1]))
print("Baseline offset: {}".format(REG_MEAS_CFG_BOC_CFG[(sensor_meas_cfg >> 2) & 0b11]))
print("Operating mode: {}".format(REG_MEAS_CFG_OP_MODE[sensor_meas_cfg & 0b11]))
print("Data ready: {}".format((sensor_meas_sts >> 4) & 0b1))
print("INT pin status: {}".format((sensor_meas_sts >> 3) & 0b1))
print("Alarm notification: {}".format((sensor_meas_sts >> 2) & 0b1))
print("")

print("--- Pressure compensation ---")
print("Pressure: {} hPa".format(read_value_double(REG_PRESS_REF_H)))
print("")

print("--- Automatic baseline offset compensation ---")
print("ABOC: {} ppm".format(read_value_double(REG_CALIB_REF_H)))
print("")

print("--- CO2 concentration ---")
print("{} ppm".format(read_value_double(REG_CO2PPM_H)))

concentration_waiting = False
while True:
	try:
		if ((read_value(REG_MEAS_STS) >> 4) & 0b1) == 1:
			if concentration_waiting:
				print("\033[F\033[K", end="")
				concentration_waiting = False
			print("{} ppm".format(read_value_double(REG_CO2PPM_H)))
		if not concentration_waiting:
			print("[... waiting for new data (period is {} s) ...]".format(sensor_meas_rate))
			concentration_waiting = True
	except OSError: # sometimes communication is dropped
		None
	time.sleep(1)


# reset sticky registers

#write_value(REG_SENS_STS, read_value(REG_SENS_STS) | 0b100) # ORTMP_CLR
#write_value(REG_SENS_STS, read_value(REG_SENS_STS) | 0b010) # ORVS_CLR
#write_value(REG_SENS_STS, read_value(REG_SENS_STS) | 0b001) # ICCER_CL

#write_value(REG_MEAS_STS, read_value(REG_MEAS_STS) | 0b10) # INT_STS_CLR
#write_value(REG_MEAS_STS, read_value(REG_MEAS_STS) | 0b01) # ALARM_CLR


# soft reset

#write_value(REG_SENS_RST, 0xA3) # soft reset
#write_value(REG_SENS_RST, 0xBC) # reset ABOC context
#write_value(REG_SENS_RST, 0xCF) # force-save calibration offset to non-volatile memory
#write_value(REG_SENS_RST, 0xDF) # disable Stepwise Reactive IIR Filter
#write_value(REG_SENS_RST, 0xFE) # enable Stepwise Reactive IIR Filter
#write_value(REG_SENS_RST, 0xFC) # reset forced calibration correction factor
