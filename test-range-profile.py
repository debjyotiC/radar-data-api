import time
import numpy as np
import matplotlib.pyplot as plt
from radarclass import RadarDataReader

config_file_path = 'config_files/AWR294X_Range_Profile.cfg'
radar_data_reader = RadarDataReader(config_file_path)

# Configure the serial ports
port_cli = '/dev/tty.usbmodemRA2902371'
port_data = '/dev/tty.usbmodemRA2902374'
cli_port_baud = 115200
data_port_baud = 852272

# Open the serial ports
cli_port, data_port = radar_data_reader.serial_config(port_cli, port_data, cli_port_baud, data_port_baud)
cf = radar_data_reader.parse_config_file(num_rx_ant=4, num_tx_ant=2)

plt.figure(figsize=(6, 6))
# Continuously read radar data
try:
    while True:
        range_doppler, range_profile, detected_objects, frameOk = radar_data_reader.read_radar_data(data_port, cf)

        if frameOk:
            plt.clf()
            plt.title("Range doppler plot")
            plt.plot(range_profile)
            plt.xlabel("Range bins")
            plt.ylabel("Doppler bins")
            plt.pause(0.5)

except KeyboardInterrupt:
    print("Stopping radar data reading.")

finally:
    if cli_port is not None and cli_port.is_open:
        cli_port.close()
    if data_port is not None and data_port.is_open:
        data_port.close()
