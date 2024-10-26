import matplotlib.pyplot as plt
from radarclass import RadarDataReader

config_file_path = 'config_files/x16_range_doppler_heatmap.cfg'
radar_data_reader = RadarDataReader(config_file_path)

# Configure the serial ports
port_cli = '/dev/ttyACM0'
port_data = '/dev/ttyACM1'
cli_port_baud = 115200
data_port_baud = 921600

# Open the serial ports
cli_port, data_port = radar_data_reader.serial_config(port_cli, port_data, cli_port_baud, data_port_baud)
cf = radar_data_reader.parse_config_file(num_rx_ant=4, num_tx_ant=2)

plt.figure(figsize=(6, 6))
# Continuously read radar data
try:
    while True:
        range_doppler, range_profile, detected_objects, range_azimuth, frameOk = radar_data_reader.read_radar_data(data_port, cf)

        if frameOk:
            plt.clf()
            plt.title("Range doppler plot")
            plt.contourf(range_doppler)
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
