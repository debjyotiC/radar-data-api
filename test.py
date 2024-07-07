import time
from radarclass import RadarDataReader

config_file_path = 'config_files/AWR294X_Scatter.cfg'
radar_data_reader = RadarDataReader(config_file_path)

# Configure the serial ports
port_cli = 'COM4'  # Replace with the actual CLI port
port_data = 'COM5'  # Replace with the actual Data port
cli_port_baud = 115200  # Baud rate for the CLI port
data_port_baud = 852272  # Baud rate for the Data port

# Open the serial ports
cli_port, data_port = radar_data_reader.serial_config(port_cli, port_data, cli_port_baud, data_port_baud)
cf = radar_data_reader.parse_config_file(num_rx=4, num_tx=2)

# Continuously read radar data
try:
    while True:
        radar_data = radar_data_reader.readAndParseData16xx(data_port, cf)
        time.sleep(1)

except KeyboardInterrupt:
    print("Stopping radar data reading.")

finally:
    if cli_port is not None and cli_port.is_open:
        cli_port.close()
    if data_port is not None and data_port.is_open:
        data_port.close()
