import time
from radarclass import RadarDataReader

config_file_path = 'path_to_config_file.cfg'
radar_data_reader = RadarDataReader(config_file_path)

# Configure the serial ports
port_cli = 'COM3'  # Replace with the actual CLI port
port_data = 'COM4'  # Replace with the actual Data port
cli_port_baud = 115200  # Baud rate for the CLI port
data_port_baud = 921600  # Baud rate for the Data port

# Open the serial ports
cli_port, data_port = radar_data_reader.serial_config(port_cli, port_data, cli_port_baud, data_port_baud)

# Continuously read radar data
try:
    while True:
        radar_data = radar_data_reader.read_radar_data()
        if radar_data['dataOK']:
            print(f"Frame Number: {radar_data['frameNumber']}")
            print(f"Detected Objects: {radar_data['detObj']}")
            if radar_data['rangeProfile'] is not None:
                print(f"Range Profile: {radar_data['rangeProfile']}")
            if radar_data['rangeDoppler'] is not None:
                print(f"Range Doppler Heat Map: {radar_data['rangeDoppler']}")

        # Add a delay if needed to avoid overloading the CPU
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping radar data reading.")

finally:
    if cli_port is not None and cli_port.is_open:
        cli_port.close()
    if data_port is not None and data_port.is_open:
        data_port.close()
