# `RadarDataReader` Class

The `RadarDataReader` class facilitates communication with a radar sensor, configuring the radar through a configuration file, parsing configuration parameters, and reading radar data for object detection and radar heat maps. It primarily utilizes `serial` communication to transmit data from the radar sensor to a computer.

## Attributes
- **config_file_name** (`str`): Name of the radar configuration file.
- **cli_port** (`serial.Serial`): Serial port for command-line interface (CLI) communication.
- **data_port** (`serial.Serial`): Serial port for data communication.
- **byteBuffer** (`numpy.ndarray`): Buffer to store raw radar data.
- **byteBufferLength** (`int`): Length of the data in `byteBuffer`.

---

### `__init__(config_file_name: str)`

Initializes the `RadarDataReader` class.

#### Parameters:
- **config_file_name** (`str`): Path to the configuration file for radar settings.

---

### `serial_config(cli_com: str, data_com: str, cli_baud: int, data_baud: int, debug: bool = False) -> Tuple[serial.Serial, serial.Serial]`

Configures serial communication for CLI and data ports, sending configuration commands to the radar hardware.

#### Parameters:
- **cli_com** (`str`): CLI port name.
- **data_com** (`str`): Data port name.
- **cli_baud** (`int`): Baud rate for CLI communication.
- **data_baud** (`int`): Baud rate for data communication.
- **debug** (`bool`): If `True`, prints each configuration command sent to the radar hardware.

#### Returns:
- **cli_port** (`serial.Serial`): Configured serial object for CLI communication.
- **data_port** (`serial.Serial`): Configured serial object for data communication.

---

### `parse_config_file(num_rx_ant: int, num_tx_ant: int) -> dict`

Parses radar configuration parameters such as number of Doppler and range bins, resolution, and max range/velocity, based on values in the configuration file.

#### Parameters:
- **num_rx_ant** (`int`): Number of receiving antennas.
- **num_tx_ant** (`int`): Number of transmitting antennas.

#### Returns:
- **configParameters** (`dict`): Dictionary with parsed configuration parameters including:
  - `"numDopplerBins"` (`int`): Number of Doppler bins.
  - `"numRangeBins"` (`int`): Number of range bins.
  - `"rangeResolutionMeters"` (`float`): Range resolution in meters.
  - `"rangeIdxToMeters"` (`float`): Range index to meters scaling factor.
  - `"dopplerResolutionMps"` (`float`): Doppler resolution in meters per second.
  - `"maxRange"` (`float`): Maximum measurable range in meters.
  - `"maxVelocity"` (`float`): Maximum measurable velocity in meters per second.

---

### `read_radar_data(data_port: serial.Serial, config_parameters: dict) -> Tuple[numpy.ndarray, numpy.ndarray, dict, dict, int]`

Reads radar data from the data port, processes raw byte data to extract object points and various radar heat maps.

#### Parameters:
- **data_port** (`serial.Serial`): Data port for reading radar data.
- **config_parameters** (`dict`): Dictionary containing radar configuration parameters (as returned by `parse_config_file`).

#### Returns:
- **rangeDoppler** (`numpy.ndarray`): Range-Doppler heat map array with shape `(numDopplerBins, numRangeBins)`.
- **rangeProfile** (`numpy.ndarray`): Array representing range profile of detected objects.
- **detObj** (`dict`): Dictionary with detected object information:
  - `"numObj"` (`int`): Number of detected objects.
  - `"x"` (`numpy.ndarray`): X-coordinates of detected objects.
  - `"y"` (`numpy.ndarray`): Y-coordinates of detected objects.
  - `"z"` (`numpy.ndarray`): Z-coordinates of detected objects.
  - `"velocity"` (`numpy.ndarray`): Velocity of detected objects.
- **azimMapObject** (`dict`): Dictionary containing azimuth heat map and position data:
  - `"posX"` (`numpy.ndarray`): X-positions of radar data in azimuth map.
  - `"posY"` (`numpy.ndarray`): Y-positions of radar data in azimuth map.
  - `"range"` (`numpy.ndarray`): Range values for azimuth map.
  - `"theta"` (`numpy.ndarray`): Angle values in degrees for azimuth map.
  - `"heatMap"` (`numpy.ndarray`): Azimuth heat map.
- **dataOK** (`int`): Status flag (`1` if data read and parsed successfully, `0` otherwise).