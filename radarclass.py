import numpy as np
import serial
import time
import logging


class RadarDataReader:
    def __init__(self, config_file_name):
        self.config_file_name = config_file_name
        self.cli_port = None
        self.data_port = None
        self.byteBuffer = None
        self.byteBufferLength = None

    def serial_config(self, port_cli, port_data, cli_port_baud, data_port_baud):
        self._configure_logging()
        port_found = False

        while not port_found:
            try:
                self.cli_port = serial.Serial(port_cli, cli_port_baud)
                self.data_port = serial.Serial(port_data, data_port_baud)
                port_found = True
            except serial.SerialException:
                logging.error("Serial port not found. Retrying in 1 second...")
                time.sleep(1)

        self._send_config_to_board()

        return self.cli_port, self.data_port

    def _send_config_to_board(self):
        try:
            with open(self.config_file_name, 'r') as config_file:
                config_lines = config_file.readlines()

            for line in config_lines:
                command = line.strip()
                self.cli_port.write((command + '\n').encode())
                logging.info(f"Sent command: {command}")
                time.sleep(0.01)
        except IOError as e:
            logging.error(f"Error reading config file: {e}")

    def _configure_logging(self):
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    def parse_config_file(self, ):
        configParameters = {}  # Initialize an empty dictionary to store the configuration parameters

        # Initialize default values
        numRxAnt = 4  # Number of receive antennas
        numTxAnt = 2  # Number of transmit antennas
        startFreq = 0
        idleTime = 0
        rampEndTime = 0
        freqSlopeConst = 0
        numAdcSamples = 0
        numAdcSamplesRoundTo2 = 1
        digOutSampleRate = 0
        chirpStartIdx = 0
        chirpEndIdx = 0
        numLoops = 0
        numFrames = 0
        framePeriodicity = 0

        try:
            with open(self.config_file_name, 'r') as file:
                config = [line.strip() for line in file]

            for line in config:
                splitWords = line.split()

                if not splitWords:
                    continue

                if splitWords[0] == "profileCfg":
                    startFreq = float(splitWords[2])
                    idleTime = int(splitWords[3])
                    rampEndTime = float(splitWords[5])
                    freqSlopeConst = float(splitWords[8])
                    numAdcSamples = int(splitWords[10])
                    numAdcSamplesRoundTo2 = 1

                    while numAdcSamplesRoundTo2 < numAdcSamples:
                        numAdcSamplesRoundTo2 *= 2

                    digOutSampleRate = int(splitWords[11])

                elif splitWords[0] == "frameCfg":
                    chirpStartIdx = int(splitWords[1])
                    chirpEndIdx = int(splitWords[2])
                    numLoops = int(splitWords[3])
                    numFrames = int(splitWords[4])
                    framePeriodicity = int(splitWords[5])

            numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
            numDopplerBins = numChirpsPerFrame / numTxAnt
            numRangeBins = numAdcSamplesRoundTo2

            configParameters["numDopplerBins"] = numDopplerBins
            configParameters["numRangeBins"] = numRangeBins
            configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (
                    2 * freqSlopeConst * 1e12 * numAdcSamples)
            configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (
                    2 * freqSlopeConst * 1e12 * numRangeBins)
            configParameters["dopplerResolutionMps"] = 3e8 / (
                    2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numDopplerBins * numTxAnt)
            configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate) / (2 * freqSlopeConst * 1e3)
            configParameters["maxVelocity"] = 3e8 / (
                    4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)

            return configParameters

        except FileNotFoundError:
            print(f"Error: The file '{self.config_file_name}' was not found.")
            return None
        except ValueError as ve:
            print(f"Error: Value error occurred - {ve}")
            return None
        except Exception as e:
            print(f"Error: An unexpected error occurred - {e}")
            return None

    def read_radar_data(self, data_port, config_parameters):
        # Constants
        OBJ_STRUCT_SIZE_BYTES = 12
        BYTE_VEC_ACC_MAX_SIZE = 2 ** 15
        MMWDEMO_UART_MSG_DETECTED_POINTS = 1
        MMWDEMO_UART_MSG_RANGE_PROFILE = 2
        MMWDEMO_OUTPUT_MSG_NOISE_PROFILE = 3
        MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP = 4
        MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP = 5
        maxBufferSize = 2 ** 15
        magicWord = [2, 1, 4, 3, 6, 5, 8, 7]

        # Initialize variables
        magicOK = 0  # Checks if magic number has been read
        dataOK = 0  # Checks if the data has been read correctly
        frameNumber = 0
        detObj = {}
        rangeProfile = None
        rangeDoppler = None
        tlv_type = 0

        try:
            readBuffer = data_port.read(data_port.in_waiting)
            byteVec = np.frombuffer(readBuffer, dtype='uint8')
            byteCount = len(byteVec)

            # Check that the buffer is not full, and then add the data to the buffer
            if (self.byteBufferLength + byteCount) < maxBufferSize:
                self.byteBuffer[self.byteBufferLength:self.byteBufferLength + byteCount] = byteVec[:byteCount]
                self.byteBufferLength += byteCount

            # Check that the buffer has some data
            if self.byteBufferLength > 16:
                # Check for all possible locations of the magic word
                possibleLocs = np.where(self.byteBuffer == magicWord[0])[0]

                # Confirm that this is the beginning of the magic word and store the index in startIdx
                startIdx = [loc for loc in possibleLocs if np.all(self.byteBuffer[loc:loc + 8] == magicWord)]

                # Check that startIdx is not empty
                if startIdx:
                    # Remove the data before the first start index
                    if 0 < startIdx[0] < self.byteBufferLength:
                        self.byteBuffer[:self.byteBufferLength - startIdx[0]] \
                            = self.byteBuffer[startIdx[0]:self.byteBufferLength]
                        self.byteBuffer[self.byteBufferLength - startIdx[0]:] = np.zeros(
                            len(self.byteBuffer[self.byteBufferLength - startIdx[0]:]), dtype='uint8')
                        self.byteBufferLength -= startIdx[0]

                    # Check that there have no errors with the byte buffer length
                    if self.byteBufferLength < 0:
                        byteBufferLength = 0

                    # Word array to convert 4 bytes to a 32-bit number
                    word = [1, 2 ** 8, 2 ** 16, 2 ** 24]

                    # Read the total packet length
                    totalPacketLen = np.matmul(self.byteBuffer[12:12 + 4], word)

                    # Check that all the packet has been read
                    if (self.byteBufferLength >= totalPacketLen) and (self.byteBufferLength != 0):
                        magicOK = 1

            # If magicOK is equal to 1 then process the message
            if magicOK:
                # Word array to convert 4 bytes to a 32-bit number
                word = [1, 2 ** 8, 2 ** 16, 2 ** 24]

                # Initialize the pointer index
                idX = 0

                # Read the header
                magicNumber = self.byteBuffer[idX:idX + 8]
                idX += 8
                version = format(np.matmul(self.byteBuffer[idX:idX + 4], word), 'x')
                idX += 4
                totalPacketLen = np.matmul(self.byteBuffer[idX:idX + 4], word)
                idX += 4
                platform = format(np.matmul(self.byteBuffer[idX:idX + 4], word), 'x')
                idX += 4
                frameNumber = np.matmul(self.byteBuffer[idX:idX + 4], word)
                idX += 4
                timeCpuCycles = np.matmul(self.byteBuffer[idX:idX + 4], word)
                idX += 4
                numDetectedObj = np.matmul(self.byteBuffer[idX:idX + 4], word)
                idX += 4
                numTLVs = np.matmul(self.byteBuffer[idX:idX + 4], word)
                idX += 4
                subFrameNumber = np.matmul(self.byteBuffer[idX:idX + 4], word)
                idX += 4

                # Read the TLV messages
                for _ in range(numTLVs):
                    # Check the header of the TLV message
                    try:
                        tlv_type = np.matmul(self.byteBuffer[idX:idX + 4], word)
                        idX += 4
                        tlv_length = np.matmul(self.byteBuffer[idX:idX + 4], word)
                        idX += 4
                    except Exception as e:
                        print(f"Error reading TLV header: {e}")
                        break

                    # Read the data depending on the TLV message
                    if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:
                        try:
                            # Word array to convert 4 bytes to a 16-bit number
                            word = [1, 2 ** 8]
                            tlv_numObj = np.matmul(self.byteBuffer[idX:idX + 2], word)
                            idX += 2
                            tlv_xyzQFormat = 2 ** np.matmul(self.byteBuffer[idX:idX + 2], word)
                            idX += 2

                            # Initialize the arrays
                            rangeIdx = np.zeros(tlv_numObj, dtype='int16')
                            dopplerIdx = np.zeros(tlv_numObj, dtype='int16')
                            peakVal = np.zeros(tlv_numObj, dtype='int16')
                            x = np.zeros(tlv_numObj, dtype='int16')
                            y = np.zeros(tlv_numObj, dtype='int16')
                            z = np.zeros(tlv_numObj, dtype='int16')

                            for objectNum in range(tlv_numObj):
                                # Read the data for each object
                                rangeIdx[objectNum] = np.matmul(self.byteBuffer[idX:idX + 2], word)
                                idX += 2
                                dopplerIdx[objectNum] = np.matmul(self.byteBuffer[idX:idX + 2], word)
                                idX += 2
                                peakVal[objectNum] = np.matmul(self.byteBuffer[idX:idX + 2], word)
                                idX += 2
                                x[objectNum] = np.matmul(self.byteBuffer[idX:idX + 2], word)
                                idX += 2
                                y[objectNum] = np.matmul(self.byteBuffer[idX:idX + 2], word)
                                idX += 2
                                z[objectNum] = np.matmul(self.byteBuffer[idX:idX + 2], word)
                                idX += 2

                            # Make the necessary corrections and calculate the rest of the data
                            rangeVal = rangeIdx * config_parameters["rangeIdxToMeters"]
                            dopplerIdx[dopplerIdx > (config_parameters["numDopplerBins"] / 2 - 1)] -= 65535
                            dopplerVal = dopplerIdx * config_parameters["dopplerResolutionMps"]
                            x = x / tlv_xyzQFormat
                            y = y / tlv_xyzQFormat
                            z = z / tlv_xyzQFormat

                            # Store the data in the detObj dictionary
                            detObj = {
                                "numObj": tlv_numObj,
                                "rangeIdx": rangeIdx,
                                "range": rangeVal,
                                "dopplerIdx": dopplerIdx,
                                "doppler": dopplerVal,
                                "peakVal": peakVal,
                                "x": x,
                                "y": y,
                                "z": z
                            }

                            dataOK = 1
                        except Exception as e:
                            print(f"Error processing detected points: {e}")

                    elif tlv_type == MMWDEMO_UART_MSG_RANGE_PROFILE:
                        try:
                            rangeProfile = self.byteBuffer[idX:idX + (tlv_length - 8)].view(np.uint16)
                            idX += (tlv_length - 8)
                            rangeArray = np.array(range(config_parameters["numRangeBins"])) * config_parameters[
                                "rangeIdxToMeters"]
                            # Process the range profile as needed
                        except Exception as e:
                            print(f"Error processing range profile: {e}")

                    elif tlv_type == MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP:
                        try:
                            # Get the number of bytes to read
                            numBytes = int(2 * config_parameters["numRangeBins"] * config_parameters["numDopplerBins"])
                            # Convert the raw data to int16 array
                            payload = self.byteBuffer[idX:idX + numBytes]
                            idX += numBytes
                            rangeDoppler = payload.view(dtype=np.int16)

                            # Some frames have strange values, skip those frames
                            # TO DO: Find why those strange frames happen
                            if np.max(rangeDoppler) > 10000:
                                continue

                            # Convert the range doppler array to a matrix
                            rangeDoppler = np.reshape(rangeDoppler, (
                                int(config_parameters["numDopplerBins"]), int(config_parameters["numRangeBins"])),
                                                      'F')  # Fortran-like reshape
                            rangeDoppler = np.append(rangeDoppler[int(len(rangeDoppler) / 2):],
                                                     rangeDoppler[:int(len(rangeDoppler) / 2)], axis=0)
                            rangeDoppler = 20 * np.log10(rangeDoppler)
                            # Generate the range and doppler arrays for the plot
                            rangeArray = np.array(range(config_parameters["numRangeBins"])) * config_parameters[
                                "rangeIdxToMeters"]
                            dopplerArray = np.multiply(
                                np.arange(-config_parameters["numDopplerBins"] / 2,
                                          config_parameters["numDopplerBins"] / 2),
                                config_parameters["dopplerResolutionMps"])
                        except Exception as e:
                            print(f"Error processing range doppler heat map: {e}")

                # Remove already processed data
                if 0 < idX < self.byteBufferLength:
                    shiftSize = totalPacketLen
                    self.byteBuffer[:self.byteBufferLength - shiftSize] = self.byteBuffer[
                                                                          shiftSize:self.byteBufferLength]
                    self.byteBuffer[self.byteBufferLength - shiftSize:] = np.zeros(
                        len(self.byteBuffer[self.byteBufferLength - shiftSize:]),
                        dtype='uint8')
                    self.byteBufferLength -= shiftSize

                    # Check that there are no errors with the buffer length
                    if self.byteBufferLength < 0:
                        self.byteBufferLength = 0

        except Exception as e:
            print(f"Unexpected error: {e}")

        return {
            "dataOK": dataOK,
            "frameNumber": frameNumber,
            "detObj": detObj,
            "rangeProfile": rangeProfile,
            "rangeDoppler": rangeDoppler
        }
