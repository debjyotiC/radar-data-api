import numpy as np
import serial
import time


class RadarDataReader:
    def __init__(self, config_file_name):
        self.config_file_name = config_file_name
        self.cli_port = None
        self.data_port = None
        self.byteBuffer = np.zeros(2 ** 15, dtype='uint8')
        self.byteBufferLength = 0

    def serial_config(self, cli_com, data_com, cli_baud, data_baud, debug=False):
        self.cli_port = serial.Serial(cli_com, cli_baud)
        self.data_port = serial.Serial(data_com, data_baud)

        # Read the configuration file and send it to the board
        config = [line.rstrip('\r\n') for line in open(self.config_file_name)]
        for i in config:
            self.cli_port.write((i + '\n').encode())
            if debug:
                print(i)
            time.sleep(0.01)

        return self.cli_port, self.data_port

    def parse_config_file(self, num_rx_ant, num_tx_ant):
        configParameters = {}  # Initialize an empty dictionary to store the configuration parameters

        # Read the configuration file and send it to the board
        config = [line.rstrip('\r\n') for line in open(self.config_file_name)]
        for i in config:

            # Split the line
            splitWords = i.split(" ")

            # Hard code the number of antennas, change if other configuration is used
            numRxAnt = num_rx_ant
            numTxAnt = num_tx_ant

            # Get the information about the profile configuration
            if "profileCfg" in splitWords[0]:
                startFreq = int(float(splitWords[2]))
                idleTime = int(splitWords[3])
                rampEndTime = float(splitWords[5])
                freqSlopeConst = float(splitWords[8])
                numAdcSamples = int(splitWords[10])
                numAdcSamplesRoundTo2 = 1

                while numAdcSamples > numAdcSamplesRoundTo2:
                    numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2

                digOutSampleRate = int(splitWords[11])

            # Get the information about the frame configuration
            elif "frameCfg" in splitWords[0]:

                chirpStartIdx = int(splitWords[1])
                chirpEndIdx = int(splitWords[2])
                numLoops = int(splitWords[3])
                numFrames = int(splitWords[4])
                framePeriodicity = float(splitWords[5])

        # Combine the read data to obtain the configuration parameters
        numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
        configParameters["numDopplerBins"] = numChirpsPerFrame // numTxAnt
        configParameters["numRangeBins"] = numAdcSamplesRoundTo2
        configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (
                2 * freqSlopeConst * 1e12 * numAdcSamples)
        configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (
                2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
        configParameters["dopplerResolutionMps"] = 3e8 / (
                2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
        configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate) / (2 * freqSlopeConst * 1e3)
        configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)

        return configParameters

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
        tlvHeaderLengthInBytes = 8
        pointLengthInBytes = 16
        magicWord = [2, 1, 4, 3, 6, 5, 8, 7]

        # Initialize variables
        magicOK = 0  # Checks if magic number has been read
        dataOK = 0  # Checks if the data has been read correctly
        frameNumber = 0
        detObj = {}
        azimMapObject = {}
        rangeProfile = []
        rangeDoppler = []

        readBuffer = data_port.read(data_port.in_waiting)
        byteVec = np.frombuffer(readBuffer, dtype='uint8')
        byteCount = len(byteVec)

        # Check that the buffer is not full, and then add the data to the buffer
        if (self.byteBufferLength + byteCount) < maxBufferSize:
            self.byteBuffer[self.byteBufferLength:self.byteBufferLength + byteCount] = byteVec[:byteCount]
            self.byteBufferLength = self.byteBufferLength + byteCount

        # Check that the buffer has some data
        if self.byteBufferLength > 16:

            # Check for all possible locations of the magic word
            possibleLocs = np.where(self.byteBuffer == magicWord[0])[0]

            # Confirm that is the beginning of the magic word and store the index in startIdx
            startIdx = []
            for loc in possibleLocs:
                check = self.byteBuffer[loc:loc + 8]
                if np.all(check == magicWord):
                    startIdx.append(loc)

            # Check that startIdx is not empty
            if startIdx:

                # Remove the data before the first start index
                if 0 < startIdx[0] < self.byteBufferLength:
                    self.byteBuffer[:self.byteBufferLength - startIdx[0]] \
                        = self.byteBuffer[startIdx[0]:self.byteBufferLength]
                    self.byteBuffer[self.byteBufferLength - startIdx[0]:] = np.zeros(
                        len(self.byteBuffer[self.byteBufferLength - startIdx[0]:]),
                        dtype='uint8')
                    self.byteBufferLength = self.byteBufferLength - startIdx[0]

                # Check that there have no errors with the byte buffer length
                if self.byteBufferLength < 0:
                    self.byteBufferLength = 0

                # word array to convert 4 bytes to a 32-bit number
                word = [1, 2 ** 8, 2 ** 16, 2 ** 24]

                # Read the total packet length
                totalPacketLen = np.matmul(self.byteBuffer[12:12 + 4], word)

                # Check that all the packet has been read
                if (self.byteBufferLength >= totalPacketLen) and (self.byteBufferLength != 0):
                    magicOK = 1

        # If magicOK is equal to 1 then process the message
        if magicOK:
            # word array to convert 4 bytes to a 32-bit number
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
            for tlvIdx in range(numTLVs):

                # word array to convert 4 bytes to a 32-bit number
                word = [1, 2 ** 8, 2 ** 16, 2 ** 24]

                # Check the header of the TLV message
                tlv_type = np.matmul(self.byteBuffer[idX:idX + 4], word)
                idX += 4
                tlv_length = np.matmul(self.byteBuffer[idX:idX + 4], word)
                idX += 4

                # Read the data depending on the TLV message
                if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:

                    # Initialize the arrays
                    x = np.zeros(numDetectedObj, dtype=np.float32)
                    y = np.zeros(numDetectedObj, dtype=np.float32)
                    z = np.zeros(numDetectedObj, dtype=np.float32)
                    velocity = np.zeros(numDetectedObj, dtype=np.float32)

                    for objectNum in range(numDetectedObj):
                        # Read the data for each object
                        x[objectNum] = self.byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        y[objectNum] = self.byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        z[objectNum] = self.byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        velocity[objectNum] = self.byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4

                    # Store the data in the detObj dictionary
                    detObj = {"numObj": numDetectedObj, "x": x, "y": y, "z": z, "velocity": velocity}
                    dataOK = 1
                elif tlv_type == MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP:

                    # Get the number of bytes to read
                    numBytes = 2 * config_parameters["numRangeBins"] * config_parameters["numDopplerBins"]

                    # Convert the raw data to int16 array
                    payload = self.byteBuffer[idX:idX + numBytes]
                    idX += numBytes
                    rangeDoppler = payload.view(dtype=np.int16)

                    # Some frames have strange values, skip those frames
                    # TO DO: Find why those strange frames happen
                    if np.max(rangeDoppler) > 10000:
                        continue

                    # Convert the range doppler array to a matrix
                    rangeDoppler = np.reshape(rangeDoppler,
                                              (config_parameters["numDopplerBins"], config_parameters["numRangeBins"]),
                                              'F')  # Fortran-like reshape
                    rangeDoppler = np.append(rangeDoppler[int(len(rangeDoppler) / 2):],
                                             rangeDoppler[:int(len(rangeDoppler) / 2)], axis=0)
                    dataOK = 1

                elif tlv_type == MMWDEMO_UART_MSG_RANGE_PROFILE:
                    rangeProfile = np.frombuffer(self.byteBuffer[idX:idX + (tlv_length - 8)], dtype=np.uint16)
                    idX += (tlv_length - 8)

                    dataOK = 1

                elif tlv_type == MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP:

                    numTxAzimAnt = 2
                    numRxAnt = 4
                    numBytes = numTxAzimAnt * numRxAnt * config_parameters["numRangeBins"] * 4;

                    q = self.byteBuffer[idX:idX + numBytes]
                    # print(q)

                    idX += numBytes
                    qrows = numTxAzimAnt * numRxAnt
                    qcols = config_parameters["numRangeBins"]
                    NUM_ANGLE_BINS = 64

                    real = q[::4] + q[1::4] * 128
                    imaginary = q[2::4] + q[3::4] * 128

                    real = real.astype(np.int16)
                    imaginary = imaginary.astype(np.int16)

                    q = real + 1j * imaginary

                    q = np.reshape(q, (qrows, qcols), order="F")

                    Q = np.fft.fft(q, NUM_ANGLE_BINS, axis=0)
                    QQ = np.fft.fftshift(abs(Q), axes=0);
                    QQ = QQ.T

                    QQ = QQ[:, 1:]
                    QQ = np.fliplr(QQ)

                    theta = np.rad2deg(np.arcsin(
                        np.array(range(-NUM_ANGLE_BINS // 2 + 1, NUM_ANGLE_BINS // 2)) * (2 / NUM_ANGLE_BINS)))
                    rangeArray = np.array(range(config_parameters["numRangeBins"])) * config_parameters[
                        "rangeIdxToMeters"]

                    posX = np.outer(rangeArray.T, np.sin(np.deg2rad(theta)))
                    posY = np.outer(rangeArray.T, np.cos(np.deg2rad(theta)))

                    # Store the data in the azimMapObject dictionary
                    azimMapObject = {"posX": posX, "posY": posY, "range": rangeArray, "theta": theta, "heatMap": QQ}

                    dataOK = 1

            # Remove already processed data
            if 0 < idX < self.byteBufferLength:
                shiftSize = totalPacketLen

                self.byteBuffer[:self.byteBufferLength - shiftSize] = self.byteBuffer[shiftSize:self.byteBufferLength]
                self.byteBuffer[self.byteBufferLength - shiftSize:] = np.zeros(
                    len(self.byteBuffer[self.byteBufferLength - shiftSize:]),
                    dtype='uint8')
                self.byteBufferLength = self.byteBufferLength - shiftSize

                # Check that there are no errors with the buffer length
                if self.byteBufferLength < 0:
                    self.byteBufferLength = 0

        return rangeDoppler, rangeProfile, detObj, azimMapObject, dataOK
