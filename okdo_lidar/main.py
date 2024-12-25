import serial
import struct
import math

# Function to parse the LiDAR data packet
def parse_lidar_data(data):
    if len(data) < 47:  # Ensure minimum packet size
        print("Invalid packet length")
        return None

    if data[0] != 0x54 or data[1] != 0x2C:  # Check header bytes
        print("Invalid packet header")
        return None

    # Extract fields
    packet_length = data[1]
    speed = struct.unpack('<H', data[2:4])[0] / 100  # Speed in degrees/second
    start_angle = struct.unpack('<H', data[4:6])[0] / 100  # Start angle in degrees
    end_angle = struct.unpack('<H', data[-4:-2])[0] / 100  # End angle in degrees
    checksum = struct.unpack('<H', data[-2:])[0]  # Checksum

    # Data points
    distances = []
    intensities = []
    num_points = (packet_length - 9) // 3
    for i in range(num_points):
        offset = 6 + i * 3
        distance = struct.unpack('<H', data[offset:offset+2])[0] / 1000  # Convert mm to meters
        intensity = data[offset+2]
        distances.append(distance)
        intensities.append(intensity)

    # Validate checksum (basic example, can vary per protocol)
    calculated_checksum = sum(data[:-2]) & 0xFFFF
    if calculated_checksum != checksum:
        print("Checksum mismatch")
        return None

    return {
        "speed": speed,
        "start_angle": start_angle,
        "end_angle": end_angle,
        "distances": distances,
        "intensities": intensities
    }

# Main function to read from LD06
def read_lidar_data(port='/dev/serial0', baudrate=115200):
    try:
        # Open serial connection
        with serial.Serial(port, baudrate, timeout=1) as ser:
            print("Connected to LiDAR")
            buffer = bytearray()

            while True:
                # Read incoming bytes
                buffer.extend(ser.read(1024))

                # Look for start of a valid packet
                while len(buffer) >= 47:  # Minimum packet length
                    if buffer[0] == 0x54 and buffer[1] == 0x2C:
                        # Extract packet
                        packet = buffer[:buffer[1] + 2]
                        buffer = buffer[buffer[1] + 2:]  # Remove packet from buffer

                        # Parse packet
                        data = parse_lidar_data(packet)
                        if data:
                            print(f"Speed: {data['speed']}°/s")
                            print(f"Start Angle: {data['start_angle']}°")
                            print(f"End Angle: {data['end_angle']}°")
                            print(f"Distances: {data['distances']}")
                            print(f"Intensities: {data['intensities']}")
                    else:
                        # Remove invalid byte
                        buffer.pop(0)
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Exiting program...")

# Run the script
if __name__ == "__main__":
    read_lidar_data()
