import struct
import pickle
import socket
def receive_array():
    host = '10.0.58.25'  # or the IP of the producer if running on a different host
    port = 12345        # Port to connect to

    # Create a socket object
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))

    try:
        while True:
            # First, receive the length of the incoming data
            raw_data_len = client_socket.recv(4)  # 4 bytes for an integer
            if not raw_data_len:
                print("No data received, closing connection.")
                break
           
            # Unpack the length of the data
            data_len = struct.unpack('>I', raw_data_len)[0]  # '>I' means big-endian unsigned int

            # Now, receive the actual data based on the length
            data = b""
            while len(data) < data_len:
                packet = client_socket.recv(4096)
                if not packet:
                    break
                data += packet

            if len(data) == data_len:  # Verify if we received the correct amount of data
                array = pickle.loads(data)
                print("Received array:", array)
            else:
                print("Data length mismatch, received", len(data), "bytes, expected", data_len)

    except (ConnectionResetError, OSError) as e:
        print(f"Connection error: {e}")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        client_socket.close()  # Close the client socket

if __name__ == "__main__":
    receive_array()