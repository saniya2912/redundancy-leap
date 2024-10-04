import struct
import pickle
import socket
import os
import fcntl

def save_object_pose_cam(array):
    """
    Save the received array to a temporary file first, then rename it to the final file
    to avoid reading incomplete data.
    """
    temp_filename = 'object_pose_cam.pkl.tmp'
    final_filename = 'object_pose_cam.pkl'

    with open(temp_filename, 'wb') as f:
        fcntl.flock(f, fcntl.LOCK_EX)  # Lock the file exclusively for writing
        pickle.dump(array, f)
        fcntl.flock(f, fcntl.LOCK_UN)  # Release the lock after writing

    # Rename the temporary file to the final one, ensuring atomic operation
    os.rename(temp_filename, final_filename)
    print(f"Array saved to file '{final_filename}'.")

def receive_array():
    host = '10.0.58.25'  # IP address of the server/producer
    port = 12345         # Port number to connect to

    # Create a socket object
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))

    try:
        buffer = b""  # Buffer to hold incomplete data
        while True:  # Continuous loop to keep receiving data
            if len(buffer) < 4:  # We need at least 4 bytes to get the data length
                buffer += client_socket.recv(4096)

            if len(buffer) >= 4:
                # Extract the first 4 bytes (data length)
                raw_data_len = buffer[:4]
                buffer = buffer[4:]

                # Unpack the length of the data
                data_len = struct.unpack('>I', raw_data_len)[0]

                # Wait until we have received the full data
                while len(buffer) < data_len:
                    buffer += client_socket.recv(4096)

                # Extract the message of the exact length
                data = buffer[:data_len]
                buffer = buffer[data_len:]  # Keep any remaining data in the buffer

                if len(data) == data_len:  # Verify if the full data is received
                    array = pickle.loads(data)  # Unpack the array
                    print("Received array:", array)

                    # Save the received array to the file
                    save_object_pose_cam(array)

                else:
                    print(f"Data length mismatch. Expected {data_len} bytes, got {len(data)} bytes.")
                    buffer = b""  # Clear buffer in case of mismatch to avoid corrupted data.

    except (ConnectionResetError, OSError) as e:
        print(f"Connection error: {e}")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        client_socket.close()  # Close the client socket when interrupted

if __name__ == "__main__":
    receive_array()  # Run the function to continuously receive and save the matrix



# # import struct
# # import pickle
# # import socket

# # def receive_array():
# #     host = '10.0.58.25'  # IP address of the server/producer
# #     port = 12345         # Port number to connect to

# #     # Create a socket object
# #     client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# #     client_socket.connect((host, port))

# #     try:
# #         buffer = b""  # Buffer to hold incomplete data
# #         while True:  # Continuous loop to keep receiving data
# #             if len(buffer) < 4:  # We need at least 4 bytes to get the data length
# #                 buffer += client_socket.recv(4096)

# #             if len(buffer) >= 4:
# #                 # Extract the first 4 bytes (data length)
# #                 raw_data_len = buffer[:4]
# #                 buffer = buffer[4:]

# #                 # Unpack the length of the data
# #                 data_len = struct.unpack('>I', raw_data_len)[0]

# #                 # Wait until we have received the full data
# #                 while len(buffer) < data_len:
# #                     buffer += client_socket.recv(4096)

# #                 # Extract the message of the exact length
# #                 data = buffer[:data_len]
# #                 buffer = buffer[data_len:]  # Keep any remaining data in the buffer

# #                 if len(data) == data_len:  # Verify if the full data is received
# #                     array = pickle.loads(data)  # Unpack the array
# #                     print("Received array:", array)

# #                     # Save the received array to a file for Code 2 to use
# #                     with open('object_pose_cam.pkl', 'wb') as f:
# #                         pickle.dump(array, f)
# #                     print("Array saved to file 'object_pose_cam.pkl'.")

# #                 else:
# #                     print(f"Data length mismatch. Expected {data_len} bytes, got {len(data)} bytes.")
# #                     buffer = b""  # Clear buffer in case of mismatch to avoid corrupted data.

# #     except (ConnectionResetError, OSError) as e:
# #         print(f"Connection error: {e}")
# #     except KeyboardInterrupt:
# #         print("Exiting...")
# #     finally:
# #         client_socket.close()  # Close the client socket when interrupted

# # if __name__ == "__main__":
# #     receive_array()  # Run the function to continuously receive and save the matrix


# import struct
# import pickle
# import socket
# import os
# import fcntl

# def save_object_pose_cam(array):
#     """
#     Save the received array to a temporary file first, then rename it to the final file
#     to avoid reading incomplete data.
#     """
#     temp_filename = 'object_pose_cam.pkl.tmp'
#     final_filename = 'object_pose_cam.pkl'

#     with open(temp_filename, 'wb') as f:
#         fcntl.flock(f, fcntl.LOCK_EX)  # Lock the file exclusively for writing
#         pickle.dump(array, f)
#         fcntl.flock(f, fcntl.LOCK_UN)  # Release the lock after writing

#     # Rename the temporary file to the final one, ensuring atomic operation
#     os.rename(temp_filename, final_filename)
#     print(f"Array saved to file '{final_filename}'.")

# def receive_array():
#     host = '10.0.58.25'  # IP address of the server/producer
#     port = 12345         # Port number to connect to

#     # Create a socket object
#     client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     client_socket.connect((host, port))

#     try:
#         buffer = b""  # Buffer to hold incomplete data
#         while True:  # Continuous loop to keep receiving data
#             if len(buffer) < 4:  # We need at least 4 bytes to get the data length
#                 buffer += client_socket.recv(4096)

#             if len(buffer) >= 4:
#                 # Extract the first 4 bytes (data length)
#                 raw_data_len = buffer[:4]
#                 buffer = buffer[4:]

#                 # Unpack the length of the data
#                 data_len = struct.unpack('>I', raw_data_len)[0]

#                 # Wait until we have received the full data
#                 while len(buffer) < data_len:
#                     buffer += client_socket.recv(4096)

#                 # Extract the message of the exact length
#                 data = buffer[:data_len]
#                 buffer = buffer[data_len:]  # Keep any remaining data in the buffer

#                 if len(data) == data_len:  # Verify if the full data is received
#                     array = pickle.loads(data)  # Unpack the array
#                     # print("Received array of length:", len(array))
#                     print("Received array:", array)

#                     # Save the received array to the file
#                     save_object_pose_cam(array)

#                 else:
#                     print(f"Data length mismatch. Expected {data_len} bytes, got {len(data)} bytes.")
#                     buffer = b""  # Clear buffer in case of mismatch to avoid corrupted data.

#     except (ConnectionResetError, OSError) as e:
#         print(f"Connection error: {e}")
#     except KeyboardInterrupt:
#         print("Exiting...")
#     finally:
#         client_socket.close()  # Close the client socket when interrupted

# if __name__ == "__main__":
#     receive_array()  # Run the function to continuously receive and save the matrix
