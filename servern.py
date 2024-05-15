import socket
import tqdm
import os
import threading

HOST = "192.168.15.7"
PORT = 5050

SEPARATOR = "<SEPARATOR>"
BUFFER_SIZE = 4096

def receive_file(client_socket, client_address):
    """Receives a file from the connected client."""
    received = client_socket.recv(BUFFER_SIZE).decode()
    filename, file_size = received.split(SEPARATOR)

    #filename = os.path.join('file', filename)
    filename = os.path.basename(filename)
    filename = os.path.join('file', filename)
    file_size = int(int(file_size) / BUFFER_SIZE + 1)  # Calculate chunks for progress bar

    progress = tqdm.tqdm(range(file_size), f"Receiving {filename}", unit="B", unit_divisor=1024, unit_scale=True)

    with open(filename, "wb") as f:
        while True:
            bytes_read = client_socket.recv(BUFFER_SIZE)
            if not bytes_read:
                break
            f.write(bytes_read)
            progress.update(len(bytes_read))

    print(f"Received file: {filename}")
    progress.close()
    client_socket.close()  # Close the client socket after each transfer
    print(f"Closed connection with {client_address}")

def main():
    """Listens for incoming connections and receives files."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print("Listening for connections...")

        while True:
            client_socket, client_address = s.accept()
            print(f"Client {client_address} connected")

            # Create a new thread to handle the client connection
            client_thread = threading.Thread(target=receive_file, args=(client_socket, client_address))
            client_thread.start()

if __name__ == "__main__":
    main()
