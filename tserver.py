import socket
import threading
import os

# 服务器地址和端口
SERVER_HOST = '0.0.0.0'
SERVER_PORT = 5050

def handle_client(client_socket, client_address):
    print(f"Accepted connection from {client_address}")

    try:
        while True:
            # 接收檔案資訊
            file_info = client_socket.recv(4096).decode().split('|')
            file_name = file_info[0]
            file_size = int(file_info[1])

            # 接收檔案內容
            with open(file_name, 'wb') as file:
                received = 0
                while received < file_size:
                    data = client_socket.recv(1024)
                    if not data:
                        break
                    file.write(data)
                    received += len(data)

            print(f'檔案 {file_name} 接收完成')
    except Exception as e:
        print(f"Error handling client {client_address}: {e}")
    finally:
        # 關閉與客戶端的連接
        client_socket.close()
        print(f"Closed connection from {client_address}")

def main():
    # 創建一個TCP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 綁定到地址和端口
    server_socket.bind((SERVER_HOST, SERVER_PORT))
    # 開始監聽連線
    server_socket.listen(5)
    print(f"Listening for connections on {SERVER_HOST}:{SERVER_PORT}")

    while True:
        # 等待客戶端連線
        try:
            client_socket, client_address = server_socket.accept()
            # 創建一個線程來處理客戶端連線
            client_thread = threading.Thread(target=handle_client, args=(client_socket, client_address))
            client_thread.start()
        except Exception as e:
            print(f"Error accepting connection: {e}")

if __name__ == "__main__":
    main()