import socket
import os
import time
import pandas as pd
import csv

# 服务器地址和端口
SERVER_HOST = '192.168.15.7'
SERVER_PORT = 5050

def send_data(file):
    # 创建一个TCP socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        # 连接到服务器
        client_socket.connect((SERVER_HOST, SERVER_PORT))
        print(f"Connected to server at {SERVER_HOST}:{SERVER_PORT}")

        # 提示用户输入要传送的文件路径
        #file_name = 'example.txt'
        file_path = os.path.join('files', file)
        # 获取文件名称和大小
        file_name = os.path.basename(file_path)
        file_size = os.path.getsize(file_path)

        # 发送文件信息
        client_socket.send(f"{file_name}|{file_size}".encode())

        # 传送文件内容
        start_time = time.time()
        transferred = 0
        with open(file_path, 'rb') as file:
            while transferred < file_size:
                try:
                    data = file.read(1024)
                    if not data:
                        break
                    client_socket.sendall(data)
                    transferred += len(data)
                    progress = int(100 * transferred / file_size)
                    elapsed_time = time.time() - start_time
                    speed = transferred / elapsed_time / 1024
                    print(f"正在傳送 {file_name} ({progress}%) - 速度: {speed:.2f} KB/s", end="\r")
                except Exception as e:
                    print(f"Error sending file {file_name}: {e}")
                    # 重试传送
                    file.seek(0)
                    transferred = 0
                    continue

        print(f'\n文件 {file_name} 傳送完成')
    except Exception as e:
        print(f"Error connecting to server or sending file: {e}")
    finally:
        # 关闭socket连接
        try:
            client_socket.close()
        except Exception as e:
            print(f"Error closing socket: {e}") 

def create_info(file):
    path  = "log.csv"
    try:
        send_data(file)
        with open(path, 'a+') as f:
            csv_write = csv.writer(f)
            data = [file, time.asctime(time.localtime(time.time()))]
            csv_write.writerow(data)
    except Exception as e:
        print(f"Error sending or recording file {file}: {e}")

if __name__ == "__main__":
    #send_data()
    localtime = time.asctime(time.localtime(time.time()))
    print(localtime)

    df1 = pd.read_csv('log.csv', encoding="utf-8", sep=",")
    for file in os.listdir("/home/pj/files"):
        #print(df1[df1.FILENAME != file])
        if not df1[df1.FILENAME == file].empty:
            print(f"{file} 已經傳送過了")
        else:
            create_info(file)



    # df1 = pd.read_csv('log.csv', encoding="utf-8", sep=",")
    # print(df1)
    # print(df1[df1.FILENAME == "file1.txt"])
    # nba = pd.read_csv("log.csv")
    # mask = 
    # # 要檢查的FILENAME
    # filename_to_check = 'test1.txt'

    # # 檢查FILENAME是否存在於檔案中
    # filename_exists = any(row['FILENAME'] == filename_to_check for row in existing_data)

    # if filename_exists:
    #     print(f"{filename_to_check} 已存在於檔案中。")
    # else:
    #     print(f"{filename_to_check} 不存在於檔案中。")

    #df1 = pd.read_csv('log.csv', encoding="utf-8", sep=",")

    # 將DataFrame寫入CSV檔案
    #print(df1)
    
