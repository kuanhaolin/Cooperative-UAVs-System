import socket 
import tqdm
import os
import time
import pandas as pd
import csv
import sys

def send_info(filename):
    SEPARATOR = "<SEPARATOR>"
    host = "192.168.15.7"
    port = 5050
    BUFFER_SIZE = 4096

    file_path = os.path.join('files', filename)
    filename = os.path.basename(file_path)
    file_size = os.path.getsize(file_path)

    s = socket.socket()
    print("服務器連接中")
    s.connect((host, port))
    print("成功連接")

    s.send(f"{filename}{SEPARATOR}{file_size}".encode())

    progress = tqdm.tqdm(range(file_size), f"發送{filename}", unit="B", unit_divisor=1024)
    with open(file_path, "rb") as f:
        for _ in progress:
            bytes_read = f.read(BUFFER_SIZE)
            if not bytes_read:
                break
            s.sendall(bytes_read)
            progress.update(len(bytes_read))
        f.flush()
    sys.stdout.flush() 
    sys.stderr.flush()
    s.close()

def create_info(filename, log_path):
    try:
        send_info(filename)
        with open(log_path, 'a+') as f:
            csv_write = csv.writer(f)
            data = [filename, time.asctime(time.localtime(time.time()))]
            csv_write.writerow(data)
    except Exception as e:
        print(f"Error sending or recording file {filename}: {e}")

if __name__ == "__main__":
    #send_data()
    localtime = time.asctime(time.localtime(time.time()))
    #print(localtime)
    log_path = 'log.csv'
    df1 = pd.read_csv(log_path, encoding="utf-8", sep=",")
    for filename in os.listdir("/home/pj/files"):
        #print(df1[df1.FILENAME != file])
        if not df1[df1.FILENAME == filename].empty:
            print(f"{filename} 已經傳送過了")
        else:
            create_info(filename, log_path)