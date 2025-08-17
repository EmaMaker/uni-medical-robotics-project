import codecs
import socket
import json
import time

HOST = ''
PORT = 13031

reader = codecs.getreader("utf-8")

# default values for the fake hand
macAddress = "00:00:00:00:00:00"
handSide = "RIGHT"
batteryLevel = 100
thimbles = {}

def buildmsg(ts, type, data) -> str:
    s = '{"ts":'+str(ts)+',"type":"'+str(type)+'","data":{'+str(data)+'}}~'
    return s.encode("utf-8")

if __name__ == "__main__":
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s: #a tcp socket (STREAM)
        s.bind((HOST, PORT))
        s.listen(1)
        conn, addr = s.accept()
        with conn:
            print('Connected by', addr)

            time.sleep(1)

            d = '"status":"RUNNING", "version":"3.0.0", "statusCode":0, "errorDesc": "","actuationsEnabled":"true","connectedDevices":[{"macAddress": "00:00:00:00:00:00", "handSide": "left"}]'
            response = buildmsg(0, "MW_STATUS", d);
            conn.send(response)

            time.sleep(0.005)
            d = '"devices": [{"macAddress": "00:00:00:00:00:00", "handSide": "left", "batteryLevel": 100, "charging": false, "thimbles": []}]'
            response = buildmsg(1, "DEVICES_STATUS", d)
            conn.send(response)