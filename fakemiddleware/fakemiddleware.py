import codecs
import socket
import json
import time
import threading
from math import sin,pi

HOST = ''
PORT = 13031
USE_FREE_SEND = True

def buildmsg(ts, type, data) -> str:
    # for a valid MW_STATUS to be recognized, type must be the first element
    # booleans must not be encased by quotes ""
    # in order right hand, left hand. For numbers each. Each number [0,255]. 0 Completely open
    # 1: Closure of index
    # 2: Closure of thumb (as in curling of the finger)
    # 3: Abduction of thumb (does not affect closure)
    # 4: Closure of middle and rest of the finfers
    s = '{"type":"'+str(type)+'","ts":'+str(ts)+',"data":{'+str(data)+'}}~'
    s = s.replace(" ", "")
    return s.encode("utf-8")

def send_response(conn, response) -> bool:
    if not response: return
    print("SENDING RESPONSE (UTF-8 encoded):", response, "\n")

    try:
        conn.send(response)
        return True
    except:
        print("Connection closed by remote while sending")
        return False

def mw_status_running(conn, ts=10000) -> str:
    d = '"status":"RUNNING","statusCode":0,"version":"3.0.0","actuationsEnabled":false,"connectedDevices":[{"macAddress": "00:00:00:00:00:00", "handSide": "right"}]'
    response = buildmsg(ts+1, "MW_STATUS", d);
    send_response(conn, response)
    return response

def devices_status_all(conn, ts=10000) -> str:
    thimbles = '[{"id":"thumb", "connected": true}, {"id":"index", "connected": true}, {"id":"middle", "connected": true}]'
    # complete payload
    d = '"devices": [{"macAddress": "00:00:00:00:00:00", "handSide": "right", "batteryLevel": 42, "charging": false, "thimbles":'+thimbles+'}]'
    response = buildmsg(ts+1, "DEVICES_STATUS", d)
    send_response(conn, response)
    return response

def calibration_success(conn) -> str:
    print("Client Requested calibration, fake success (0) for right hand (1)")
    response=b'CalibrationResult:1:0~';
    send_response(conn, response)
    return response

def thimble_tracking_all(conn, d=128) -> str:
    print("Sending fake thimble tracking data (all fingers/all hands control)")
    t = (':'+str(d))*8
    response='Tracking:TrackType1'+t+'~';
    response=response.encode("utf-8")
    send_response(conn,response)
    return response

def thimble_tracking(conn, right_index_closure=0, right_thumb_closure=0, right_thumb_abduct=0, right_middle_closure=0, left_index_closure=0, left_thumb_closure=0, left_thumb_abduct=0, left_middle_closure=0) -> str:
    print("Sending fake thimble tracking data (per finger control)")
    response='Tracking:TrackType1:'+str(right_index_closure)+':'+str(right_thumb_closure)+':'+str(right_thumb_abduct)+':'+str(right_middle_closure)+':'+str(left_index_closure)+':'+str(left_thumb_closure)+':'+str(left_thumb_abduct)+':'+str(left_middle_closure)+'~';
    response=response.encode("utf-8")
    send_response(conn,response)
    return response

def thimble_dance(conn) -> None:
    thimble_tracking_all(conn, 0)
    time.sleep(0.75)
    thimble_tracking(conn, 0,0,255,0,0,0,0,0)
    time.sleep(0.75)
    thimble_tracking_all(conn, 0)
    time.sleep(0.75)
    thimble_tracking(conn, 0,255,0,0,0,0,0,0)
    time.sleep(0.75)
    thimble_tracking_all(conn, 0)
    time.sleep(0.75)
    thimble_tracking(conn, 0,255,0,0,0,0,0,0)
    time.sleep(0.75)
    thimble_tracking(conn, 255,255,0,0,0,0,0,0)
    time.sleep(0.75)
    thimble_tracking(conn, 255,255,0,255,0,0,0,0)
    time.sleep(1.5)
    thimble_tracking_all(conn, 255)
    time.sleep(1.5)
    thimble_tracking_all(conn, 0)
    time.sleep(1.5)

def init_unity_connection(conn) -> None:
    mw_status_running(conn)
    devices_status_all(conn)
    time.sleep(0.5)
    calibration_success(conn)

def free_send(conn) -> None:
    print(conn)
    if not conn: return

    # min = 20
    # max = 255
    min = 0
    max = 255
    ex = max-min;
    sine = min
    plus = 5
    while EXECUTE_FREE_SEND:
        #thimble_dance(conn)
        #sine += plus
        #if sine >= max or sine <= min: plus *= -1
        sine = int(min + 0.5*ex*(1+sin(time.time())))
        thimble_tracking(conn, sine,0,0,0,0,0,0,0)
        time.sleep(0.1)

if __name__ == "__main__":
    t = None
    print("### WEART Fake Middleware ###")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s: #a tcp socket (STREAM)
        s.bind((HOST, PORT))
        while True:
            s.listen(1)
            conn, addr = s.accept()

            with conn:
                print('Connected by', addr)
                while True:
                    msg = conn.recv(1024).decode("utf-8").strip("~")

                    if not msg: break
                    print(msg)

                    response=None
                    # message is utf-8 encoded json string, with a stray ~ at the end
                    try:
                        msg_json = json.loads(msg)
                        ts = msg_json['ts']
                        data = msg_json['data']
                        type = msg_json['type']

                        if type=="MW_GET_STATUS":
                            # responde that middleware is RUNNING
                            # this string is correctly parsed by the python sdk
                            #msg ='{"ts":100000,"type":"MW_STATUS","data":{"status":"RUNNING", "version":"3.0.0", "statusCode":0, "errorDesc": "","actuationsEnabled":"true","connectedDevices":[]}}~'; conn.sendall(msg.encode("utf-8"))
                            # this worked with the unity sdk (type must be first
                            #msg='{"type":"MW_STATUS","ts":10000,"data":{"status":"RUNNING","statusCode":0,"version":"3.0.0","actuationsEnabled":true,"connectedDevices":[{"macAddress":"00:00:00:00:00:00", "handSide": "right"}]}}~'
                            #msg1 = msg.replace(" ", "").encode("utf-8")   
                            mw_status_running(conn)              
                        elif type=="DEVICES_GET_STATUS":
                            # https://github.com/WEARTHaptics/WEART-SDK-Python/blob/afa7aeaf478f39c8f19eeab1c4b5a79ab6a75821/weartsdk/WeArtCommon.py#L367
                            #s = '"[macAddress":"'+macAddress+'",handSide:"'+handSide+'","batteryLevel":"'+str(batteryLevel)+'","charging":"false","thimbles:{}]"'
                            #d = '"macaddress":"00:00:00:00:00:00", "handside":"right", "batteryLevel":100, "charging": "false","actuationsEnabled":"true","connectedDevices":["fakedevice"]'

                            # fake a device with given mac address, right hand, 42% charged battery and all thimbles connected
                            # each thimble a ThimbleStatus, with fields
                            # id (ActuationPoint): one of "thumb", "index", "middle"
                            # connected (bool): true/false
                            # statusCode (int): defaults to 0, ok
                            # errorDesc (str): defaults to ""
                            devices_status_all(conn)
                    except Exception as e:
                        # the message was not in json. Possibly CSV
                        # If start, respond with SDK type and version an tracking type (device)
                        # no middleware needed for this type of message
                        if "StartFromClient" in msg:
                            print("Client sent start message, hello :)")
                        elif "StopFromClient" in msg:
                            print("Client sent stop message, bye o/")
                            EXECUTE_FREE_SEND = False
                        elif msg=="StartCalibration":
                            #print("Client requested calibration, but this is not implemented yet :/")
                            # send a successful calibration report
                            # calibration messages are in CSV
                            # first number is the hand index, 0: left; 1: right
                            # second number is the status: 0: success; 1: failure
                            # hand is moved to the calibration point in unity manually (move RightHandController)
                            calibration_success(conn)
                            if USE_FREE_SEND:
                                t = threading.Thread(target=free_send, args=(conn,))
                                EXECUTE_FREE_SEND = True
                                t.start()
                        else:
                            print("Error occurred while reading message. Error follows: \t",e)

            EXECUTE_FREE_SEND = False
            if t: t.join()
