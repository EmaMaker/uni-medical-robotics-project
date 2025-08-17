# List of messages sent by the WEART Middleware

These are extracted from the source code of the Low-Level Python SDK, source available https://github.com/WEARTHaptics/WEART-SDK-Python/

Messages are defined in classes (the different .py files, eg MiddlewareStatusListener), each inheriting from the base class WeArtMessage  (possibly aso WeArtJsonMessage, WeArtCSVMessage, etc), in WeArtMessages.py.

The ID of the messages is set by the ID parameter, again listed in WeArtMessages.py, starting from line 846, MiddlewareStatusMessage: https://github.com/WEARTHaptics/WEART-SDK-Python/blob/afa7aeaf478f39c8f19eeab1c4b5a79ab6a75821/weartsdk/WeArtMessages.py#L846

The messages are decoded by de/serializing the classes, The messages are discriminatd from one another by the type attribute in the json, which must be set to a valid ID. Read below for more info.

Common variables and type/payload responses are in WeArtCommon.py

## Structure of message

### JSON
**Note**: Messages to unity are terminated by a ~. (WeArtClient.cs:54)

Messages I have observed up until now are usually utf-8 encoded json strings, containing:
- **ts**: timestamp in milliseconds, Unix epoch
- **type**: type of the message, read below
- **data**: additional payload, formatted in json.

The common structure of such a message is `b'{"type": "TYPE_AS_STRING", "ts": TIMESTAMP_AS_INT, "data": {} }'`. Where `data` is a json array to be filled as in the examples below.

### CSV (Column-Separated-Values)
I also have observed other type of messages, also in utf-8 encoded string, but in CSV (Column Separated, not _Comma Separated_). They contain info about communication between unity and the middleware, eg:

- `b'StartFromClient:SdkUnity:2.0.0:TrackType1~'`: messages sent from unity to middleware right after connection over TCP socket. Initiates comms, json messages are always sent after this. Reports SDK Version.
- `b'StopFromClient~'`: sent after client disconnects (stopping playmode in unity)
- `b'StartCalibration~'`: sent to request calibration

### Peculiarities

Initially the messages where tested against the python version of the sdk (for simplicity and ease of debug) https://github.com/WEARTHaptics/WEART-SDK-Python/

- What the documentation (README.md>Usage) lists as attributes of the object are actually part of the **data** block, and must be input in valid json syntax
- Lists are a bit finicky, I derived the correct syntax by manually creating a json dump of a list of two MiddlewareConnectedDevice objects, check the code for example
- The variables are CASE SENSITIVE

Regarding the unity version of the sdk, there are peculiarities absent from the python version

- The **type** attribute MUST BE THE FIRST TO BE LISTED IN THE JSON, otherwise the message will not be deserialized correctly and the message will be discarded by unity
- Booleans must not be encased in quotes (NO: "true"; YES: true)


## List of some type of messages

The README of the python sdk is a nice way to see all the attributes for the payload, as it offers some common examples.
If in doubt, data needed in the payload of a response can be better understood by checking the C#/Python classes for each type of message, or WeArtCommon.{cs/py} or WeArtMessages.{cs/py}.

| Type | Syntax | Comms Direction | Description | Payload
|-|-|-|-|-
| `MW_GET_STATUS` | JSON | Client->Middleware| Asks for status of the middleware | Empty
| `MW_STATUS` | JSON | Middleware->Client | Reports status of the middleware to the client | [Read below](#mw-status)
| `DEVICES_STATUS` | JSON | Client->Middleware | Asks for status of connected devices | Empty
| `DEVICES_STATUS` | JSON | Middleware->Client | Reports status of the connected devices to the client, including status of single thimbles (fingers) | [Read below](#devices-status)
| `StartFromClient` | CSV |Client->Middleware | Signals the middleware that the client as been started, reports SDK version and client type | [Read below](#startfromclient)
| `StopFromClient` | CSV | Client->Middleware|Signals the middleware the client is stopping | Empty
| `StartCalibration` | CSV | Client->Middleware | Asks the middleware for the start of a calibration procedure. Middleware must `espond with `CalibrationStatus` or `CalibrationResults` | Empty
| `CalibrationResult` | CSV | Middleware->Client | Sends client information about calibration result (either calibration is finished or calibration failed | [Read below](#calibration-result)
| `Tracking` | CSV | Middleware->Client | Sends client information about tracking of the hand | [Read below](#tracking)

### MW_STATUS {#mw-status}

#### Payload

- https://github.com/WEARTHaptics/WEART-SDK-Python#example-object
- `status` (String): IDLE, STARTING, RUNNING, STOPPING, UPLOADING_TEXTURES, CONNECTING_DEVICE, CALIBRATION
- `connectedDevices/handSide` (String): `right` or `left`, lowercase.

#### Example

```
'{"type":"MW_STATUS","ts":10000,"data":{"status":"RUNNING","statusCode":0,"version":"3.0.0","actuationsEnabled":true,"connectedDevices":[{"macAddress":"00:00:00:00:00:00", "handSide": "right"}]}}~'
```

### DEVICES_STATUS {#devices-status}

#### Payload

- https://github.com/WEARTHaptics/WEART-SDK-Python#an-example-of-a-devicestatusupdate-object
- `connectedDevices/handSide` (String): `right` or `left`, lowercase.

#### Examples

```
'{"type":"DEVICES_STATUS","ts":10001,"data":{"devices":[{"macAddress":"00:00:00:00:00:00","handSide":"right","batteryLevel":42,"charging":false,"thimbles":[{"id":"thumb","connected":true},{"id":"index","connected":true},{"id":"middle","connected":true}]}]}}~' 
```

### Calibration
### Payload
- Hand side:
    - 0: left hand
    - 1: right hand
- CalibrationResult: (WeArtCommon.py:109)
    - 0: SUCCESS
    - 1: FAILURE

#### Examples
````
response=b'CalibrationResult:1:0~';
````

### Tracking {#tracking}
#### Payload
- Column separated values
- First is the message type (_Tracking_)
- Follow 9 fields, in order
    - Tracking Type: TrackType1 is the TouchDiver
    - Right hand index closure [0, 255]
    - Right hand thumb closure [0, 255]
    - Right hand thumb ambduction [0, 255]
    - Right hand middle/rest of the fingers closure [0, 255]
    - Left hand index closure [0, 255]
    - Left hand thumb closure [0, 255]
    - Left hand thumb ambduction [0, 255]
    - Left hand middle/rest of the fingers closure [0, 255]

Where zero indicates fully open/not abducted and 255 indicates fully closed/abducted

#### Example
```
`Tracking:TrackType1:0:0:0:0:0:0:0:0~'
```

#### Extra Info

Values from these appear to be used in both the animation files and WeArtThimbleTrackingObject.cs:137
