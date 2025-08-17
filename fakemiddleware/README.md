This folder contains the Fake Middleware.

To continue testing outside the university lab and withou an actual interface, we developed a python script that fakes being the middleware, and notifies the SDK that, in order:

- The middleware is active and has an interface connected (Middleware Status)
- The interface is working and has all three thimbles connected, charged and ready (Devices Status)
- Calibration was successful (CalibrationReturn)
- Eventually Sends fake _thimble closure tracking_ data for testing purposes

1. and 2. are necessary for the SDK to start parsing further data; 3. is necessary in the PlaygroundDemo scene and 4. is used for testing purposes.

Furthermore this fake middleware can also be used on Linux, while the actual middleware only runs on windows.

## Middleware <-> SDK communication

The communication between the middleware and SDK happens over a TCP socket, default port 13031. Luckily, while the protocol is custom, data is sent between the two parts using UTF-8 encoded strings of either JSON or Column-Separated-Value messages.

This script opens a TCP socket on port 13031, parses the messages coming from the SDK, and sends back the appropriate response.

For more info on message structure and the reverse-engineering work of this middleware, see [messages.md](./messages.md)

## Files

- `messages.md` contains info on different messagesm their structure and payloads and the reverse-engineering work done on the Unity and Python SDKs to make this middleware work
- `fakemiddleware.py` is the actual fake middleware. Use this with unity. It also contains a `free_send()` function that can be customized by the user to send appropriate data. The rest should be left as-is.
- `fakemiddleware-pythonsdk.py` is a barebones version of the fake middleware, used during development of the actual one to interact with the ExampletouchDriver.py example of the Python SDK to figure out the message structure

### To Do

 - `mitm.py` a Man In The Middle between the SDK and the real Middleware, to capture some real-world data and save it on file.


