# Arduino example: Making a Tracker with the Charlie board
The source code in [elderly_monitor](elderly_monitor/elderly_monitor.ino) provides an example of a tracking device for elderly people monitoring.


### What you can use this example for:

With this sketch you can start prototyping applications that can fit into the following use cases:

- elderly people tracking
- position tracking
- step counting (through aggregation of data from the accelerometer)

This example:

- activates the embedded ME310 modem
- reads the data from accelerometer
- connects the board to GSM modem to send sms message
- reads the button state if pressed (if it sense the pressure for 2 seconds, it send an an alarm to a registered number)
- send the sms data with latitude and longitude and a warning text

## Before starting

There are a few checks to get your Charlie board ready for this example.

### Download and install the Telit Charlie Board in the Arduino IDE

Go to the Arduino IDE Preferences, and under "Additional Boards Manager URLs" field paste the URL https://raw.githubusercontent.com/telit/arduino-charlie/main/Arduino/package_Telit-board_index.json. Then go to the Boards Manager and search for Telit-Board. This will install the Board Package for the Charlie.

You can find a short article explaining how to do it here https://support.arduino.cc/hc/en-us/articles/360016119519-How-to-add-boards-in-the-board-manager

### Download and install the ME310 Library

Download the [arduino-me310-library](https://github.com/telit/arduino-me310-library) and [arduino-tlt-library](https://github.com/telit/arduino-tlt-library), then extract them in your Arduino libraries folder, or install the new libraries from the ZIP files.

These Libraries will simplify the interactions with the ME310G1 Module.

### Install the BMA400 library

In order to read data from the embedded BOSCH BMA400 accelerometer, download the [telit BMA400-API library porting](https://github.com/telit/arduino-BMA400-API) and install as explained in the previous section.

### SIM card and antenna requirements

Please check with your provider that the SIM card has an associated number, and is able to send SMS messages (insert it in a cell phone and try sending a message).
Also, note that the antenna must comply with the available connectivity protocols in your country (GSM/UMTS/LTE).


### Destination mobile phone number

The source code requires to setup a destination phone number where the alert messages will be sent. Please refer to the line below

``` 
char remoteNum[20] = "+39XXXXXXXXX";
```


### Additional resources

If you wish to send a google maps link containing the position, there are many posts and threads about the topic, like this one
https://stackoverflow.com/questions/30544268/create-google-maps-links-based-on-coordinates
