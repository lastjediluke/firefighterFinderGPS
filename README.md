## Cloning and Pushing Protocol
1. Clone repo
2. Copy, ctrl+c not drag and drop, LukeApplication/User files into Eclipse workspace under Application/User
* Drag and drop messes with eclipse, so don't use it
3. If you make changes, make a folder called ZijianApplication
4. Copy, ctrl+c, modified files from Eclipse to this folder
5. Push code to github

## Overview...
1. UART4 and UART1(console uart) are initialized in googleiot.c.
2. UART4 is receiving the GPS data to and from the ublox module.
* An interrupt is fired everytime a byte is received from the GPS.
3. The user button (blue) can be double-clicked to enter a publication loop every five seconds.
* Or it is can be pressed once to send the immediately present data.
4. We have two additional buttons that are used to control floor and status.

## ...And Where to look in our code
1. Go to the 430 folder.
2. Most of the uart and interrupt initialization code is towards the top of googleiot.c.
3. In googleiot.c, towards line 800, there is a function called gcpiot_publishtelemetry.
* When the userbutton is pressed, this function is called and it publishes data to google iot cloud core.
* We send squad, member, temperature, floor number, status, and a gps chunk of sentences.
4. In googleiot.c, towards line 100, we have declared a gps buffer.
* When an interrupt is fired on uart4, whatever is received is placed within the gps buffer.
* When the buffer is filled, we begin to refill the buffer with the latest data and we reset the buffer iterator to 0.
5. stm32l4xx_hal_msp.c has UART4 initialization specs towards the bottom of the file.
6. stm32l4xx_it.c has some of the interrupt handler code.

## Put this code into your project
1. Download and open the Google Cloud Project from ST: https://www.st.com/en/embedded-software/x-cube-gcp.html.
2. Open up the project in Eclipse or the ST Workbench.
3. Observe the file structure of the project.
4. Follow the README of the project to setup the cloud integration for the board: certificates, MQTT topics, functions, etc.
5. Replace the files, or the contents of the files, from the Eclipse project with the ones from this github.

## Forward the data from the microcontroller -> Google Cloud Project MQTT Topic -> Firebase
1. Firebase (which is now owned by Google) can be linked to a Google Cloud Project.
2. A Firebase project is initialized by following the Firebase docs for web apps and CLI: https://firebase.google.com/docs.
3. We wrote a Firebase function that is triggered by a publication to a Google Cloud Project Pub/Sub MQTT Topic.
* Pub/Sub triggers: https://firebase.google.com/docs/functions/pubsub-events.
4. The function can be found in index.js.
5. This function takes a snapshot of the most recent publication.
6. The data being sent is JSON.  The function looks for the "gps" key.
* The "gps" key has a chunk of data containing GNGGA sentences.
7. The GNGGA is parsed by a JS parser: https://github.com/infusion/GPS.js/tree/master.
8. The parser creates an object that contains the latitude and longitude coordinates.
9. The function then returns a reference to the database and updates the appropriate member's data: temp, floor, status, coordinates, squad.




