## Cloning and Pushing Protocol
1. Clone repo
2. Copy, ctrl+c not drag and drop, Luke430Complete files into Eclipse workspace under Application/User
* Drag and drop messes with eclipse, so don't use it
3. If you make changes, make a folder called ZijianApplication
4. Copy, ctrl+c, modified files from Eclipse to this folder
5. Push code to github

## Overview...
1. Most of the code is in googleiot.c.
2. UART4 is receiving the GPS data to and from the ublox module.
* An interrupt is fired everytime a byte is received from the GPS.
3. The user button (blue) can be double-clicked to enter a publication loop every five seconds.
* Or it is can be pressed once to send the immediately present data.
4. We have two additional buttons that are used to control floor and status.

## ...And Where to look in our code
1. Go to the Luke430Complete folder.
2. Lines 100-300 have the functions and variables that we added to the project.
3. In googleiot.c, towards line 800, there is a function called gcpiot_publishtelemetry.
* When the userbutton is pressed, this function is called and it publishes data to google iot cloud core.
* We send squad, member, temperature, floor number, status, and a gps chunk of sentences in JSON format.
4. In googleiot.c, towards line 100, we have declared a gps buffer.
* When an interrupt is fired on uart4, whatever is received is placed within the gps buffer.
* When the buffer is filled, we begin to refill the buffer with the latest data and we reset the buffer iterator to 0.
5. stm32l4xx_hal_msp.c has UART4 initialization specs towards the bottom of the file (void HAL_UART_MspInit(UART_HandleTypeDef* huart)).
* HOWEVER, this is not being called, static void MX_UART4_Init(void) is declared in googleiot.c, called by uartAndGpsInit() which is called in main.c.
6. stm32l4xx_it.c has some of the interrupt handler code.
* Other handlers are in main.c and googleiot.c: void Uart4_RX_GPS_Int_Handler(void), 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart), void floorButtonISR(), void dangerButtonISR().
* The handlers have some code that debounces the interrupts so that a single button press doesn't fire twice.

## Put this code into your project
1. Download and open the Google Cloud Project from ST: https://www.st.com/en/embedded-software/x-cube-gcp.html.
2. Open up the project in Eclipse or the ST Workbench.
3. Observe the file structure of the project.
4. Follow the README of the project to setup the cloud integration for the board: certificates, MQTT topics, functions, etc.
5. Replace the files, or the contents of the files, from the Eclipse project with the ones from this github (Luke430Complete).
6. NOTE: There is some unnecessary code and functions in the project:
* The includes, gps_buff.h and gps.h were used in an older iteration of the code and are not used here.
* You can comment them out and comment out some of the their objects: gps_t, gps_buff_t.
* If you build and it throws an error, it is likely from these files.
* If you want to add GPS files to your project, follow this tutorial: http://www.openstm32.org/forumthread3284.


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




