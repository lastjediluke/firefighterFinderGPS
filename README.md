1. Clone repo
2. Copy, ctrl+c not drag and drop, LukeApplication/User files into Eclipse workspace under Application/User
* Drag and drop messes with eclipse, so don't use it
3. If you make changes, make a folder called ZijianApplication
4. Copy, ctrl+c, modified files from Eclipse to this folder
5. Push code to github

## How it Works
1. UART4 and UART1(console uart) are initialized.
2. UART4 is sending and receiving the gps data to and from itself.
3. Buff manage is not being used. I need Zijian to work on that.
4. For now, we ideally want buff manage to:
* parse the gps message and grab gpgga data.
* store it in a variable
* print it to the console using printf



