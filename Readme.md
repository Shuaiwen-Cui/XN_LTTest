# TABLE OF CONTENTS

> Folder - Xnode-Base

- Base Version (v15 from Kirill)
- No Internet Connection

> Folder - Xnode-4G

- 4G Version (based on FRA version from Tu, modified by Kirill)


# Dev Log

## Xnode-4G 

### 📅 Nov 22, 2023
> Debugging
- The Problem
  - UART_Send() is working
  - UART_Receive() nothing received

- The Solution
    - Middle & Bottom Boards Replaced - not working
    - Antenna Installed - related to signal strength

> Code
- Phone number reading logic is modified. The limitation on the number of digits is removed.
  - US: 10 digits
  - SG: 8 digits

> SD Card Config
- Phone number 
- APN
- Server IP Address (MQTT Broker) - 8.214.19.225 - port 1883

> Test
- check Test-4G-2023-11-22.txt

