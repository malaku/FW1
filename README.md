# FW1 Repository

## Overview

Welcome to the FW1 repository, which is part of a comprehensive Vehicle ECU and CAN Modeling project. This project includes the modeling and simulation of vehicle communication through CAN (Controller Area Network) and Ethernet, distributed across various subsystems.

The FW1 repository specifically contains the code for the **Front Left Wheel ECU**. It is part of a larger system involving the following related repositories:

- **Vehicle-and-Driver-Model**: Central vehicle and driver simulation.
- **FW2**: Front right wheel ECU.
- **RW1**: Rear left wheel ECU.
- **RW2**: Rear right wheel ECU.
- **WAC1**: Wheel Angle Calculator 1.
- **WAC2**: Wheel Angle Calculator 2.
- **WAC3**: Wheel Angle Calculator 3.
- **WAC4**: Wheel Angle Calculator 4.

## Code Content

The FW1 codebase includes the implementation of CAN and Ethernet communication protocols, specifically within the `ert_main.c` file. This file is crucial for ensuring that the Front Right Wheel ECU can communicate effectively within the vehicle's network.

### Key Files:

- **`ert_main.c`**: Contains the main program, including CAN and Ethernet implementations.
- **`FW1.c`**: Core logic for the front left wheel ECU.
- **`linuxinitialize.c`**: Linux-specific initialization routines.
- **`FW1_data.c`**: Data management and configuration for the FW1 ECU.

## Compilation

To compile the code, use the following GCC command:

```bash
gcc -o mycode /home/debian/FW1_ert_rtw/ert_main.c /home/debian/FW1_ert_rtw/FW1.c /home/debian/FW1_ert_rtw/linuxinitialize.c /home/debian/FW1_ert_rtw/FW1_data.c -I/home/debian/FW1_ert_rtw -lm -lpthread
```
This command will compile the necessary source files into an executable named `mycode`.

## CAN Setup

Before running the compiled code, ensure that the CAN interface is correctly set up. The setup is done using the following bash script:

```bash
bash can_setup.sh can1 1000000
```
This command configures the CAN interface (`can1`) with a baud rate of 1 Mbps.

## Execution

Once the CAN interface is set up and the code is compiled, you can execute the program using:

```bash
./mycode
```
This will run the Front Left Wheel ECU code, enabling it to communicate with other ECUs in the simulated vehicle network.
