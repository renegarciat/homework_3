# HW3 Lab Code

---

Follow this link for the video instruction on running the simulator: [https://youtu.be/hEzgVZ7KFcQ?si=ErHykIkAZtZQvN6w](https://youtu.be/hEzgVZ7KFcQ?si=ErHykIkAZtZQvN6w)

---

This is the code that we use in the lab for HW3.
There are three empty files that you need to fill in the folder `OwnCode`:
1. Controller.c
2. OwnVariables.c
3. RenewControllerState.c


Remember to upload the three files contained in the folder `OwnCode` together with your homework report! (Make sure that you replace the C files first; do not upload the empty template files given.)


## Running the code on your computer

To run the software on an Unix computer or virtual machine, execute this commands in terminals from within this folder.

    ./CompileExecuteCFiles.sh
    ./ExecuteGUI.sh

In the simulation you should check the Activate Simulation button and enable the MoCap to get position feedback
Then you can enable control and Automatic control, with automatic control enabled you can move between start position and goal position driven by your controller.

Note: If you change your controller in OwnCode, you will need to recompile, step 3.

Data logging
6. Click Select File and find a location for the log files, then save it as NAME.csv
7. Click logging on, then Control inputs, position and serial logs should be saved in the selected location


## Structure

The structure is as follows:

* `ArduinoFiles/hybrid_control.ino`: The code for the Arduino for the students to fill in.
* `ArduinoFiles/libraries`: Arduino libraries for the robot. The have to be copied into `sketchbook/libraries` to work with the Arduino IDE.
* `CFiles`: Contains the C++ files for the simulation. Students put their own code the three files in `CFiles/OwnCode`
* `Python files`: This contains the controller GUI files. This is both to control the simulation was well was the actual robot. The entry point in `PythonFiles/gui.py`. The core program sits in `PythonFiles/ui/mainwindow.py` The actual GUI code is created with QT creator and is contained in the file `PythonFiles/ui/mainwindow.ui` from which `PythonFiles/ui/UI_mainwindow.py` gets generated.



## Running the robot in the lab

Do not be scared by these instructions.
The TAs will help you to set up your code in the lab.
Just make sure that your code works well on the simulator.

### Software needed

* `python 2.7`
* `pyqt4`
* arduino IDE
* Nexus driver in `/home/sketchbook/libraries`

### Arduino

* Board: Ardiuno Duemilanove w/ATmega 328
* Select Tools -> serial port once arduino is connected
* Remove Jumper to flash the board
* Don't select SS usb ports
* Make sure the headers are the unix way (all headers with .h)


## Control application

1. `cd ./Desktop/hybrid\ lab\ var/control\ application/`
2. `python gui.py`
3. check serial port: `dmesg`

For debugging: in *arduino ide*, select *right port*, *tools*, *serial monitor*.



## Info on the lab MoCap system

### To run

1. Open track manager
2. Open new file
3. Body number is the list position on the body list (settings)

### To create new body

1. record file
2. shift+drag to select makers
3. right click --> create new body
4. name the body
5. close the file and open a new one
