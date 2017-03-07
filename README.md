# CubePuzzle

![](http://iot.do/wp-content/uploads/sites/2/2015/12/77847.jpeg)

Images recognition system for Baxter robot. 
Also Baxter now can play with cube puzzle!

How setup and start Baxter(Short Manual)

**Baxter:**

* Connect keyboard

* Press button on back Baxter panel(ON)

* Press Alt+F while Baxter loading

* Select Applications -> RSDK GUI

**OS**

* Enable root mode: sudo su

* Enable daemon: avahi-daemon

* Go to "baxter" folder(cd baxter)

* Setup Baxter: ./baxter.sh

* rosrun -baxter_tools enable_robot.py -e

* Executing own scripts python %SCRIPT_NAME%.py

_P.S. Scripts must be located in PYTHONPATH folder_

