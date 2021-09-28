dataguzzler-python driver for the Microsoft Azure Kinect
("Kinect for Azure"/K4A) depth camera.

You probably have give the setup script a parameter to tell it where
to find the Azure Kinect SDK files. e.g.

python setup.py build --with-azurekinect=/usr/local/src/k4a-1.4.1_ubuntu1804
then (as root if needed)
python setup.py install

It is helpful to put these commands into a script, e.g. setupcmd.sh:

#!/bin/bash
python setup.py build --with-azurekinect=/usr/local/src/k4a-1.4.1_ubuntu1804 && 
sudo python setup.py install

