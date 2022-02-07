dataguzzler-python driver for the Microsoft Azure Kinect
("Kinect for Azure"/K4A) depth camera.

You probably have give the setup script a parameter to tell it where
to find the Azure Kinect SDK files. e.g.

python setup.py build --with-azurekinect=/usr/local/src/k4a-1.4.1_ubuntu1804
then (as root if needed)
python setup.py install --with-azurekinect=/usr/local/src/k4a-1.4.1_ubuntu1804

It is helpful to put these commands into a script, e.g. setupcmd.sh:

#!/bin/bash
python setup.py build --with-azurekinect=/usr/local/src/k4a-1.4.1_ubuntu1804 && sudo python setup.py install --with-azurekinect=/usr/local/src/k4a-1.4.1_ubuntu1804

or setupcmd.bat:
python setup.py build --with-azurekinect="c:\Program Files\Azure Kinect SDK v1.4.1"
python setup.py install --with-azurekinect="c:\Program Files\Azure Kinect SDK v1.4.1"


NOTE: If you upgrade spatialnde2 it is highly recommended that, after rebuilding and performing the 
Python reinstall ("python setup.py install" from the spatialnde2 build directory) that you 
clean out your dgpython-azurekinect build/ and dist/ directories and do a full reinstall, 
e.g. from the dgpython-azurekinect directory:
	  WINDOWS:
	    rmdir /s build dest
	    setupcmd.bat
      LINUX:
	    rm -r build/ dest/
	    bash setupcmd.sh
		