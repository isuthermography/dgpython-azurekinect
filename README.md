dgpython-azurekinect
--------------------

This is a Dataguzzler-Python/SpatialNDE2 driver for the Microsoft
Azure Kinect ("Kinect for Azure"/K4A) depth camera.



Installation
------------

This package requires the Azure Kinect SDK from Microsoft (uses the C
API), Dataguzzler-Python, Cython, and SpatialNDE2.

The dgpython-azurekinect package installs with the usual "python
setup.py install", but you probably need to tell it where to find the
Azure Kinect SDK files (see below). Make sure that you install into
the same Python environment you are using for SpatialNDE2 and
Dataguzzler-Python

To tell the setup script where to find the Azure Kinect SDK files,
use the --with-azurekinect parameter to the "build" and "install" steps,
e.g.

python setup.py build --with-azurekinect=/usr/local/src/k4a-1.4.1_ubuntu1804
then (as root if needed)
python setup.py install --with-azurekinect=/usr/local/src/k4a-1.4.1_ubuntu1804

It is helpful to put these commands into a script, e.g. setupcmd.sh (Linux):

#!/bin/bash
python setup.py build --with-azurekinect=/usr/local/src/k4a-1.4.1_ubuntu1804 && sudo python setup.py install --with-azurekinect=/usr/local/src/k4a-1.4.1_ubuntu1804

or setupcmd.bat (Windows):
python setup.py build --with-azurekinect="c:\Program Files\Azure Kinect SDK v1.4.1"
python setup.py install --with-azurekinect="c:\Program Files\Azure Kinect SDK v1.4.1"


NOTE: If you upgrade spatialnde2 it is highly recommended that, after
rebuilding and performing the Python reinstall ("python setup.py
install" from the spatialnde2 build directory) that you clean out your
dgpython-azurekinect build/ and dist/ directories and do a full
reinstall, e.g. from the dgpython-azurekinect directory:

      WINDOWS:
            rmdir /s build dest
            setupcmd.bat
      LINUX:
            rm -r build/ dest/
            bash setupcmd.sh

Usage
-----

This package is designed to be used from Dataguzzler-Python. In your
Dataguzzler-Python configuration (.dgp file):


  import dgpython_azurekinect as ak
  k4a = ak.K4A("k4a",recdb, None, "/k4achan" )
  k4a.running=True # Start capture

The above configuration will start live capture into the
"/k4achan" channel of the SpatialNDE2 recording database. 

The parameters to the K4A constructor are:
  1. The module name
  2. The SpatialNDE2 recording database
  3. The serial number of the camera to open, or None to open the
     only camera available
  4. Device recording database depth channel name
  5. Device recording database color channel name (optional)

There is an alternative class, K4AFile, that extracts frames from
a .mkv file saved by the k4arecorder application. The parameters
are the same except the device serial number is replaced by the
filename for the .mkv file.


Configuring the Azure Kinect camera
-----------------------------------

Configuration is controlled and queried through a number of
Python properties of the K4A object. These are assigned
similar to "k4a.running=True" in the example above.

Many of the properties currently use values enumerated
in the ak.syms dictionary that match the low level numbers
used in the Azure Kinect C API. A future version may convert
the numeric properties to string properties. See the Azure
Kinect API manual for more information. 

Property         Function         Type     Possible values
--------------------------------------------------------------
running          Enable capture   boolean  True, False
color_format     Color cam. fmt.  integer  ak.syms.K4A_IMAGE_FORMAT_COLOR_...
color_resolution Color cam. res.  integer  ak.syms.K4A_COLOR_RESOLUTION_...
depth_mode       Depth cam. res.  integer  ak.syms.K4A_DEPTH_MODE_...
camera_fps       Frames per sec.  integer  ak.syms.K4A_FRAMES_PER_SECOND_...
synchronized_images_only          boolean  True, False
depth_delay_off_color_usec        integer  Magn. less than one capt. period
wired_sync_mode  Multi-camera     integer  ak.syms.K4A_WIRED_SYNC_MODE_...
subordinate_delay_off_master_usec integer  Up to one capture period
disable_streaming_indicator       boolean  True, False
point_cloud_frame                 integer  K4A_CALIBRATION_TYPE_...
depth_data_mode  Depth cam. acq.  string   "IMAGE" or "POINTCLOUD"
depth_data_type  Depth cam. acq.  string   "INT" or "FLOAT"
calcsync         Wait for math    boolean  True, False

Dynamic Metadata
----------------
This driver supports dynamic metadata on live acquisitions.
Call the .dynamic_metadata.AddStaticMetaDatum() and/or
.dynamic_metadata.AddDynamicMetaDatum() methods, e.g.

  k4a.dynamic_metadata.AddStaticMetaDatum("/k4achan","testmd","testmd_value")
  k4a.dynamic_metadata.AddDynamicMetaDatum("/k4achan","testmd2",lambda: k4a.depth_mode)


Examples
--------
There is an example dataguzzler-python configuration for live capture,
demos/acquire_azurekinect.dgp. Run it with:
  dataguzzler-python acquire_azurekinect.dgp

There is an example dataguzzler-python configuration for playing back
sequences recorded using the k4arecorder application,
demos/playback_azurekinect.dgp. Run it with:
  dataguzzler-python playback_azurekinect.dgp --filename <file.mkv>






