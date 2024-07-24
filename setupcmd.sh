#!/bin/bash
#python setup.py build --with-azurekinect=/usr/local/src/k4a-1.4.1_ubuntu1804 && sudo python setup.py install
export DIST_EXTRA_CONFIG=setup_local.cfg
pip install --no-deps --no-build-isolation .

