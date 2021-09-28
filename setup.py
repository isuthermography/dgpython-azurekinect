import os
import os.path
import numpy as np
from setuptools import setup,Extension
from distutils.command.build import build

from Cython.Build import cythonize
import dataguzzler_python


ext_modules=cythonize(Extension("dgpython_azurekinect.kinect",
                                sources=["dgpython_azurekinect/kinect.pyx"],
                                libraries=["k4a"]),
                      language_level=3)


class BuildCommand(build):
    user_options = build.user_options + [
        ('with-azurekinect=',None,'Path to Azure Kinect SDK')
    ]
    def initialize_options(self):
        build.initialize_options(self)
        self.with_azurekinect = None
        pass

    def finalize_options(self):
        if self.with_azurekinect is not None:
            #for ext in ext_modules:
            build_ext_cmd = self.distribution.get_command_obj("build_ext")
            if not hasattr(build_ext_cmd,"include_dirs") or build_ext_cmd.include_dirs is None:
                build_ext_cmd.include_dirs=[]
                pass
            build_ext_cmd.include_dirs.append(os.path.join(self.with_azurekinect,'include'))
            if not hasattr(build_ext_cmd,"library_dirs") or build_ext_cmd.library_dirs is None:
                build_ext_cmd.library_dirs=[]
                pass

            build_ext_cmd.library_dirs.append(os.path.join(self.with_azurekinect,'lib'))

            if not hasattr(build_ext_cmd,"rpath") or build_ext_cmd.rpath is None:
                build_ext_cmd.rpath=[]
                pass

            build_ext_cmd.rpath.insert(0,os.path.join(self.with_azurekinect,'lib'))            
            build_ext_cmd.rpath.insert(0,"$ORIGIN")

            
            pass
        build.finalize_options(self)
        pass

    def run(self):
        #print("with_azurekinect=%s" % (self.with_azurekinect))
        #print("ext[0].include_dirs=%s" % (str(ext_modules[0].include_dirs)))
        build.run(self)
        pass
    pass

setup(name="dgpython_azurekinect",
      description="Azure Kinect module for dgpython",
      author="Stephen D. Holland",
      url="http://thermal.cnde.iastate.edu",
      ext_modules=ext_modules,
      zip_safe=False,
      packages=["dgpython_azurekinect"],
      cmdclass = { 'build': BuildCommand })
