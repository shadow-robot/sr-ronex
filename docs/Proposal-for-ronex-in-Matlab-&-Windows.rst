Intro
~~~~~

The current software for using ronex is dependent on and thus restricted
to Linux and ROS. The market for Ronex hardware can be expanded to
include users of Matlab, Labview and developers on Windows and Linux
that don't use ROS. This is a proposal for the development of a software
library with such goals :

1. Available in Windows and Linux
2. Immediately usable in Matlab and Labview
3. Usable from C, C++ and Python
4. Identify and report connected ronex devices
5. Modify any I/O state of connected devices
6. Documentation and examples for the above
7. Short development time (2-4 weeks)

Available solutions as a basis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Due to the short development time requirement an existing ethercat
library should be used as basis. Here are some options. All are free,
available in Linux and support real-time kernels:

1. `eml <http://wiki.ros.org/eml>`__. Ethercat master library in C++.
   pr2\_ethercat\_drivers that are currently used by ronex and the
   ethercat hand depend on eml. Apparently development has ceased on
   this project. license : binary only, free to use latest : 0.1.0 pros
   : tested and used by Shadow cons : needs to be ported to Windows for
   use in Matlab and Labview, uncertain future and maintenance. Without
   ROS and pr2\_ethercat it has limited features.

2. `Etherlab (IgH) <http://etherlab.org/en/components.php>`__\  license
   : source under GPL2 latest : 1.5.2 - 12.02.2013 pros : many
   additional tools including Matlab and Scilab but ... cons : Ethercat
   master itself only available in Linux

3. `Soem <http://soem.berlios.de/>`__. Ethercat master library in C.
   license : source under GPL2, free for commercial use latest : 1.3.0 -
   26.02.2013 pros : available in Windows. Used in Shadow for the
   sr-ronex-serial-db package cons : simple library in C with only basic
   functionality

Soem appears to be the best starting point mainly due to its
availability in Windows. For completeness, some commercial EtherCAT
master software packages :

`ixxat <http://www.ixxat.com/ethercat-master-stack_en.html>`__,
`acontis <http://www.acontis.com/eng/index.php>`__,
`Ackermann <http://www.ackermann-automation.de/ecatlv_en.htm>`__,
`Mathworks <http://www.mathworks.co.uk/programs/ethercat/>`__,
`Beckoff <http://www.ethercat.org/en/products/4F84049B9950437FB34749A52AB20786.htm>`__\ 

Design
~~~~~~

It is assumed that Soem will be used as basis. Soem is able to find
connected devices that use EtherCAT and read/write data packets. It
supports blocking and non-blocking I/O. In the former case no
multi-threading is required. Further details can be found
`here <http://soem.berlios.de/>`__. Additional functionality that needs
to be implemented.

**Basic Ronex wrapper for soem in C**

Required Features

1. "Ronexes" Data structure with a list of connected ronex modules,
   their serial numbers and types
2. Init function that will (re)scan for connected ronexes and
   (re)initialize the "Ronexes" data structure
3. Read and Write functions for individual I/O or ranges of I/O
4. Report function that will list the connected ronex modules to the
   user

Useful Features

5. Constant scanning for Ethercat devices to support hot plugging and
   unplugging
6. "Configuration" Data structure mapping ronex modules (and I/O maybe)
   to user defined names
7. Configure function(s) that will initialize the "Configuration" by
   user supplied info or a disk file
8. Save function to save a user supplied "Configuration"

These functions and data structures can be implemented in a single
dynamic linked library file (dll file in windows, so file in Linux).
This file will be linked with the binaries produced by compiling soem.

**Use the Basic Ronex wrapper in Matlab**

The user friendly way is `MEX
files <http://www.mathworks.co.uk/help/matlab/create-mex-files.html>`__.
A MEX file allows a function implemented in C/C++ to be called from
Matlab command line as if it was an integral Matlab function. A DLL can
be loaded directrly in MATLAB as described
`here <http://www.mathworks.co.uk/help/matlab/using-c-shared-library-functions-in-matlab-.html>`__.
That obviously easier to implement but less user-friendly for customers.
MEX files are build from C source that follows the MEX API. The free MS
Visual Studio Express with an appropriate SDK will do fine as a compiler
but a (non-free) Matlab installation is also needed. A tutorial should
be provided for using ronex from the MATLAB command line and as an extra
feature a Simulink block. There is 30-day trial version available that
can be used to make a first term version. A long term solution could be
to cooperate with a university that has a MATLAB license for future
upgrades and bug fixes. To summarize :

MEX - files

-  More development time required
-  Matlab installation required
-  Easy for the user

Directly load DLL - Less Development required - No Matlab installation
required - Less user-friendly

**Use the Basic Ronex wrapper in Labview**

Labview has its own API for creating "drivers", software that
communicates to specific hardware. A free Windows application is
`available <http://sine.ni.com/nips/cds/view/p/lang/en/nid/211922>`__
that helps in and guides through the generation of such drivers. That is
the obvious starting point. A tutorial should be provided for including
ronex blocks in a Labview control diagram.

**Use the Basic Ronex wrapper from C++ or Python** A customer may use
the Ronex wrapper to develop a custom C++ or Python application in
either Windows or Linux independently from any other framework like ROS.
All that is needed for C++ is a header file with the function
declaration and (Doxygen) comments with intended use of these functions.
In windows this can include code for dynamically loading a binary DLL.
In Linux linking is needed during building. Documentation for these
things is abundant in the Internet. Using the Ronex wrapper through
Python is similar to dynamically loading a DLL in MATLAB. Instructions
are
`here <http://docs.python.org/2/library/ctypes.html#module-ctypes>`__.
