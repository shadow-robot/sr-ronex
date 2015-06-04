.. raw:: html

   <html>
   <body>

   <h3>

Build sr-ronex with catkin

.. raw:: html

   </h3>
   <p>
    <ol>
     <li>

make a workspace directorymkdir -p sr\_ronex\_ws/src

.. raw:: html

   <li>

to get sr-ronex git clone in an src sub-folder and checkout development
branch e.g. cd sr\_ronex\_ws/src git clone
https://github.com/shadow-robot/sr-ronex-dev.git

.. raw:: html

   <li>

download and install dependencies sudo apt-get install
ros-hydro-desktop-full ros-hydro-pr2-controller-interface
ros-hydro-pr2-ethercat-drivers ros-hydro-pr2-hardware-interface
ros-hydro-pr2-common ros-hydro-pr2\_ethercat

.. raw:: html

   <li>

switch to the workspace directory sr\_ronex\_ws and for a debug build
type cd .. catkin\_make -DCMAKE\_BUILD\_TYPE=Debug

.. raw:: html

   </li>
    </ol>
   </p>

   <h3>

Run Tests

.. raw:: html

   </h3>
   <p>
    <ol>
     <li>

switch to the build sub-folder of the workspace folder and ran make
tests e.g. cd sr\_ronex\_ws/build make tests

.. raw:: html

   </li>
     <li>

then ran make test to actually run the tests make test

.. raw:: html

   </li>
     <li>

alternatively, the above commands are aggregated in the following make
run\_tests

.. raw:: html

   </li>
    </ol>
   <p>

   </html>
   </body>

