.. raw:: html

   <html>
   <body>

   <h2>

Description of the steps that were taken to catkinize sr-ronex-dev stack

.. raw:: html

   </h2>

   <h3>

Software used

.. raw:: html

   </h3>
   <p>
    <ul>
     <li>

Ubuntu 12.04 LTS

.. raw:: html

   </li>
     <li>

branched sr-ronex-dev from F-multiple-modules branch

.. raw:: html

   </li>
     <li>

ros-hydro-desktop-full

.. raw:: html

   </li>
     <li>

ros-hydro-pr2-controller-interface

.. raw:: html

   </li>
     <li>

ros-hydro-pr2-ethercat-drivers

.. raw:: html

   </li>
     <li>

ros-hydro-pr2-hardware-interface

.. raw:: html

   </li>
     <li>

ros-hydro-pr2-common

.. raw:: html

   </li>
   </ul>
   </p>

   <h3>

Steps taken

.. raw:: html

   </h3>
   <p>
    <ul>
     <li>

Create a workspace folder to contain all packages of a stack and create
a src folder there e.g. sr\_ronex\_ws/src

.. raw:: html

   </li>
     <li>

cd to the main folder (sr\_ronex\_ws) and run catkin\_make. This will
create some subfolders and files for cmake

.. raw:: html

   </li>
     <li>

cd to src subfolder, git clone any stack you want to catkinize and run
catkinize\_stack e.g git clone
https://github.com/shadow-robot/sr-ronex-dev.git sr\_ronex
catkinize\_stack sr\_ronex 1.0.0

.. raw:: html

   </li>
     <li>

catkinize\_stack will backup old CMakeLists.txt and manifest.xml and
create new CMakeLists.txt files and package.xml

.. raw:: html

   </li>
     <li>

catkinize\_stack will create a subfolder with the same name as the
stack's folder e.g. /sr\_ronex/sr\_ronex and put there a package.xml
that replaces stack.xml. The latter is backed-up on its previous
location in order to follow the filesystem conventions for catkin move
contents of sr\_ronex\_ws/sr\_ronex to sr\_ronex\_ws/src and delete
sr\_ronex\_ws/sr\_ronex

.. raw:: html

   </li>
    </ul>
   </p>

   <h3>

CMakeLists.txt

.. raw:: html

   </h3>
   <p>
    <ul>
     <li>

since sr\_common\_msg contains messages edit its CMakeLists.txt like so

.. raw:: html

   <ol>
      <li>

add message\_generation in find\_package : find\_package(catkin REQUIRED
COMPONENTS message\_generation std\_msgs sensor\_msgs)

.. raw:: html

   </li>
      <li>

modify add\_message\_files like so : add\_message\_files(FILES
BoolArray.msg GeneralIOState.msg PWM.msg)

.. raw:: html

   </li>
      <li>

modify catkin\_package like so : catkin\_package(CATKIN\_DEPENDS
message\_runtime std\_msgs sensor\_msgs)

.. raw:: html

   </li>
      <li>

modify generate\_messages like so : generate\_messages(DEPENDENCIES
std\_msgs sensor\_msgs)

.. raw:: html

   </li>
     </ol>

.. raw:: html

   </li>
     <li>

sr\_ronex\_ethercat\_drivers has a cfg file so the following line was
required generate\_dynamic\_reconfigure\_options(cfg/GeneralIO.cfg) also
the following changes were required at GeneralIO.cfg file Remove import
roslib;roslib.load\_manifest(PACKAGE)" Change from
dynamic\_reconfigure.parameter\_generator import * to from
dynamic\_reconfigure.parameter\_generator\_catkin import *\ 

.. raw:: html

   </li>
     <li>

sr\_ronex\_ethercat\_drivers also contains tests that use rostest and
depend on gtest. These lines are needed for any package that contains
tests. add\_rostest\_gtest(test\_ethercat\_drivers
test/test\_ethercat\_drivers.test test/test\_ethercat\_drivers.cpp)
target\_link\_libraries(test\_ethercat\_drivers
:math:`{PROJECT_NAME} `\ {GTEST\_LIBRARIES})

.. raw:: html

   </li>
     <li>

for projects using sr\_common\_msgs, include the dependency to
auto-generated component sr\_common\_msgs\_gencpp
add\_dependencies(sr\_ronex\_controllers sr\_common\_msgs\_gencpp)
add\_dependencies(sr\_ronex\_ethercat\_drivers sr\_common\_msgs\_gencpp)

.. raw:: html

   </li>
     <li>

make sure that includes and libs include the catkin ones for appropriate
targets include\_directories(include
:math:`{catkin_INCLUDE_DIRS} )</code>     <code>target_link_libraries(sr_ronex_ethercat_drivers `\ {catkin\_LIBRARIES})

.. raw:: html

   </ul>
   </p>

   <h3>

package.xml

.. raw:: html

   </h3>
   <p>
    <ul>
     <li>

in all package.xml files modify maintainer field to put email address in
quotation marks : <maintainer email="software@shadowrobot.com">Toni
Oliver</maintainer>

.. raw:: html

   </li>
     <li>

for any package that uses tests add this line
<test\_depend>gtest</test\_depend>
<build\_depend>rostest</build\_depend>

.. raw:: html

   </li>
     <li>

for any package that uses dynamic\_reconfigure add these lines
<build\_depend>dynamic\_reconfigure</build\_depend>
<run\_depend>dynamic\_reconfigure</run\_depend>

.. raw:: html

   </li>
    </ul>
   </p>

   <h3>

References

.. raw:: html

   </h3>
   <p>
    

Catkin conceptual overview Description of catkin Differences between
catkin and rosbuild Migrating from rosbuild

.. raw:: html

   </p>

   <html>
   <body>


