# nrf52_wheelLine_accel
C code for nRF52 MCU to use a LIS2DH12 accelerometer to move a wheel line.

# IDE setup backstory
Collin used to use Eclipse CDT with GnuArmEclipse plugin, then used to download GNU MCU Eclipse https://github.com/gnu-mcu-eclipse
That has since been deprecated, and the requirements narrowed down to (for a Windows system): 
- Eclipse IDE to edit the code and generate a "build" (creates a makefile, runs it to build).
- GNU ARM Embedded Toolchain (to build the code for ARM MCUs, which the nRF52 is one of).
- make.exe

## Eclipse install
We want to install the Ecipse IDE CDT, which has support for C code development. Most any version from 2019 on should work. Currently, the page here https://github.com/eclipse-cdt/cdt directs to here to install Eclipse https://eclipseide.org/
  Download link will download an installer that can install CDT Eclipse:
  https://www.eclipse.org/downloads/download.php?file=/oomph/epp/2023-12/R/eclipse-inst-jre-win64.exe
  Choose Eclipse IDE for Embedded C/C++ Developers.
  
  ![image](https://github.com/colzin/nrf52_wheelLine_accel/assets/10608175/a700f86e-8584-43ce-a7d8-a4f35c8a14b8)

## Toolchain install
Next, download and install the GNU ARM Embedded Toolchain https://developer.arm.com/downloads/-/gnu-rm
Collin downloaded the [10.3-2021.10 version for Windows 10](https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-win32.exe?rev=29bb46cfa0434fbda93abb33c1d480e6&hash=3C58D05EA5D32EF127B9E4D13B3244D26188713C)
Remember the path to which it was installed, for later. You do not need to add PATH to environment variable. Collin recommends manually setting the project's path.
Collin had C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10

## Make install
Finally, we need to install make.exe, which is used by the build process. [GnuWin32 will install make.exe for you](https://gnuwin32.sourceforge.net/packages/make.htm), [Download link](https://gnuwin32.sourceforge.net/downlinks/make.php)
Then just remember the path for later.
Collin had C:\Program Files (x86)\GnuWin32

## Nrf Command Line Tools install
To program and erase Nordic nRF52 MCUs, install the [nRF Command Line Tools](https://www.nordicsemi.com/Products/Development-tools/nrf-command-line-tools/download)
Collin downloaded [10.23.4 for Windows x64](https://nsscprodmedia.blob.core.windows.net/prod/software-and-other-downloads/desktop-software/nrf-command-line-tools/sw/versions-10-x-x/10-23-4/nrf-command-line-tools-10.23.4-x64.exe)
Run the installer, which will also install SEGGER tools, which are used to program and debug Nordic MCUs.
If you use a Nordic dev board, you don't have to buy a SEGGER pod.


# Project setup in Eclipse IDE
## Download the code
After installing Eclipse IDE, GNU ARM Embedded Toolchain, and make.exe, you can download this repository anywhere on your computer (shorter paths are recommended, eg D:\projects\wheelLinenRF52).
## Open up Eclipse
Next, open Eclipse. It will ask you to create an Eclipse Workspace. This can be anywhere on your computer, and contains a list of projects that you have open in Eclipse. It is **not the same** as the code project location. The default path should be fine, or you can append _fornrf52 to the workspace name to differentiate it, if you have multiple Eclipse instances.
Next, click Import Existing Projects on the Welcome page, or File>Import, then choose General>Existing Projects

![image](https://github.com/colzin/nrf52_wheelLine_accel/assets/10608175/31f249cc-6313-419c-bc81-abcc2f6b4ea2)
Browse to the folder where you checked out this repo, and select both the DFU and the FW.

![image](https://github.com/colzin/nrf52_wheelLine_accel/assets/10608175/8a1bd23a-0b89-4268-9ce6-6c15927428d1)
Next, set up the PATH for each project by right-clicking in Project Explorer, click Properties.
Select C/C++ Build > Environment. In there, double-click the PATH entry, to bring up a dialog box. Enter the paths to GNU Arm Embedded tools, and make.exe, separated by ;
Collin had the following:
C:\Program Files (x86)\GnuWin32\bin;C:\Program Files\Nordic Semiconductor\nrf-command-line-tools\bin
Set the path for both Debug and Release builds of each project that you plan to build.
By manually setting the PATH, instead of using the default Windows path, you can be sure of which tool set you are using to build the project.



