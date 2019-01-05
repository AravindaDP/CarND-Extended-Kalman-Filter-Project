## Visual Studio IDE profile

[//]: # (Image References)
[copy_to_ports]: ./images/copy_to_ports.png
[uws_include]: ./images/uws_include.png
[startup]: ./images/startup.png
[debug]: ./images/debug.png

### vcpkg installation

Clone vcpkg from https://github.com/Microsoft/vcpkg.git into c:\vcpkg (or any folder of your choice)

```
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
```

Run bootstrap

```
.\bootstrap-vcpkg.bat
```

Open command prompt as **administrator** and run:

```
.\vcpkg integrate install
```

### Install uWebSockets commit e94b6e1

Since udacity's project are not using the latest master branch of uWebSockets, we need to install the same commit as is done for Ubuntu/Mac. I've already compiled the particular commit and have included the library/include files in [carnd-term2-libs-ports.zip](https://raw.githubusercontent.com/drganjoo/term2-setup/master/carnd-term2-libs-ports.zip). Just unzip and copy the 'carnd-term2-libs' folder from the zip file to ports folder inside vcpkg folder.

![copy_to_ports]

From vcpkg directory, Install carnd-term2-libs using:

```
.\vcpkg install carnd-term2-libs --triplet x86-windows
.\vcpkg install carnd-term2-libs --triplet x64-windows
```

Make sure you have a uWS folder in \vcpkg\installed\x86-windows\include and \vcpkg\installed\x64-windows\include:

![uws_include]


### How to build using Visual Studio Solution?

Open `ExtendedKF.sln` in the directory containing this README. After that you can press F6 to build the solution.

### How to debug using Visual Studio profile?

Mark ExtendedKF as the startup project (Right click ExtendedKF in the solution explorer and choose "Set as Startup Project").

![startup]

Press F5 (or menu option Debug->Start Debugging)

You can put a breakpoint on line # 69 of main.cpp and re-run the program (F5) and re-run the simulator.

The program should stop when it gets data from the simulator and you should be able to inspect values in the debugger local / auto window.

![debug]

