# [Windows Instructions](@id windows)

*(Tested on Windows 11)*


## Install Windows Subsystem for Linux (WSL)
* Follow this guide to setup Ubuntu 22.04: [LINK](https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-11-with-gui-support)
* If you run into issues, try the following:
    * Enable bash for Windows [LINK](https://www.groovypost.com/howto/install-and-start-bash-in-windows-10-anniversary-update)
    * Enable virtualization [LINK](https://learn.microsoft.com/en-us/windows/wsl/troubleshooting#error-0x80370102-the-virtual-machine-could-not-be-started-because-a-required-feature-is-not-installed). For instance, here are instructions specific for a Dell Precision 7760: [LINK1](https://www.dell.com/support/kbdoc/en-us/000195978/how-to-enable-or-disable-hardware-virtualization-on-dell-systems) [LINK2](https://www.dell.com/support/kbdoc/en-us/000195980/how-to-enable-or-disable-windows-virtualization-on-dell-systems) (make sure to check "Hyper-V")

To launch WSL, we recommend launching it from the Microsoft Store as opposed to
the Programs Menu or the terminal, otherwise Windows' PATH will not be passed
onto WSL.

## Set up Environment
Once inside WSL, do the following

* Install system-level packages:
  ```bash
  sudo apt-get update
  sudo apt-get install cmake g++ mpich
  ```
* Install python packages:
  ```bash
  sudo apt-get install python3-pip python3-tk
  pip3 install matplotlib mpmath scipy --user
  ```
* Create a folder where to install programs
  ```bash
  mkdir ~/Programs
  cd ~/Programs
  ```

## Install Julia
Still inside WSL,

* Download Julia:
  ```bash
  wget -O julia.tar.gz "https://julialang-s3.julialang.org/bin/linux/x64/1.10/julia-1.10.7-linux-x86_64.tar.gz"
  ```
* Decompress Julia:
  ```bash
  tar -xvf julia.tar.gz -C ~/Programs/
  ```
* Add Julia to user-level path:
  ```bash
  sudo ln -s ~/Programs/julia-1.10.7/bin/julia /usr/local/bin/
  ```

## Install ParaView
* Donwload and install [ParaView v5.9.1](https://www.paraview.org/paraview-downloads/download.php?submit=Download&version=v5.9&type=binary&os=Windows&downloadFile=ParaView-5.9.1-Windows-Python3.8-msvc2017-64bit.exe)
* Identify the location of `paraview.exe` (most likely, this is `C:\Program Files\ParaView 5.9.1\bin\`)
* Add the path of ParaView (*e.g.*, `C:\Program Files\ParaView 5.9.1\bin\`) to the system-level PATH: [LINK](https://www.computerhope.com/issues/ch000549.htm)
* Create a `paraview` alias inside WSL typing this in the WSL terminal:
  ```bash
  which paraview.exe | xargs -I{} sudo ln -s {} /usr/local/bin/paraview
  ```

!!! info "ParaView 5.10+ on Windows"
    ParaView versions 5.10 and newer on Windows cannot open XDMF files (like
    the particle field) from the WSL file system. Please copy them to a Windows
    file system or use Paraview 5.9.1.


---

**Now you can proceed with [the general instructions](@ref installation)**
(you can skip the Julia and ParaView instructions since we already took care of that)
