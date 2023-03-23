# [Windows Instructions](@id windows)

*(Tested on Windows 11)*


## Install Windows Subsystem for Linux (WSL)
* Follow this guide to setup Ubuntu 22.04: [LINK](https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-11-with-gui-support)
* If you run into issues, try some of the following things:
    * Enable bash for Windows [[LINK](https://www.groovypost.com/howto/install-and-start-bash-in-windows-10-anniversary-update)]
    * Enable virtualization [[LINK](https://learn.microsoft.com/en-us/windows/wsl/troubleshooting#error-0x80370102-the-virtual-machine-could-not-be-started-because-a-required-feature-is-not-installed)]. For instance, here are instructions specific for a Dell Precision 7760: [LINK1](https://www.dell.com/support/kbdoc/en-us/000195978/how-to-enable-or-disable-hardware-virtualization-on-dell-systems) [LINK2](https://www.dell.com/support/kbdoc/en-us/000195980/how-to-enable-or-disable-windows-virtualization-on-dell-systems) (also check Hyper-V)

## Set up Environment
Once inside WSL, do the following steps

* Install system-level packages:
  ```bash
  sudo apt-get update
  sudo apt-get install cmake g++ mpich
  ```
```@raw html
<br>
```
* Install python packages:
  ```bash
  sudo apt-get install python3-pip python3-tk
  pip3 install matplotlib mpmath scipy --user
  ```

```@raw html
<br>
```
* Create a folder where to install programs
  ```bash
  mkdir ~/Programs
  cd ~/Programs
  ```

```@raw html
<br>
```
* Install Julia
  * Download Julia:
    ```bash
    wget -O julia.tar.gz "https://julialang-s3.julialang.org/bin/linux/x64/1.8/julia-1.8.5-linux-x86_64.tar.gz"
    ```
  * Decompress Julia:
    ```bash
    tar -xvf julia.tar.gz -C ~/Programs/
    ```
  * Add Julia to system-level path:
    ```bash
    sudo ln -s ~/Programs/julia-1.8.5/bin/julia /usr/local/bin/
    ```

```@raw html
<br>
```
## Miscellaneous
* Get the path of Python: `which python3`. This is the path you will use [setting up PyCall](@ref pycall).

* To be able to [pull up ParaView from the terminal](@ref paraview) through the command `paraview`:
    1. Donwload and install [ParaView](https://www.paraview.org/)
    2. Identify the location of `paraview.exe` (most likely, this is `C:\Program Files\ParaView 5.11.0\bin\`)
    3. Add the path of ParaView to the system-level PATH: [LINK](https://www.computerhope.com/issues/ch000549.htm)
    4. Create a `paraview` alias inside WSL: `which paraview.exe | xargs -I{} sudo ln -s {} /usr/local/bin/paraview`


---

**Now you can proceed with [the general instructions](@ref installation)** (you can skip the Julia and ParaView setup since we already took care of that)
