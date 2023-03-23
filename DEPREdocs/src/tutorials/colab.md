# FLOWUnsteady in Colab

Colab is a Jupyter notebook environment that requires no setup to use and runs entirely in the cloud.
Here we show how you can set up FLOWUnsteady in your Google Drive and have Google Colaboratory do all the computation for you, without having to install anything on your own computer! (and for free!)

## Install Colab

First, install the Colaboratory app in your G Suite from the Marketplace: [gsuite.google.com/marketplace/app/colaboratory/1014160490159](https://gsuite.google.com/marketplace/app/colaboratory/1014160490159). After it has been installed, you will see the option to open files with Google Colaboratory every time you double click a file with the extension `.ipynb` in your Google Drive.

## FLOWUnsteady Folder

Now, we will set up FLOWUnsteady in your Google Drive.
Start by create a new folder named `FLOWUnsteady` in your drive, resulting in the path `My Drive/FLOWUnsteady/`.
Download [`dotjulia.tar.gz`](https://bit.ly/dotjulia-tar-gz) and upload it (without decompressing) into `My Drive/FLOWUnsteady/`.
This file is an image of all FLOW packages and other Julia dependencies needed to run FLOWUnsteady.

## Simple Example

Now let's test FLOWUnsteady+Colab with a simple example.
In this example we will use the FLOW package [AirfoilPrep](https://github.com/byuflowlab/AirfoilPrep.jl) to run XFOIL on a NACA 0012.
Then we will run the tandem heaving wing example of FLOWUnsteady with the quasi-steady solver (hence, you don't need the VPM solver).
The simulation looks like this:

[![Vid here](../assets/img/play01_wide.png)](https://youtu.be/Pch94bKpjrQ)

To run this example, download the following files and upload them to `My Drive/FLOWUnsteady/`. Then open `FLOWUnsteady-example.ipynb` in your Google Drive with Google Colaboratory and follow the instructions in the notebook.

| File | URL | Description|
|:-:|:-:|:-:|
|[`FLOWUnsteady-example.ipynb`](https://github.com/byuflowlab/FLOWUnsteady/blob/master/docs/resources/colab/FLOWUnsteady-example.ipynb)|[LINK](https://raw.githubusercontent.com/byuflowlab/FLOWUnsteady/master/docs/resources/colab/FLOWUnsteady-example.ipynb)| Jupyter notebook |
|[`naca0012.csv`](https://github.com/byuflowlab/FLOWUnsteady/blob/master/docs/resources/colab/naca0012.csv)|[LINK](https://raw.githubusercontent.com/byuflowlab/FLOWUnsteady/master/docs/resources/colab/naca0012.csv)| NACA 0012 airfoil geometry |

**NOTE:** You may run into a webpage 400 error when opening Jupyter notebooks if you are logged into a corporate Google account with limited functionality. To solve this issue, open Google Colab by right-clicking anywhere in the background of your Google Drive (`right click`>`More`>`Google Colaboratory`) then upload the notebook (`File`>`Upload notebook`).

## Rotor Aeroacoustics Example

Finally, let's run a rotor simulation solving the aerodynamics and noise of a 10in-diameter DJI 9443 rotor.
Here we show how to generate the aerodynamic solution, run PSU-WOPWOP to get tonal noise, and use BPM to get broadband noise. We then compare the results to experimental acoustic data from the literature.

This is the aeroacoustic output by FLOWUnsteady:

```@raw html
<img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/dji9443_ccblade01_1.gif" alt="Vid here" width="700px">
```

To run this example, download the following files and upload them to `My Drive/FLOWUnsteady/`.

| File | URL | Description|
|:-:|:-:|:-:|
|[`FLOWUnsteady-rotor-noise.ipynb`](https://github.com/byuflowlab/FLOWUnsteady/blob/master/docs/resources/colab/FLOWUnsteady-rotor-noise.ipynb)|[LINK](https://raw.githubusercontent.com/byuflowlab/FLOWUnsteady/master/docs/resources/colab/FLOWUnsteady-rotor-noise.ipynb)| Jupyter notebook |
|`wopwop3_linux_serial`| Contact the developers$^{[1]}$ | PSU-WOPWOP binary, v3.4.2 or v3.4.3|
|`zawodny-data.zip`|[LINK](https://github.com/byuflowlab/FLOWUnsteady/raw/master/docs/resources/colab/zawodny-data.zip)| Experimental data for comparison. Extract this zip. |

**NOTE 1:**  PSU-WOPWOP is not included in the FLOWUnsteady suite, but the user can request a binary of PSU-WOPWOP directly from its developers at Penn State University.
