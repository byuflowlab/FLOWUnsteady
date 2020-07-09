# Aeroacoustic Noise Analysis

FLOWUnsteady uses PSU-WOPWOP (a FW-H code) and a BPM code for calculating tonal and broadband aeroacoustic noise, respectively.
PSU-WOPWOP is not included in the FLOWUnsteady suite, but rather the user can request a binary of PSU-WOPWOP directly from the developer at Penn State University.

The example under [`examples/rotornoise/singlerotor.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FLOWUnsteady/blob/master/examples/rotornoise/singlerotor.ipynb) shows instructions and validation of the aeroacoustics solver.
Further validation of rotor-on-rotor interactions is given in the following paper by the authors: Alvarez, E. J., Schenk, A., Critchfield, T., and Ning, A., “Rotor-on-Rotor Aeroacoustic Interactions of Multirotor in Hover,” Journal of the American Helicopter Society, Jul. 2020, (in review).


![blown wing](../resources/vid/val_piv_multi16_005D_99_1_noise1_cropped00.gif)
![blown wing](../resources/vid/cfdnoise_ningdji_multi_005D_03_20.gif)
