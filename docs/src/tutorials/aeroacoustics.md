# Aeroacoustics

FLOWUnsteady uses a FW-H code (PSU-WOPWOP) and a BPM code for calculating tonal and broadband aeroacoustic noise, respectively.
PSU-WOPWOP is not included in the FLOWUnsteady suite, but the user can request a binary of PSU-WOPWOP directly from the developers at Penn State University.

The example under [`examples/rotornoise/singlerotor.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FLOWUnsteady/blob/master/examples/rotornoise/singlerotor.ipynb) shows validation of the aeroacoustics solver and instructions on how to run a rotor noise analysis.
Further validation of rotor-on-rotor interactions is given in the following paper by the authors:

* Alvarez, E. J., Schenk, A., Critchfield, T., and Ning, A., “Rotor-on-Rotor Aeroacoustic Interactions of Multirotor in Hover,” Journal of the American Helicopter Society, Jul. 2020, (in review). [[PDF](https://scholarsarchive.byu.edu/facpub/4053/)]


```@raw html
<img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/val_piv_multi16_005D_99_1_noise1_cropped00.gif" alt="Vid" width="400px"/>
```
```@raw html
<img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/cfdnoise_ningdji_multi_005D_03_20.gif" alt="Vid" width="600px"/>
```
