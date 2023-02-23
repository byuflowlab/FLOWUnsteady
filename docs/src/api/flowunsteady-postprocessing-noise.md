# Aeroacoustic Noise

FLOWUnsteady uses an FW-H code and [a BPM code](https://github.com/byuflowlab/BPM.jl) for calculating tonal and broadband aeroacoustic noise, respectively.
The tonal code (PSU-WOPWOP) is not included in the FLOWUnsteady suite, but the user can request a binary of PSU-WOPWOP directly from the developers at Penn State University.

The example under [`examples/rotornoise/singlerotor.ipynb`](https://nbviewer.jupyter.org/github/byuflowlab/FLOWUnsteady/blob/master/examples/rotornoise/singlerotor.ipynb) shows validation of the aeroacoustics solver and instructions on how to run a rotor noise analysis.
Further validation of rotor-on-rotor interactions is given in the following paper by the authors:

* Alvarez, E. J., Schenk, A., Critchfield, T., and Ning, A., “Rotor-on-Rotor Aeroacoustic Interactions of Multirotor in Hover,” The Vertical Flight Society’s 76th Annual Forum, 2020, Jul. 2020. [\[SLIDES\]](http://edoalvar2.groups.et.byu.net/public/AlvarezSchenkCritchfield_2020-PresentationVFSForum-multirotor_noise_interactions_in_hoverSTATIC.pdf)[\[PDF\]](https://scholarsarchive.byu.edu/facpub/4053/)


```@raw html
<img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/dji9443_ccblade01_1.gif" alt="Vid" width="75%"/>
```
```@raw html
<img src="http://edoalvar2.groups.et.byu.net/public/FLOWUnsteady/cfdnoise_ningdji_multi_005D_03_20.gif" alt="Vid" width="75%"/>
```
```@raw html
<div style="position:relative;padding-top:60%;">
  <iframe style="position:absolute;left:0;top:0;height:100%;width:100%;" src="https://www.youtube.com/embed/ntQjP6KbZDk?hd=1" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
</div>
```

```@docs
FLOWUnsteady.run_noise_wopwop
FLOWUnsteady.run_noise_bpm
```
