# FLOWUnsteady

```@contents
Pages = ["FLOWUnsteady.md"]
```

## [Creating a Custom Rotor](@id createcustomrotor)

If you do not already have the rotor files to describe your rotor, then you will need to create them. Here we will go through each file type and what to put in them. It is important to note that file name and types are not important, however they must be comma delimited.  We will use the DJI-II from the FLOWUnsteady database as an example.

It is important to note that if XFOIL is not run (marked false), then airfoil polars must be provided. The files that affect solution outcome are the chord distribution, pitch distribution, leading edge distribution, and airfoil polar files. If XFOIL is marked to run, all of the geometric material must be accurate.

#### MainFile.csv

The main file describes the overall rotor, and points to the blade file that describes one of the blades.

| property | file             | description             |
| -------- | ---------------- | ----------------------- |
| Rtip     | 0.75             | (m) Radius of blade tip |
| Rhub     | 0.0375           | (m) Radius of hub       |
| B        | 3                | Number of blades        |
| blade    | DJI-II_blade.csv | Blade file              |



#### Blade.csv

The main file points to all of the files that desribe the blade and spline parameters.

| property      | file                  | description            |
| ------------- | --------------------- | ---------------------- |
| chorddist     | DJI-II_chorddist.csv  | Chord distribution     |
| pitchdist     | DJI-II_pitchdist.csv  | Pitch distribution     |
| sweepdist     | DJI-II_sweepdist.csv  | LE sweep distribution  |
| heightdist    | DJI-II_heightdist.csv | LE height distribution |
| airfoil_files | DJI-II_airfoils.csv   | Airfoil distribution   |
| spl_k         | 1                     | Spline order           |
| spl_s         | 2.0e-7                | Spline smoothing       |



#### ChordDist.csv

This file provides the distribution of the chord along the blade, normalized by the radius of the blade. The file must go in order of root to tip. The root need not be defined, but the tip must be. The file can have any number of paired numbers. Any values used by the solver that are not provided will be interpolated.

| r/R       | c/R       |
| --------- | --------- |
| 0.0411523 | 0.121011  |
| 0.0685871 | 0.138171  |
| ...       | ...       |
| 1.0       | 0.0978361 |



#### PitchDist.csv

This file describes the twist of the blade along the blade. The file can have any number of paired numbers. Any values used by the solver that are not provided will be interpolated.

| r/R       | twist (deg) |
| --------- | ----------- |
| 0.0411523 | 16.4567     |
| 0.0685871 | 17.5        |
| ...       | ...         |
| 1.0       | 11.6        |



#### SweepDist.csv

This file describes the sweep distribution. This is also known as the leading edge distribution, it describes the distance of the the leading edge from a line coming from the center of the hub. These lines change depending on how many blades are included on a rotor (see photo below for example of a the rotor with 3 blades). The file can have any number of paired numbers. Any values used by the solver that are not provided will be interpolated.

```@raw html
<img src="../assets/howtofigs/sweepdist.png" alt="sweepdist" style="zoom:33%;" />
```

| r/R       | y/R (y-distance of LE from the middle point of hub) |
| --------- | --------------------------------------------------- |
| 0.0411523 | 0.0576211                                           |
| 0.0685871 | 0.0605955                                           |
| ...       | ...                                                 |
| 1.0       | 0.0344412                                           |



#### HeightDist.csv

This file describes the height distribution, also known as anhedral (or precode for turbines). This describes the height of the leading edge from the top face of the hub. The file can have any number of paired numbers. Any values used by the solver that are not provided will be interpolated.

```@raw html
<img src="../assets/howtofigs/precone.png" alt="precone" width="400" />
```

| r/R       | z/R (height of LE from the top face of hub) |
| --------- | ------------------------------------------- |
| 0.0686391 | -0.00242965                                 |
| 0.2       | 0.00728895                                  |
| ...       | ...                                         |
| 1.0       | -0.0242965                                  |



#### Airfoil_Files.csv

This file describes the airfoils along the blade and the paired aero file that goes with the contour file. The contour file is a file of the geometric shape of the airfoil. The aero file is the airfoil polar, the file that has the coefficient of lift, drag and moment for a given set of angles of attack for the airfoil. Note that the information is interpolated, so airfoils between two stated airfoils will be an interpolation between the two. If XFOIL is marked to run, then the aero files will not be used. As many pairs of airfoil files as desired may be used.

| r/R  | Contour file | Aero file               |
| ---- | ------------ | ----------------------- |
| 0.0  | e856-il.csv  | xf-e856-il-50000-n5.csv |
| 0.3  | e63.csv      | xf-e63-il-50000-n5.csv  |
| 1.0  | e63.csv      | xf-e63-il-50000-n5.csv  |



#### AirfoilGeoFile.csv

This file describes the geometry of the airfoil by giving x and y coordinates of the airfoil surface. These coordinates are normalized by the chord length. The order of the points should be trailing edge, upper surface, leading edge, lower surface, then trailing edge. As many coordinate pairs as desired may be used, all other points used will be interpolated.

| x/c     | y/c     |
| ------- | ------- |
| 1.0     | 0.0     |
| 0.99619 | 0.00144 |
| ...     | ...     |
| 1.0     | 0.0     |



#### AirfoilPolarFile.dat

This file contains all of the airfoils' coefficients of lift, drag and moment for a given set of angles of attack for the airfoil. Values that are required but not given will be interpolated. Note that the polar should match the general Reynolds number that the given section will experience. If XFOIL is set to run, this file will not be used. This is the only file that is not a comma delimited file.

```shell
DU21 airfoil with an aspect ratio of 17.  Original -180 to 180deg Cl, Cd, and Cm versus AOA data taken from Appendix A of DOWEC document 10046_009.pdf (numerical values obtained from Koert Lindenburg of ECN).
Cl and Cd values corrected for rotational stall delay and Cd values corrected using the Viterna method for 0 to 90deg AOA by Jason Jonkman using AirfoilPrep_v2p0.xls.
one more line
 1        	 Number of airfoil tables in this file
 1.0      	 Reynolds numbers in millions
 0.0      	 Control setting
 8.0      	 Stall angle (deg)
-5.0609      Zero lift angle of attack (deg)
 6.2047      Cn slope for zero lift (dimensionless)
 1.4144      Cn at stall value for positive angle of attack
-0.5324      Cn at stall value for negative angle of attack
-1.50        Angle of attack for minimum CD (deg)
 0.0057   Minimum CD value
-180.00    0.000   0.0185   0.0000
-175.00    0.394   0.0332   0.1978
-160.00    0.670   0.2809   0.2738
  ...				...			...				...
```



## [Rotor Database Structure](@id rotordatabasestructure)

The database can be found as a subdirectory of the FlowUnsteady package.

```shell
../FlowUnsteady/data
```



| MainFile.csv      | =>   | ../data/rotors   |
| ----------------- | ---- | ---------------- |
| Airfoils.csv      | =>   | ../data/rotors   |
| Blade.csv         | =>   | ../data/rotors   |
| ChordDist.csv     | =>   | ../data/rotors   |
| HeightDist.csv    | =>   | ../data/rotors   |
| PitchDist.csv     | =>   | ../data/rotors   |
| SweepDist.csv     | =>   | ../data/rotors   |
| AirfoilGeo1.csv*  | =>   | ../data/airfoils |
| AirfoilGeo2.csv   | =>   | ../data/airfoils |
| AirfoilGeox.csv   | =>   | ../data/airfoils |
| AirfoilPol1.dat** | =>   | ../data/airfoils |
| AirfoilPol2.dat   | =>   | ../data/airfoils |
| AirfoilPolx.dat   | =>   | ../data/airfoils |

*Note: This can be as many geometry files as you would like to include along the blade.

**Note: Airfoil Polars are needed if xfoil is set to false. An airfoil polar must be provided for every airfoil.





