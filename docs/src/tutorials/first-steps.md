# First Steps

## Geometry Basics

In this tutorial, we'll begin by defining the geometry for a simple wing.  Note that all the functions used are described in the Reference section of this documentation if you would like further information at any point.  Before you begin this tutorial, it is expected that you've already installed the necessary components of FLOWUnsteady, and that you are familiar with visualization in Paraview.

First things first, we need to include the FLOWUnsteady components
```@setup tut
import FLOWUnsteady
uns = FLOWUnsteady
vlm = uns.vlm

span = 1.0
aspectratio = 10.0
taperratio = 0.5
wingtwist = 0.0
wingsweep = 10.0 #degrees
wingdihedral = 7.0 #degrees
```

```
import FLOWUnsteady
uns = FLOWUnsteady
vlm = uns.vlm
```

Let's begin with a single section, symmetric wing.  We'll start by defining some basic geometry.

```
span = 1.0
aspectratio = 10.0
taperratio = 0.5
wingtwist = 0.0
wingsweep = 10.0 #degrees
wingdihedral = 7.0 #degrees
```

Then we'll call the ```simpleWing()``` function to create a simple wing object.

```@example tut
mainwing = vlm.simpleWing(span,aspectratio,taperratio,wingtwist,wingsweep,wingdihedral)
```

Congratulations! You've created your first wing object.  If you want, you can take a moment to explore the contents of your newly created ```mainwing```.  You can do so using ```fieldnames(mainwing)``` and poking around. The Reference section will have more info on the contents of the wing object.

Next, let's create a wing system.

```@example tut
system = vlm.WingSystem()
```

You now have an empty wing system, so let's add our mainwing object to it with the name "mainwing."

```@example tut
vlm.addwing(system,"mainwing",mainwing)
```

Now that we have a wing system, let's save it as a .vtk file so we can view it in paraview.  In order to do so, we are required to define a freestream velocity.

```@example tut
Vinf(x,t) = [1,0,0]
vlm.setVinf(system, Vinf)
```

We will also want to set some parameters for saving files and set up our file system to put the files where we want.

```@example tut
run_name = "tutorial"
save_path = "./simplewing/"

run(`rm -rf $save_path`)
run(`mkdir $save_path`)
```

Finally, we can save the files.

```
vlm.save(system, run_name; path=save_path)
```

And now we can view our wing in Paraview using the command ```run(`paraview --data="$(save_path)/$(run_name)_mainwing_vlm.vtk"`)``` (assuming you've set up an alias for paraview on your computer).

![alt text](https://media.githubusercontent.com/media/byuflowlab/FLOWUnsteady/master/docs/src/assets/tutorialfigs/geometry-basics.gif)

## Adding a Rotor

Now that we have a basic wing, let's go ahead and add a rotor.  We'll use some data for the rotor that already exists in FLOWUnsteady.  You can visit the How-to guides for more information on creating your own rotor database.

```
rotor_file = "apc10x7.csv"          # Rotor geometry
data_path = uns.def_data_path       # Path to rotor database
```

With the rotor data, we can generate our rotor. This might take a minute or so to run. We supress the output here with a semi-colon as it prints a large output.

```@example tut
rotor_file = "apc10x7.csv"          # hide
data_path = uns.def_data_path       # hide
rotor = uns.generate_rotor(rotor_file; pitch=0.0,
                                            n=10, CW=true, ReD=1.5e6,
                                            verbose=true, xfoil=false,
                                            data_path=data_path,
                                            plot_disc=false);
```

And then we can generate a rotor object, where we again supress the output.

```@example tut
rotors = vlm.Rotor[rotor];
```

This will put the rotor at the default location and orientation which we will define here since we now need to move the rotor relative to the wing which is already at this location.

```@example tut
vehicleorigin = [0.0; 0.0; 0.0]
vehicleaxis = [1.0 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 1.0]
```

To move the rotor, we need to define a new origin point.

```@example tut
rotororigin = [-0.1; 0.0; 0.0]
```

Then we can use that origin to set the rotor coordinate system in order to move the rotor.

```@example tut
for rotor in rotors
    vlm.setcoordsystem(rotor, rotororigin, vehicleaxis; user=true)
end
```

which we can put in a tuple that stores our rotor system(s).

```@example tut
rotors_system = (rotor,);
```

We also need to add it to our overall system.

```@example tut
for rotor in rotors; vlm.addwing(system, run_name, rotor); end;
```

Like setting the Vinf parameter for the main wing, we need to give our rotor an RPM as well.

```@example tut
for rotor in rotors; vlm.setRPM(rotor, 6000); end;
```

We should now be able to visualize our wing with a rotor.

```
run(`rm -rf $save_path`)
run(`mkdir $save_path`)

vlm.save(system, run_name; path=save_path)
run(`paraview --data="$(save_path)/tutorial_mainwing_vlm.vtk;tutorial_tutorial_Blade1_vlm.vtk;tutorial_tutorial_Blade2_vlm.vtk;tutorial_tutorial_Blade1_loft.vtk;tutorial_tutorial_Blade2_loft.vtk;"`)
```

![alt text](https://media.githubusercontent.com/media/byuflowlab/FLOWUnsteady/master/docs/src/assets/tutorialfigs/add-rotor.gif)


## Other Systems


### VLM Systems

### Wake Systems

### Tilting Systems


## Kinematic Maneuvers


## Setting up a Basic Simulation