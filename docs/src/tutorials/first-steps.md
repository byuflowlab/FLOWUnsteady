# First Steps

## Geometry Basics

In this tutorial, we'll begin by defining the geometry for a simple wing.  Note that all the functions used are described in the Reference section of this documentation if you would like further information at any point.  Before you begin this tutorial, it is expected that you've already installed the necessary components of FLOWUnsteady, and that you are familiar with visualization in Paraview.

First things first, we need to include the FLOWUnsteady components
```@setup simplewing
import FLOWUnsteady
vlm = FLOWUnsteady.vlm

span = 5.0
aspectratio = 10.0
taperratio = 0.5
wingtwist = 0.0
wingsweep = 10.0 #degrees
wingdihedral = 7.0 #degrees
```

```
import FLOWUnsteady
vlm = FLOWUnsteady.vlm
```

Let's begin with a single section, symmetric wing.  We'll start by defining some basic geometry.

```
span = 5.0
aspectratio = 10.0
taperratio = 0.5
wingtwist = 0.0
wingsweep = 10.0 #degrees
wingdihedral = 7.0 #degrees
```

Then we'll call the ```simpleWing()``` function to create a simple wing object.

```@example simplewing
mainwing = vlm.simpleWing(span,aspectratio,taperratio,wingtwist,wingsweep,wingdihedral)
```

Congratulations! You've created your first wing object.  If you want, you can take a moment to explore the contents of your newly created ```mainwing```.  You can do so using ```fieldnames(mainwing)``` and poking around. The Reference section will have more info on the contents of the wing object.

Next, let's create a wing system.

```@example simplewing
system = vlm.WingSystem()
```

You now have an empty wing system, so let's add our mainwing object to it with the name "mainwing."

```@example simplewing
vlm.addwing(system,"mainwing",mainwing)
```

Now that we have a wing system, let's save it as a .vtk file so we can view it in paraview.  In order to do so, we are required to define a freestream velocity.

```@example simplewing
Vinf(x,t) = [1,0,0]
vlm.setVinf(system, Vinf)
```

We will also want to set some parameters for saving files and set up our file system to put the files where we want.

```@example simplewing
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


##