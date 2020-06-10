# Define Complex Wings

In the tutorial [First Steps](@ref) we introduced the ```simpleWing()``` function.  In this guide, we look at creating more complex wings using the ```complexWing()``` function. Note that this is a how to guide, showing how to use the ```complexWing()``` function. For details on this function implementation, please refer to the refernce section of this documentation.

## Define a Symmetric Wing

In order to use the ```complexWing()``` function, we only need to understand that we are responsible for defining the elements of each chord section individually, which gives us greater flexibility than the ```simpleWing()``` function, where many assumptions are made for the user.

Let us begin by defining some of the wing parameters

```
span = 1.0                      #length of full span
numlattice = 10                 #number of lattice elements on half-span
```

Now let's define the parameters of each chord.  Note that position is normalized by the span, and length is normalized by the tip chord. It is generally easiest to define things absolutely and then normalize.


Also Since we are creating a symmetric wing, we only need information from the center to the tip of the wing, postive indicating along the positive axis for the wing system.

```
poschord = [0.0; 0.5; 1.0]./span                #postion of chord stations
lengthtipchord = 0.3                            #length of tip chord
lengthchord = [0.5; 0.4; 0.3]./lengthtipchord   #length of chords
twistchord = [0.0; 0.0; -3.0]                   #twist of chords, in degrees
```

Then let's define the parameters of each section between chord definitions.

```
sweepsection = [10.0; 5.0]      #sweep of sections between chords, in degrees
dihedralsection = [0.0; 7.0]    #dihedral of sections between chords, in degrees
```

We also need to define the aspect ration of the wing, which is simply the absolute span over the absolute tip chord length.

```
aspectratio = span/lengthtipchord
```

With this, we can call the ```complexWing()``` function for a symmetric wing.

```
mainwing = vlm.compleWing(span, aspectratio, numlattice, poschord, lengthchord, twistchord, sweepsection, dihedralsection; symmetric=true)
```

![symmetric wing](../assets/howtofigs/sym-wing.gif)

## Define a Non-symmetric Wing

A good example of a non-symmetric wing is something like a vertical stabilizer. The definitions are basically the same, we just set the symmetric flag to false when creating the wing.

```
span = 0.15                                     #length of full span
numlattice = 5                                  #number of lattice elements on half-span
poschord = [0.0; 0.5; 1.0]./span                #postion of chord stations
lengthtipchord = 0.025                          #length of tip chord
lengthchord = [0.03; 0.025]./lengthtipchord     #length of chords
twistchord = [0.0; 0.0]                         #twist of chords, in degrees
sweepsection = [10.0; 5.0]                      #sweep of sections between chords, in degrees
dihedralsection = [0.0; 7.0]                    #dihedral of sections between chords, in degrees
aspectratio = span/lengthtipchord

verticalstabilizer = vlm.compleWing(span, aspectratio, numlattice, poschord, lengthchord, twistchord, sweepsection, dihedralsection; symmetric=false, chordalign=1.0)
```

!!! note "Chord Alignment"

    The user can choose where to align the chords. A value of 0.0 aligns along the leading edge, while a value of 1.0 aligns along the trailing edge. Values between 0.0 and 1.0 will align along that ratio of the chord lengths.  For example, if you wanted to align the wing along the quarter-chord postition, you would set ```cordalign = 0.25```

Now our verticle stabilizer is defined, but it's not really verticle yet, and it is positioned at the default location, where our main wing is as well.  To make it a verticle stabilizer, we'll need to rotate and translate it.  To do so, we need to define an origin and coordinate system for the verticle stabilizer wing object.

```
originvstab = [0.5; 0.0; 0.0]                       #origin moved 0.5 in positive x-direction
csysvstab = [1.0 0.0 0.0; 0.0 0.0 1.0; 0.0 1.0 0.0] #csys rotated 90 degrees from default
```

Then we can set the coordianate system.

```
vlm.setcoordsystem(verticalstabilizer,originvstab,csysvstab)
```

![non-symmetric wing](../assets/howtofigs/non-sym-wing.gif)

## Define Control Surfaces

There is no automatic way to define control surfaces. Each control surface will need to be defined as its own wing object and placed manually.