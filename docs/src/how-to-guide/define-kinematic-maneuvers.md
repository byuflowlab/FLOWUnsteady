# How to Define Kinematic Maneuvers

There are two ways to generate a maneuver in FLOWUnsteady, prescribing a kinematic maneuver and prescribing actions to generate motion. Prescribing a kinematic maneuver, you inform the simulator where the aircraft is moving and the simulator determines the effects. When you prescribe an action, like running the rotors, to the simulator, the simulator then determines the forces on the aircraft that generate motion.



# Prescribe a Kinematic Maneuver

To prescribe a kinematic maneuver you must generate a maneuver object, setup the simulation, and then simulate. Here we use the kinematic maneuver from the Vahana example to further explain. 

## Generate a Maneuver Object

##### Generate Geometry

Before generating a simulation object, you must have generated the geometry object. If you are unsure of how to generate geometry, refer to the [tutorial](@ref First Steps). 

```julia
 # Generate geometry
    (vehicle, grounds) = generate_geometry_vahana(; n_factor=1,
                                                    xfoil=false,
                                                    data_path=data_path,
                                                    run_name=run_name,
                                                    optargs...)
```



##### Define reference velocity, RPM, total time, and number of steps

```julia
    # Maneuver to perform
    Vref = 0.25 * 125*0.44704           # Cruise speed
    RPMref = 100                        # Reference RPM
    ttot = 30.0                         # Total time to perform maneuver
    nsteps = 1500                       # Time steps
```



##### Define an Angle function

```julia
   """
        Receives a nondimensional time between 0 and 1, and returns the angles
        (in degrees) of the main wing relative to the aircraft axis.
    """
    function angle_main(t)

        # ------------ TAKE OFF ------------------------------------------------
        if t<t1
            return [0, 90, 0]

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t2
            val = 1.5 * (t-t1) / (t2-t1)
            val1 = val + 0.25
            angle_main = 90 - 90*(1-exp(-(val1)^5))
            return [0, angle_main, 0]

        # ------------ CRUISE --------------------------------------------------
        elseif t<t3
            return [0, 0, 0]

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t4
            # Weibull decceleration to hover
            val = 1.5 * (1 - (t-t3) / (t4-t3))
            angle_main = 90 - 90*(1-exp(-(val)^5))
            return [0, angle_main, 0]

        # ------------ LANDING -------------------------------------------------
        else
            return [0, 90, 0]
        end
    end
```



##### Define RPM function

```julia
"""
        Receives a nondimensional time between 0 and 1, and returns the
        main wing RPM non-dimensionalized by the hover RPM.
    """
    function RPM_main(t)

        # ------------ TAKE OFF ------------------------------------------------
        if t<t1
            # Weibull acceleration to target climb
            if t<t1/2
                val = t / (t1/2)
                RPM_w = RPM1*(1.05-exp(-(5*val)^2))
            # Weibull decceleration to hover
            else
                val = 1 - (t-t1/2) / (t1/2)
                RPM_w = 1.0 + (RPM1-1.0)*(1-exp(-(2*val)^5))
            end

            return RPM_w

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t2
            # Increases RPM to sustain forward flight and hover
            if (t-t1)<(t2-t1)*0.25
                val = (t-t1)/((t2-t1)*0.25)
                RPM_w = 1.0 + (RPM2-1.0)*(1-exp(-(2*val)^5))
            # Decreases RPM as it tilts the wing out of hover support
            else
                val = 1 - ((t2-t)/(t2-t1)-0.25) / (1-0.25)
                RPM_w = RPM2 + (RPM3-RPM2)*(1-exp(-(1.1*val)^5))
            end

            return RPM_w

        # ------------ CRUISE --------------------------------------------------
        elseif t<t3
            RPM_w = RPM3
            return RPM_w

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t4
            # Weibull decceleration to hover
            val = 1.5 * (1 - (t-t3) / (t4-t3))
            RPM_w = RPM4 + (RPM3-RPM4)*(1-exp(-(val)^3))
            return RPM_w

        # ------------ LANDING -------------------------------------------------
        else
            # Weibull acceleration to target descend
            if (t-t4)<(1-t4)*0.55
                val = (t-t4) / ((1-t4)*0.55)
                RPM_w = RPM4 + (RPM5-RPM4)*(1-exp(-(3*val)^5))
            # Weibull decceleration to hover
            else
                val = ((t-t4) - (1-t4)*0.55) / ((1-t4)*(1-0.55))
                val = val + 0.60
                RPM_w = RPM5*(3.0*(val)^(3.0-1)*exp(-(val)^5.0))/1.0
            end

            return RPM_w

        end
    end
```



##### Define Vehicle Velocity Function

```julia
 """
        Receives a nondimensional time between 0 and 1, and returns the
        vector of velocity of the aircraft at that instant.
    """
    function Vaircraft(t)

        # ------------ TAKE OFF ------------------------------------------------
        if t<t1

            # Weibull acceleration to target climb
            if t<t1/2
                val = t / (t1/2)
                Vz = V0 + V1*(1-exp(-(2*val)^5))
            # Weibull decceleration to hover
            else
                val = 1 - (t-t1/2) / (t1/2)
                Vz = V1*(1-exp(-(2*val)^5))
            end

            return [0, 0, Vz]

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t2

            # Weibull acceleration to cruise
            val = 1.5 * (t-t1) / (t2-t1) + 0.25
            Vx = V3*(1-exp(-(val)^5))

            # Slight drop during transition
            val = (1/0.8) * (t-t1) / (t2-t1)
            Vz = 0.5 * (5 * val^4 * exp(-val^5))
            Vz = 0.2*V3 * Vz

            return [-Vx, 0, -Vz]

        # ------------ CRUISE --------------------------------------------------
        elseif t<t3

            return [-V3, 0, 0]

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t4

            # Weibull decceleration to hover
            val = 1.5 * (1 - (t-t3) / (t4-t3))
            Vx = V3*(1-exp(-(val)^5))

            # Slight climb during transition
            val = 2.0 * (t-t3) / (t4-t3)
            Vz = 0.5 * (5 * val^4 * exp(-val^5))
            Vz = 0.2*V3 * Vz

            return [-Vx, 0, Vz]

        # ------------ LANDING -------------------------------------------------
        else

            # Weibull acceleration to target descend
            if (t-t4)<(1-t4)*0.35
                val = (t-t4) / ((1-t4)*0.35)
                Vz = V5*(1-exp(-(2*val)^5))
            # Weibull decceleration to hover
            else
                val = 1 - ((t-t4) - (1-t4)*0.35) / ((1-t4)*(1-0.35))
                Vz = V0 + V5*(1-exp(-(2*val)^10))
            end

            return [0, 0, -Vz]

        end
    end
```



##### Define Vehicle Angle function

```julia
 """
        Receives a nondimensional time between 0 and 1, and returns the angle
        (in degrees) of the aircraft.
        Returns: (angle_aircraft)
    """
    function angleaircraft(t)

        # ------------ TAKE OFF ------------------------------------------------
        if t<t1
            return [0, 0, 0]

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t2
            val = 1.5 * (t-t1) / (t2-t1)
            val3 = 1.5*val
            val3 = (val3)^(val3 < 1 ? 3 : 1.5)
            angle_aircraft = -15*(1.5/0.75 * val3^0.5 * exp(-(val3^1.5)))
            return [0, angle_aircraft,0 ]

        # ------------ CRUISE --------------------------------------------------
        elseif t<t3
            return [0, 0, 0]

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t4
            # Weibull decceleration to hover
            val = 1.5 * (1 - (t-t3) / (t4-t3))
            # Slight climb during transition
            val = 2.0 * (t-t3) / (t4-t3)
            angle_aircraft = 15 * 0.5 * (5 * val^4 * exp(-val^5))
            return [0, angle_aircraft, 0]

        # ------------ LANDING -------------------------------------------------
        else
            return [0, 0, 0]
        end
    end

```

##### Define a Maneuver

The maneuver is an object that returns the angle of tilting surfaces, RPM of rotor sets, translational velocity and angle of the vehicle. Each of these attributes must be defined as a function of non-dimensional time. Every vehicle attribute must be accounted for, even if a blank function is placed in the object. 

```julia
    angle = (angle_main, angle_tandem)
    if add_rotors
        RPM = (RPM_main, RPM_tandem)
    else
        RPM = ()
    end
    Vvehicle = Vaircraft
    anglevehicle = angleaircraft

    maneuver = uns.KinematicManeuver(angle, RPM, Vvehicle, anglevehicle)
```

!!! warning

​    The simulator modifies the velocity and angle of the vehicle by using the derivative velocity and angle functions respectively. The angle, RPM, tilt angle, and velocity functions do not directly define the current motion. This also means that the derivative of each of these functions must be defined (The plot of each function must be continuous.).

##### Define initial velocity

The initial state of the vehicle must be defined. 

```julia
    Vinit = Vref*maneuver.Vvehicle(0)       # (m/s) initial vehicle velocity
                                            # (rad/s) initial vehicle angular velocity
    Winit = pi/180 * (maneuver.anglevehicle(0+1e-12)-
                                          maneuver.anglevehicle(0))/(ttot*1e-12)
```

!!! Tip

​    The initial position, tilt-angle and vehicle angle is defined when defining the geometry.  



## Simulate

##### Generate a Simulation object

```julia
    simulation = uns.Simulation(vehicle, maneuver, Vref, RPMref, ttot;
                                                    Vinit=Vinit, Winit=Winit)
```

##### Simulate or visualize

```julia
    # ----------------- SIMULATION MONITOR -------------------------------------
    monitor = generate_monitor_vahana(vehicle, rho, RPMref, nsteps, save_path, Vinf)

    # ----------------- RUN SIMULATION -----------------------------------------
    pfield = uns.run_simulation(simulation, nsteps;
                                      # SIMULATION OPTIONS
                                      Vinf=Vinf,
                                      rho=rho,
                                      mu=mu,
                                      # SOLVERS OPTIONS
                                      p_per_step=p_per_step,
                                      overwrite_sigma=overwrite_sigma,
                                      vlm_sigma=vlm_sigma,
                                      vlm_rlx=vlm_rlx,
                                      surf_sigma=surf_sigma,
                                      max_particles=max_particles,
                                      shed_unsteady=shed_unsteady,
                                      extra_runtime_function=monitor,
                                      # OUTPUT OPTIONS
                                      save_path=save_path,
                                      run_name=run_name,
                                      prompt=prompt,
                                      verbose=verbose, v_lvl=v_lvl,
                                      save_code=splitdir(@__FILE__)[1]
                                      )
```

