#=##############################################################################
# DESCRIPTION
    Maneuver kinematics definitions of every vehicle. Keep adding more maneuvers
    to this file as needed.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Oct 2019
  * License   : MIT
=###############################################################################

#=
    Maneuver functions return a tuple of three functions `(Vaircraft, angles, RPMs)`
    where `Vaircraft(t)` is the translation velocity of both `system` and
    `fuselage`, `angles(t)[i]` is the tilt angle of `tilting_systems[i]`,
    `RPMs(t)[i]` is the RPM of `rotors_systems[i]`. `angles(t)[end]` is
    the tilt angle of the entire aircraft.

    Each of the three functions received a non-dimensional time between 0 and 1
    (where 0 is the beginning of the maneuver, and 1 is the end of the maneuver)
    and returns non-dimensionalized outputs: `Vaircraft(t)` returns a components
    of velocity [vx, vy, vz] normalized by the cruise speed; `angles(t)[i]`
    returns the tilt of the i-th tilt system in degrees; `RPMs(t)[i]` returns
    the RPM of the rotors in the i-th tilt system normalized by the hover RPM of
    the main wing.

=#

################################################################################
# VAHANA
################################################################################
function maneuver_vahana1(; Mtip=0.1,
                            verbose=true, v_lvl=1,
                            visualize=true, vis_nsteps=1000)

    # NOTE: All of the following number are non-dimensional and scaled between
    # 0 and 1, with 0 and 1 beginning and end of simulation, or a scaling
    # parameter

    # End time of each stage
    #  Stage 1: [0, t1]  -> Take off
    #  Stage 2: [t1, t2] -> Transition
    #  Stage 3: [t2, t3] -> Cruise
    #  Stage 4: [t3, t4] -> Transition
    #  Stage 5: [t4, 1]  -> Landing
    t1, t2, t3, t4 = 0.2, 0.3, 0.5, 0.6

    # Target velocity at each stage (ratio of cruise velocity)
    V1 = 0.25
    V2 = 0.10
    V3 = 1.00
    V4 = V2
    V5 = 0.5*V1

    # Target RPM at each stage (ratio of hover RPM)
    RPM1 = 1.10
    RPM2 = 1.50
    RPM3 = 0.75
    RPM4 = 1.00
    RPM5 = 0.90

    r_RPMh_tw = 0.75               # Ratio between tandem and main wing RPM in hover

    # NOTE: -x is in the direction of flight and +z is climb with ground at z=0




    ############################################################################
    # AIRCRAFT VELOCITY
    ############################################################################
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
                Vz = V1*(1-exp(-(2*val)^5))
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
                Vz = V5*(1-exp(-(2*val)^10))
            end

            return [0, 0, -Vz]

        end
    end

    ############################################################################
    # WINGS AND AIRCRAFT ANGLES
    ############################################################################
    """
        Receives a nondimensional time between 0 and 1, and returns the angle
        (in degrees) of both main and tandem wings relative to the aircraft
        axis, and pitch of aircraft.
        Returns: (angle_main, angle_tandem, angle_aircraft)
    """
    function angles(t)

        # ------------ TAKE OFF ------------------------------------------------
        if t<t1

            return (90, 90, 0)

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t2

            val = 1.5 * (t-t1) / (t2-t1)

            val1 = val + 0.25
            angle_main = 90 - 90*(1-exp(-(val1)^5))

            val2 = val - 0.00
            angle_tandem = 90 - 90*(1-exp(-(val2)^6))
            # val2 = val
            # angle_tandem = 90 - 90*(1-exp(-(val2)^1.5))

            val3 = 1.5*val
            val3 = (val3)^(val3 < 1 ? 3 : 1.5)
            angle_aircraft = -15*(1.5/0.75 * val3^0.5 * exp(-(val3^1.5)))

            return (angle_main, angle_tandem, angle_aircraft)

        # ------------ CRUISE --------------------------------------------------
        elseif t<t3

            return (0, 0, 0)

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t4

            # Weibull decceleration to hover
            val = 1.5 * (1 - (t-t3) / (t4-t3))
            angle_main = 90 - 90*(1-exp(-(val)^5))
            angle_tandem = 90 - 90*(1-exp(-(val)^5))

            # Slight climb during transition
            val = 2.0 * (t-t3) / (t4-t3)
            angle_aircraft = 15 * 0.5 * (5 * val^4 * exp(-val^5))

            return (angle_main, angle_tandem, angle_aircraft)

        # ------------ LANDING -------------------------------------------------
        else

            return (90, 90, 0)

        end
    end




    ############################################################################
    # ROTORS RPM
    ############################################################################
    """
        Receives a nondimensional time between 0 and 1, and returns the
        RPM non-dimensionalized by the hover RPM of main and tandem wing as
        `(RPM_w, RPM_tw)`.
    """
    function RPMs(t)

        # ------------ TAKE OFF ------------------------------------------------
        if t<t1

            # Weibull acceleration to target climb
            if t<t1/2
                val = t / (t1/2)
                RPM_w = RPM1*(1-exp(-(5*val)^2))
                RPM_tw = r_RPMh_tw*RPM_w
            # Weibull decceleration to hover
            else
                val = 1 - (t-t1/2) / (t1/2)
                # RPM_w = RPM1 * ( RPM2 + RPM1*(1-exp(-(2*val)^5)) ) / (RPM2 + RPM1)
                RPM_w = 1.0 + (RPM1-1.0)*(1-exp(-(2*val)^5))
                RPM_tw = r_RPMh_tw*RPM_w
            end

            return (RPM_w, RPM_tw)

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

            # Increases RPM to sustain forward flight and hover
            if (t-t1)<(t2-t1)*0.65
                val = (t-t1)/((t2-t1)*0.65)
                RPM_tw = r_RPMh_tw*(1.0 + (RPM2-1.0)*(1-exp(-(2*val)^5)))
            # Decreases RPM as it tilts the wing out of hover support
            else
                # val = 1 - ((t2-t)/(t2-t1)-0.65) / (1-0.65)
                # RPM_tw = r_RPMh_tw*(RPM2 + (RPM3-RPM2)*(1-exp(-(0.2*val)^1)))
                val = 1 - (t2-t)/(t2-((t2-t1)*0.65 + t1))
                RPM_tw = r_RPMh_tw*(RPM2 + val*(RPM3-RPM2))
            end
            # Weibull acceleration to cruise
            # val = 1.5 * (t-t1) / (t2-t1) + 0.25
            # RPM_w = 1.0 + (RPM2-1.0)*(1-exp(-(val)^5))

            return (RPM_w, RPM_tw)

        # ------------ CRUISE --------------------------------------------------
        elseif t<t3
            RPM_w = RPM3
            RPM_tw = r_RPMh_tw * RPM3

            return (RPM_w, RPM_tw)

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t4

            # Weibull decceleration to hover
            val = 1.5 * (1 - (t-t3) / (t4-t3))
            RPM_w = RPM4 + (RPM3-RPM4)*(1-exp(-(val)^3))
            RPM_tw = r_RPMh_tw * (RPM4 + (RPM3-RPM4)*(1-exp(-(0.8*val)^8)))

            return (RPM_w, RPM_tw)

        # ------------ LANDING -------------------------------------------------
        else

            # Weibull acceleration to target descend
            if (t-t4)<(1-t4)*0.55
                val = (t-t4) / ((1-t4)*0.55)
                RPM_w = RPM4 + (RPM5-RPM4)*(1-exp(-(3*val)^5))
                RPM_tw = r_RPMh_tw * RPM_w
            # Weibull decceleration to hover
            else
                # val = 1 - ((t-t4) - (1-t4)*0.55) / ((1-t4)*(1-0.55))
                # RPM_w = RPM5*(1-exp(-(2*val)^5))

                val = ((t-t4) - (1-t4)*0.55) / ((1-t4)*(1-0.55))
                val = val + 0.60
                RPM_w = RPM5*(3.0*(val)^(3.0-1)*exp(-(val)^5.0))/1.0

                RPM_tw = r_RPMh_tw * RPM_w
            end

            return (RPM_w, RPM_tw)

        end
    end





    ############################################################################
    # VISUALIZATION
    ############################################################################

    if visualize
        for i in 1:10
            close()
        end

        figure(figsize=[7*2, 5*2])

        # -------------------- Velocity history --------------------------------
        subplot(221)
        ts = linspace(0, 1, vis_nsteps)
        Vs = Vaircraft.(ts)
        Vmax = max([maximum([V[i] for V in Vs]) for i in 1:3]...)
        Vmin = min([minimum([V[i] for V in Vs]) for i in 1:3]...)

        plot(ts, [V[1] for V in Vs], "-g", label=L"V_x", alpha=0.8)
        plot(ts, [V[2] for V in Vs], "-b", label=L"V_y", alpha=0.8)
        plot(ts, [V[3] for V in Vs], "-r", label=L"V_z", alpha=0.8)

        for tstage in [t1, t2, t3, t4]
            plot(tstage*ones(2), [Vmin, Vmax], ":k", alpha=0.5)
        end

        xlabel("Non-dimensional time")
        ylabel("Non-dimensional velocity")
        legend(loc="best", frameon=false)

        # -------------------- Mission profile ---------------------------------
        Xinit = zeros(3)        # Initial position
        dt = ts[2]-ts[1]        # Time step

        Xs = [Xinit]
        for i in 1:(size(ts,1)-1)
            push!(Xs, Xs[end] + dt*Vs[i])
        end

        subplot(222)
        Xmax = max([maximum([X[i] for X in Xs]) for i in 1:3]...)
        Xmin = min([minimum([X[i] for X in Xs]) for i in 1:3]...)

        plot(ts, [X[1] for X in Xs], "-g", label=L"x", alpha=0.8)
        plot(ts, [X[2] for X in Xs], "-b", label=L"y", alpha=0.8)
        plot(ts, [X[3] for X in Xs], "-r", label=L"z", alpha=0.8)

        for tstage in [t1, t2, t3, t4]
            plot(tstage*ones(2), [Xmin, Xmax], ":k", alpha=0.5)
        end

        xlabel("Non-dimensional time")
        ylabel("Non-dimensional position")
        legend(loc="best", frameon=false)


        # -------------------- Angles history ----------------------------------
        subplot(224)
        as = angles.(ts)

        amax = max([maximum([a[i] for a in as]) for i in 1:3]...)
        amin = min([minimum([a[i] for a in as]) for i in 1:3]...)

        plot(ts, [a[1] for a in as], "-g", label="Main", alpha=0.8)
        plot(ts, [a[2] for a in as], "-b", label="Tandem", alpha=0.8)
        plot(ts, [a[3] for a in as], "-r", label="Aircraft", alpha=0.8)

        for tstage in [t1, t2, t3, t4]
            plot(tstage*ones(2), [amin, amax], ":k", alpha=0.5)
        end

        xlabel("Non-dimensional time")
        ylabel(L"Angle ($^\circ$)")
        legend(loc="best", frameon=false)


        # -------------------- RPMs history ----------------------------------
        subplot(223)
        rpms = RPMs.(ts)

        rpmmax = max([maximum([rpm[i] for rpm in rpms]) for i in 1:2]...)
        rpmmin = min([minimum([rpm[i] for rpm in rpms]) for i in 1:2]...)

        plot(ts, [rpm[1] for rpm in rpms], "-g", label="Main", alpha=0.8)
        plot(ts, [rpm[2] for rpm in rpms], "-b", label="Tandem", alpha=0.8)

        for tstage in [t1, t2, t3, t4]
            plot(tstage*ones(2), [rpmmin, rpmmax], ":k", alpha=0.5)
        end

        xlabel("Non-dimensional time")
        ylabel("Non-dimensional RPM (RPM/RPMh)")
        legend(loc="best", frameon=false)

    end

    return Vaircraft, angles, RPMs
end
# -------------- END OF VAHANA -------------------------------------------------




################################################################################
# COMMON FUNCTIONS
################################################################################

function visualize_kinematic(maneuver::Function, system, rotors, tilting_systems,
                             rotors_systems, fuselage;
                             # SIMULATION OPTIONS
                             Vcruise=125*0.44704,       # Cruise speed
                             RPMh_w=400,                # RPM of main wing rotors in hover
                             telapsed=30.0,             # Total time to perform maneuver
                             nsteps=100,                # Time steps
                             rand_RPM=true,             # Randomize RPM fluctuations
                             # OUTPUT OPTIONS
                             save_path="temps/vahanageometry02",
                             run_name="vahana",
                             prompt=true,
                             verbose=true, v_lvl=1, verbose_nsteps=10)



     speedofsound = 342                  # (m) speed of sound
     R = 0.75                            # (m) rotor radius

     # RPMh_w = Vtip * 60/(2*pi*R)         # RPM in main wing in hover
     Vtip = 2*pi*RPMh_w/60*R           # Velocity of blade tip in main wing in hover
     Mtip = Vtip/speedofsound
     revs_per_sec = RPMh_w/60

     println("\t"^(v_lvl)*"Tip Mach:\t$(round(Mtip, 2))")
     println("\t"^(v_lvl)*"RPM:\t\t$(ceil(Int, RPMh_w))")
     println("\t"^(v_lvl)*"Revs per sec:\t$(round(revs_per_sec, 1))")

     Vinf(x,t) = [1,0,0]
     vlm.setVinf(system, Vinf)
     for rotor in rotors; vlm.setRPM(rotor, 6000); end;

     vlm.vtk.create_path(save_path, prompt)

     Vaircraft, angles, RPMs = maneuver(; verbose=verbose, v_lvl=v_lvl+1, vis_nsteps=nsteps)

    println("\t"^(v_lvl)*"*******************************************************************")
    println("\t"^(v_lvl)*"START $(save_path!=nothing ? joinpath(save_path,run_name) : "")")
    println("\t"^(v_lvl)*"*******************************************************************")

    prev_angles = Float64[angles(0)...]
    O_fuselage = zeros(3)

    dt = telapsed/nsteps

    println("\t"^(v_lvl)*"Rotation per step:\t$(round(360*revs_per_sec*dt, 2)) deg")

    for i in 0:nsteps
        # Verbose
        if verbose && i%verbose_nsteps==0
            println("\t"^(v_lvl+1)*"Time step $i out of $nsteps")
        end

        # Step in time
        if i!=0
            t = i*dt

            # Translation
            V = Vcruise * Vaircraft(t/telapsed)
            dX = V*dt

            vlm.setcoordsystem(system, system.O + dX, system.Oaxis)
            vlm.vtk.lintransform!(fuselage, eye(3), dX)

            O_fuselage .+=  dX

            # Rotation of tilting systems and aircraft (dangles[end] is aircraft)
            new_angles = [angles(t/telapsed)...]
            dangles = new_angles - prev_angles

            for (j, dangle) in enumerate(dangles)

                Oaxis = vlm.vtk.rotation_matrix2(0, -dangle, 0)

                if j != size(dangles, 1)        # Titlting systems
                    sys = tilting_systems[j]
                else                            # Aircraft and fuselage
                    sys = system
                    vlm.vtk.lintransform!(fuselage, eye(3), -O_fuselage)
                    vlm.vtk.lintransform!(fuselage, Oaxis, O_fuselage)
                end

                vlm.setcoordsystem(sys, sys.O, Oaxis*sys.Oaxis)
            end

            prev_angles[:] = new_angles


            # Rotation of rotors in every tilting system
            rpms = RPMs(t/telapsed)
            for j in 1:size(rpms, 1)
                for rotor in rotors_systems[j]
                    rpm = rpms[j] * (rand_RPM ? 1 + (rand()-0.5)*0.1 : 1.0)
                    rotation = 360*(RPMh_w*rpm)/60 * dt
                    vlm.rotate(rotor, rotation)
                end
            end

        end

        # Output vtks
        vlm.save(system, run_name; save_horseshoes=false, path=save_path, num=i)
        vlm.vtk.save(fuselage, run_name*"_FuselageGrid"; path=save_path, num=i)

    end
end
