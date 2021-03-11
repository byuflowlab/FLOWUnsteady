#=##############################################################################
# DESCRIPTION
    Vahana vehicle kinematic maneuver definition.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Feb 2021
  * License   : MIT
=###############################################################################


function generate_maneuver_vahana(; disp_plot=false, add_rotors=true, V0=0.0001)

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
    RPM3_stacked = 2/600
    RPM4 = 1.00
    RPM5 = 0.90

    r_RPMh_stup = 2.00             # Ratio between stacked and main rotors in hover
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

    ############################################################################
    # AIRCRAFT ANGLES
    ############################################################################
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

    ############################################################################
    # TILT-WINGS ANGLES
    ############################################################################
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
            val2 = val - 0.00
            angle = 90 - 90*(1-exp(-(val2)^6))
            return [0, angle, 0]

        # ------------ CRUISE --------------------------------------------------
        elseif t<t3
            return [0, 0, 0]

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t4
            # Weibull decceleration to hover
            val = 1.5 * (1 - (t-t3) / (t4-t3))
            angle = 90 - 90*(1-exp(-(val)^5))
            return [0, angle, 0]

        # ------------ LANDING -------------------------------------------------
        else
            return [0, 90, 0]
        end
    end

    """
        Receives a nondimensional time between 0 and 1, and returns the angles
        (in degrees) of the tandem wing relative to the aircraft axis.
    """
    function angle_tandem(t)

        # ------------ TAKE OFF ------------------------------------------------
        if t<t1
            return [0, 90, 0]

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t2
            val = 2.0 * (t-t1) / (t2-t1)
            val1 = val + 0.25
            angle = 90 - 90*(1-exp(-(val1)^5))
            return [0, angle, 0]

        # ------------ CRUISE --------------------------------------------------
        elseif t<t3
            return [0, 0, 0]

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t4
            # Weibull decceleration to hover
            val = 1.5 * (1 - (t-t3) / (t4-t3))
            angle = 90 - 90*(1-exp(-(val)^5))
            return [0, angle, 0]

        # ------------ LANDING -------------------------------------------------
        else
            return [0, 90, 0]
        end
    end





    ############################################################################
    # ROTORS RPM
    ############################################################################
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

    """
        Receives a nondimensional time between 0 and 1, and returns the RPM of
        stacked upper rotors in main wing non-dimensionalized by the hover RPM.
    """
    function RPM_stacked_up(t)

        # ------------ TAKE OFF ------------------------------------------------
        if t<t1
            # Weibull acceleration to target climb
            if t<t1/5
                val = t / (t1/5)
                RPM_w = RPM1*(1.05-exp(-(5*val)^2))
            # Weibull decceleration to hover
            else
                val = 1 - (t-t1/5) / (1-t1/5)
                RPM_w = 1.0 + (RPM1-1.0)*(1-exp(-(2*val)^5))
            end

            return r_RPMh_stup*RPM_w

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t2
            # Increases RPM to sustain forward flight and hover
            if (t-t1)<(t2-t1)*0.65
                val = (t-t1)/((t2-t1)*0.65)
                RPM_tw = r_RPMh_tw*(1.0 + (RPM2-1.0)*(1-exp(-(2*val)^5)))
            # Decreases RPM as it tilts the wing out of hover support
            else
                val = 1 - (t2-t)/(t2-((t2-t1)*0.65 + t1))
                RPM_tw = r_RPMh_tw*(RPM2 + val*(RPM3_stacked-RPM2))
            end

            return RPM_tw

        # ------------ CRUISE --------------------------------------------------
        elseif t<t3
            return RPM3_stacked

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t4
            # Weibull decceleration to hover
            val = 1.5 * (1 - (t-t3) / (t4-t3))
            RPM_w = RPM4 + (RPM3_stacked-RPM4)*(1-exp(-(val)^3))
            return r_RPMh_stup*RPM_w

        # ------------ LANDING -------------------------------------------------
        else
            # Weibull acceleration to target descend
            if (t-t4)<(1-t4)*0.75
                val = (t-t4) / ((1-t4)*0.75)
                RPM_w = RPM4 + (RPM5-RPM4)*(1-exp(-(3*val)^5))
            # Weibull decceleration to hover
            else
                val = ((t-t4) - (1-t4)*0.75) / ((1-t4)*(1-0.75))
                val = val + 0.60
                RPM_w = RPM5*(3.0*(val)^(3.0-1)*exp(-(val)^5.0))/1.0
            end

            return r_RPMh_stup*RPM_w
        end
    end


    """
        Receives a nondimensional time between 0 and 1, and returns the RPM of
        stacked lower rotors in main wing non-dimensionalized by the hover RPM.
    """
    function RPM_stacked_low(t)

        # ------------ TAKE OFF ------------------------------------------------
        if t<t1
            return RPM_stacked_up(t)

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t2
            return RPM_stacked_up(t)

        # ------------ CRUISE --------------------------------------------------
        elseif t<t3
            return RPM_stacked_up(t)

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t4
            return RPM_stacked_up(t)

        # ------------ LANDING -------------------------------------------------
        else
            return RPM_stacked_up(t)
        end
    end

    """
        Receives a nondimensional time between 0 and 1, and returns the
        tandem wing RPM non-dimensionalized by the hover RPM.
    """
    function RPM_tandem(t)

        # ------------ TAKE OFF ------------------------------------------------
        if t<t1
            return r_RPMh_tw*RPM_main(t)

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t2
            # Increases RPM to sustain forward flight and hover
            if (t-t1)<(t2-t1)*0.65
                val = (t-t1)/((t2-t1)*0.65)
                RPM_tw = r_RPMh_tw*(1.0 + (RPM2-1.0)*(1-exp(-(2*val)^5)))
            # Decreases RPM as it tilts the wing out of hover support
            else
                val = 1 - (t2-t)/(t2-((t2-t1)*0.65 + t1))
                RPM_tw = r_RPMh_tw*(RPM2 + val*(RPM3-RPM2))
            end

            return RPM_tw

        # ------------ CRUISE --------------------------------------------------
        elseif t<t3
            return r_RPMh_tw*RPM_main(t)

        # ------------ TRANSITION ----------------------------------------------
        elseif t<t4
            # Weibull decceleration to hover
            val = 1.5 * (1 - (t-t3) / (t4-t3))
            RPM_tw = r_RPMh_tw * (RPM4 + (RPM3-RPM4)*(1-exp(-(0.8*val)^8)))
            return RPM_tw

        # ------------ LANDING -------------------------------------------------
        else
            return r_RPMh_tw*RPM_main(t)
        end
    end



    ############################################################################
    # MANEUVER OBJECT
    ############################################################################
    angle = (angle_main, angle_tandem)
    if add_rotors
        RPM = (RPM_main, RPM_stacked_up, RPM_stacked_low, RPM_tandem)
    else
        RPM = ()
    end
    Vvehicle = Vaircraft
    anglevehicle = angleaircraft

    maneuver = uns.KinematicManeuver(angle, RPM, Vvehicle, anglevehicle)

    if disp_plot
        uns.plot(maneuver)
    end

    return maneuver
end
