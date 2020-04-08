#=##############################################################################
# DESCRIPTION
    Windcraft vehicle kinematic maneuver definition.

# AUTHORSHIP
  * Author    : Judd Mehr and Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Apr 2020
  * License   : MIT
=###############################################################################

function generate_maneuver_windcraft_kinematic(nrevs;
                                                 disp_plot=false,
                                                 includetail=true,
                                                 includewing=true,
                                                 includecontrols=true,
                                                 includerotors=true)

    omegamax = pi/180 * maximum(omegastar.(linspace(0, 1, 361), nrevs))

    ############################################################################
    # AIRCRAFT VELOCITY
    ############################################################################
    """
     Receives a nondimensional time between 0 and 1, and returns the
     vector of velocity of the vehicle at that instant.
    """
    function Vvehicle(t)

        theta = thetastar_periodic(t, nrevs)*pi/180
        omega = omegastar(t, nrevs)*pi/180
        scaling = omega/omegamax                # Scales velocity to a maximum value of 1

        Vcomp = [0.0, cos(theta), -sin(theta)]  # Counter-clockwise rotation
        return scaling*Vcomp
    end

    ############################################################################
    # AIRCRAFT ANGLES
    ############################################################################
    """
        Receives a nondimensional time between 0 and 1, and returns the angle
        (in degrees) of the vehicle.
        Returns: (angle_aircraft_x, angle_aircraft_y, angle_aircraft_z)
    """
    function anglevehicle(t)

        angle = [0.0, 0.0, thetastar_periodic(abs(t), nrevs)]

        if t < 0.0
            return -angle
        else
            return angle
        end

    end

    ############################################################################
    # ANGLES OF CONTROL SURFACES
    ############################################################################

    """
        Receives a nondimensional time between 0 and 1, and returns the angles
        (in degrees) of the tilting system relative to the aircraft axis.
    """
    function angle_default(t)
        return [0, 0, 0]
    end

    if includetail == true
        angle = Tuple(angle_default for i in 1:6)  # Angle of each tilting system

    elseif includewing == true && includecontrols == true
        angle = Tuple(angle_default for i in 1:4)

    else
        angle = ()
    end


    ############################################################################
    # ROTORS RPM
    ############################################################################
    """
        Receives a nondimensional time between 0 and 1, and returns the RPM
        non-dimensionalized by the hover RPM.
    """
    function RPM_default(t)
        tsr = 4.0
        R = 1.0

        v = Vvehicle(t)
        vmag = sqrt(v[1]^2 + v[2]^2 + v[3]^2)
        omega = tsr*vmag/R
        rpm = omega*pi/30

        return rpm
    end


    if includerotors == true
        RPM = (RPM_default,)        # RPM of each rotor system
    else
        RPM = ()
    end


    ############################################################################
    # MANEUVER OBJECT
    ############################################################################
    maneuver = uns.KinematicManeuver(angle, RPM, Vvehicle, anglevehicle)

    if disp_plot
        uns.plot(maneuver)
    end

    return maneuver

end




"""
angle theta output in degrees
"""
function thetastar(tstar; theta0=0.0, thetan=360.0, omegan=[4.0/9.0;20.0/27.0;4.0/9.0], tn=[0.0;0.5;1.0])
    #find index of max tn less than tstar and set to index variable m.
    if tstar==tn[end]
        m = length(tn)
    elseif tstar==tn[1]
        m = 1
    else

        m = maximum(find(x->x<tstar,tn))
    end
    # m = searchsortednearest(tn,tstar)
    #loop from 1 to m and sum areas.
    areas = 0.0
    den = 0.0
    for i = 2:length(tn)
        if i <= m
            areas += (tn[i]-tn[i-1])*(omegan[i]+omegan[i-1])/2.0
        end
        den += (tn[i]-tn[i-1])*(omegan[i]+omegan[i-1])/2.0
    end

    #add integral of final partial area up to tstar
    if m < length(tn) #if we haven't already integrated over everything
        integral = ( tstar - tn[m] ) * ( omegan[m] + (omegan[m+1]-omegan[m])/2.0 * (tstar-tn[m])/(tn[m+1]-tn[m]) )
    else
        integral = 0.0
    end

    #solve for scale factor, which
    sf = thetan / den
    #add theta0 (constant from integration)
    theta = sf * (areas + integral) + theta0

    return theta
end

"""
Transform time of thetastar and make it periodic to extend the angle output
outside the range [0, 360]
"""
function thetastar_periodic(t, nrevs; optargs...)
    tstar = (t*nrevs)%1    # Convert general time to time of one revolution
    return 360*floor(t*nrevs) + thetastar(tstar; optargs...)
end

function omegastar(tstar, nrevs; h=1e-8, optargs...)
    return (thetastar_periodic(tstar+h, nrevs; optargs...)
                - thetastar_periodic(tstar, nrevs; optargs...))/h
end
