module MyVPM

import Dates
using GeometricTools: create_path

mutable struct ParticleField
  # Initialization variables (USER INPUT)
  n_max::Int64                # Max number of particles
  Uinf::Any                   # Freestream velocity function
  Uphi::Any                   # Scalar-potential velocity function
  method::String              # VPM solver method (direct/ExaFMM/directblob)

  # Properties
  np::Int64                   # Current number of particles
  nt::Int64                   # Current time step number
  t::Float64                  # Current time
  nu::Float64                 # Kinematic viscosity for viscous diffusion

  # Internal data structure
  _p_field::Array{Float64,2}      # Current state of all particles

  # Initialization
  ParticleField(  n_max, Uinf, Uphi, method,
                  np=0, nt=0, t=0.0, nu=1.506e-5,
                  _p_field=zeros(0, 0)
        ) = new(  n_max, Uinf, Uphi, method,
                  np, nt, t, nu,
                  _p_field
                )
end

function get_np(self::ParticleField)
  return self.np
end
function addparticle(args...; optargs...)
    nothing
end
function delparticle(args...; optargs...)
    nothing
end
function set_TIMEMETH(args...; optargs...)
    nothing
end
function set_STRETCHSCHEME(args...; optargs...)
    nothing
end
function set_RELAXETA(args...; optargs...)
    nothing
end
function set_PSE(args...; optargs...)
    nothing
end
function set_CS(args...; optargs...)
    nothing
end
function set_P2PTYPE(args...; optargs...)
    nothing
end
function conv(args...; optargs...)
    error("LOGIC ERROR: Dummy function was called!")
end
function reg_Uomega(args...; optargs...)
    error("LOGIC ERROR: Dummy function was called!")
end

"""
    run_vpm!(p_field, dt, nsteps; runtime_function=nothing,
                  save_path=nothing, run_name="pfield", nsteps_save=1,
                  save_code="", prompt=true)
Solves `nsteps` of the particle field with a time step of `dt`.

# Optional Arguments
  * `runtime_function::Any`   : Give it a function of the form
                                `myfun(pfield, t, dt)`. On each time step it
                                will call this function. Use this for adding
                                particles, deleting particles, etc.
  * `solver_method::String`   : Solver method (ExaFMM, direct, directblob).
  * `nsteps_relax::Int64` : Relaxes the particle field every this many time steps.
  * `save_path::Any`      : Give it a string for saving VTKs of the particle
                            field. Creates the given path.
  * `run_name::String`    : Suffix of vtk files.
  * `nsteps_save::Int64`  : Saves vtks every this many time steps.
  * `save_code::String`   : Give it the name of a file and it will save make a
                            copy of this file along with the vtks. Use this for
                            backing up whatever code is used for generating this
                            run.
  * `prompt::Bool`        : If `save_path` already exist, it will prompt the
                            user before overwritting the folder if true; it will
                            directly overwrite it if false.
  * `verbose::Bool`       : Prints progress of the run to the terminal.
  * `verbose_nsteps::Bool`: Number of time steps between verbose.
  * `parallel_Uphi::Bool` : Flag for evaluating Uphi in parallel.
  * `group_Uphi::Bool` : Flag for evaluating Uphi as a group of points.
  * `beta::Real`          : Maximum core growth (sgm/sgm0) in CS scheme.
  * `sgm0::Real`          : Reinitialization core size.
"""
function run_vpm!(p_field::ParticleField, dt::Float64, nsteps::Int64;
                                              runtime_function::Any=nothing,
                                              save_path=nothing,
                                              run_name::String="pfield",
                                              create_savepath::Bool=true,
                                              save_code::String="",
                                              prompt::Bool=true,
                                              verbose::Bool=true,
                                              verbose_nsteps::Int64=10,
                                              profile=false,
                                              optargs...)

    # Create save path
    if save_path!=nothing
        if create_savepath
            create_path(save_path, prompt)
        end

        # Save code
        if save_code!=""
            run(`cp -r $save_code $save_path`)
        end
    end

    if verbose
        time_beg = Dates.DateTime(Dates.now())
        println("*******************************************************************")
        println("START $(save_path!=nothing ? joinpath(save_path,run_name) : "")\t$(time_beg)")
        println("*******************************************************************")
    end

    # RUN
    for i in 0:nsteps
        # Verbose
        if verbose && i%verbose_nsteps==0; println("Time step $i out of $nsteps "); end;

        # Time step
        if i!=0
            p_field.t += dt
            p_field.nt += 1
        end

        # Call user-defined runtime function
        if runtime_function!=nothing

            if profile; tm = time(); print("\truntime_function:\t"); end;
            breakflag = runtime_function(p_field, p_field.t, dt)
            if profile; println(time()-tm); tm = time(); end;

            if breakflag
                break
            end
        end
    end

    if verbose
        time_end = Dates.DateTime(Dates.now())
        hrs,mins,secs = timeformat(time_beg, time_end)
        println("*******************************************************************")
        println("END $(save_path!=nothing ? save_path*run_name : "")\t$(time_end)")
        println("*******************************************************************")
        println("ELAPSED TIME: $(hrs) hours $mins minutes $secs seconds")
    end
end


function timeformat(time_beg, time_end)
    time_delta = Dates.value(time_end)-Dates.value(time_beg)
    hrs = Int(floor(time_delta/1000/60/60))
    mins = Int(floor((time_delta-hrs*60*60*1000)/1000/60))
    secs = Int(floor((time_delta-hrs*60*60*1000-mins*1000*60)/1000))
    return hrs,mins,secs
end

end # END OF MODULE
