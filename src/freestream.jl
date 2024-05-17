"""
    AbstractFreestream

Supertype for freestream objects.

"""
abstract type AbstractFreestream end

function initialize_verbose(freestream::AbstractFreestream)
    @error "initialize_verbose not defined for $(typeof(freestream))"
end

"""
    SimpleFreestream <: AbstractFreestream

Bare-bones freestream object.

# Fields

* `velocity::SVector{3,Float64}`: the freestream velocity
* `density::Float64`: the fluid density

"""
struct SimpleFreestream{TF} <: AbstractFreestream
    velocity::SVector{3,TF}
    density::TF
end

"""
    SimpleFreestream(velocity; kwargs...)

Convenience constructor for a `SimpleFreestream` object.

# Arguments

* `velocity::SVector{3,Float64}`: the freestream velocity

# Keyword Arguments

* `density::Float64`: the fluid density

# Returns

* `freestream::SimpleFreestream`: the freestream object
"""
function SimpleFreestream(velocity; density=1.0)
    return SimpleFreestream(velocity, density)
end

#------- verbose -------#

function initialize_verbose(freestream::SimpleFreestream, v_lvl)
    println("\t"^v_lvl, rpad("Type:", 12, " "),  typeof(freestream))
    println("\t"^v_lvl, rpad("Velocity:", 12, " "), freestream.velocity)
    println("\t"^v_lvl, rpad("Density:", 12, " "), freestream.density)
end
