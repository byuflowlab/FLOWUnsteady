#=##############################################################################
# DESCRIPTION
    Useful macros for metaprogramming.

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Apr 2020
  * License   : MIT
=###############################################################################

"""
  `WingSystem()`
Initiates a system of wings. All method applicable to a Wing object are
applicable to a WingSystem. When solved, it will calculate the interaction
between wings.
"""
type WingSystem
  # Properties
  wings::Array{Any,1}             # Wings in the system
  wing_names::Array{String,1}     # Names of the wings
  O::vlm.FArrWrap                     # Origin of local reference frame
  Oaxis::vlm.FMWrap                   # Unit vectors of the local reference frame
  invOaxis::vlm.FMWrap                # Inverse unit vectors
  Vinf::Any                       # Vinf function used in current solution

  # Data storage
  sol::Dict{String, Any}          # Solution fields available

  WingSystem( wings=[], wing_names=String[],
                O=FWrap[0.0,0.0,0.0],
                Oaxis=FWrap[1.0 0 0; 0 1 0; 0 0 1],
                invOaxis=FWrap[1.0 0 0; 0 1 0; 0 0 1],
                Vinf=nothing,
              sol=Dict()
      ) = new(wings, wing_names,
                O, Oaxis, invOaxis, Vinf,
              sol)
end

"""
    `inherit(name, baseT, suptype, fields)`

Macro for defining the mutable type `name` that inherits all properties (fields)
of `baseT`, in addition to having the properties `fields` and subtyping `suptype`.

# Examples
```julia-repl
julia> abstract type AbstractPerson{M, N} end

julia> mutable struct Person{M, N} <: AbstractPerson{M, N}
            name::M
            age::N
        end

julia> @inherit Citizen Person AbstractPerson{Any, Any} begin
            nationality::String
        end

julia> cit = Citizen("Peter", 22, "USA")
Citizen{String,Int64}("Peter", 22, "USA")

julia> cit.name
"Peter"

julia> cit.nationality
"USA"
```
"""
macro inherit(name, baseT, suptype, fields)

    # Extract fields
    println("Rabbit1")
    base_type = Core.eval(current_module(), baseT)
    base_fieldnames = fieldnames(base_type)
    while typeof(base_type)==UnionAll
        base_type = base_type.body
    end
    base_types = [Symbol(t) for t in base_type.types]
    base_fields = [:($f::$T) for (f, T) in zip(base_fieldnames, base_types)]
    println("Rabbit2")

    # Extract parameters
    base_parameters = [Symbol(p) for p in base_type.parameters]

    # Mutable flag
    base_mutable = !isimmutable(baseT)
    println("Rabbit3")

    # ---- Construct expression ----------------------
    # Type statement
    res = :(struct $name{} <: $suptype end)
    # true==mutable, false==immutable
    res.args[1] = base_mutable
    println("Rabbit4")
    # Parameters
    push!(res.args[2].args[1].args, base_parameters...)
    # Fields
    push!(res.args[end].args, base_fields...)
    push!(res.args[end].args, fields.args...)
    println("Rabbit5 asdsad")
    println("$res")

    println(current_module())

    # Core.eval(res)
    Core.eval(current_module(), res)
    # Core.eval(Union{FLOWVLM}, res)
    println("Rabbit6")

    return nothing
end



# abstract type AbstractPerson{M, N} end
#
# struct Person{M, N} <: AbstractPerson{M, N}
#     name::M
#     age::N
# end
#
# @inherit Citizen Person AbstractPerson{Any, Any} begin
#     nationality::String
# end
