abstract type AbstractVehicle end

#--- Vortex Lattice Vehicle ---#

struct VLMVehicle{TF,Formulation,ViscousScheme,SubFilterScale,TKernel,TUJ,Tintegration}
    vlm_system::vlm.System{TF}
    dynamic_state_map::
    wake_system::vpm.ParticleField{TF,Formulation,ViscousScheme,SubFilterScale,TKernel,TUJ,Tintegration}
end

struct DynamicStateMap
    surface_indices::Vector{Int}
    substate_maps::Vector{DynamicStateMap}
end

@inline function vlm.rotate(panel::vlm.SurfacePanel, Q::Quaternion, r = (@SVector zeros(3)))

    rtl = rotate(panel.rtl - r, Q) + r
    rtc = rotate(panel.rtc - r, Q) + r
    rtr = rotate(panel.rtr - r, Q) + r
    rbl = rotate(panel.rbl - r, Q) + r
    rbc = rotate(panel.rbc - r, Q) + r
    rbr = rotate(panel.rbr - r, Q) + r
    rcp = rotate(panel.rcp - r, Q) + r
    ncp = rotate(panel.ncp, Q)
    core_size = panel.core_size
    chord = panel.chord

    return SurfacePanel(rtl, rtc, rtr, rbl, rbc, rbr, rcp, ncp, core_size, chord)
end

function convert!(vehicle::VLMVehicle, delta_state::RigidBodyDelta, state::RigidBodyState)
    dynamic_state_map = vehicle.dynamic_state_map
    for 
end

function convert!(system::vlm.System, state_map::DynamicStateMap, translation, center_of_rotation, rotation)
    # apply translation and rotation to the current state surfaces 
    for surface_index in state_map.surface_indices
        # translate position
        vlm.translate!(system.surfaces[surface_index], translation)

        # rotate position
        ## set center of rotation as the origin
        vlm.translate!(system.surfaces[surface_index], -center_of_rotation)

        ## rotate points

    end

    # recurse over substates
    for substate_map in state_map.substate_maps
        convert!(system, substate_map, translation, center_of_rotation, axis, angle)
    end
end

function convert!(system::vlm.System, delta_state::RigidBodyDelta, state::RigidBodyState, state_map::DynamicStateMap)
    
    # ensure substates are consistent
    @assert length(delta_state.substates) == length(state.substates) == length(state_map.substate_maps) "length of substates is not consistent: got length(delta_state.substates) = $(length(delta_state.substates)), length(state.substates) = $(length(state.substates)), length(state_map.substate_maps) = $(length(state_map.substate_maps))"
    
    # recurse this function over substates, noting that each is expressed in its parent frame, and
    # therefore implicitly inherits all of its parent's changes
    for (ds, s, sm) in zip(delta_state.substates, state.substates, state_map.substate_maps)
        convert!(system, ds, s, sm)
    end
    
    # convert dynamic state to the VLM system
    convert!(system, state_map, translation, center_of_rotation, axis, angle)
end

function solve!(vehicle::VLMVehicle)
    nothing  
end

