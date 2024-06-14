function extract_history(h::History, fieldname, dim, nsurface)
    # Recast fieldname as an array if it's a single field
    fieldname = typeof(fieldname) == Symbol ? [fieldname] : fieldname

    # Compute magnitude of 3d vectors
    mags = []
    for field in fieldname
        vals = h.history[field]

        if ndims(vals) == 3
            if dim == 0
                push!(mags, sqrt.(sum(abs2, view(vals,1:3,nsurface,:), dims=1)))
            else
                push!(mags, vals[dim, nsurface, :]')
            end

        elseif ndims(vals) == 2
            if dim == 0
                push!(mags, sqrt.(sum(abs2, vals, dims=1)))
            else
                push!(mags, vals[dim, :]')
            end

        else
            push!(mags, vals)
        end
    end

    field_values = hcat(mags'...)

    return fieldname, field_values
end

@recipe function f(sim::Simulation, fieldname=[:vehicle_force]; dim=0, nsurface=1, nstep=0)
    # Find History inside postprocessors
    i_hist = 0
    for i = 1:length(sim.postprocessor.postprocessors)
        if sim.postprocessor.postprocessors[i] isa History
            i_hist = i
            return
        end
    end
    if i_hist == 0
        error("History not found in simulation")
    end

    h = sim.postprocessor.postprocessors[i_hist]

    # Extract data
    fieldname, field_values = extract_history(h, fieldname, dim, nsurface)

    # Plot attributes
    xlabel --> "t"
    # ylabel --> "magnitude"

    field_labels = hcat(String.(fieldname)...)
    labels --> field_labels

    return h.time_range, field_values
end

@recipe function f(h::History, fieldname=[:vehicle_force]; dim=0, nsurface=1, nstep=0)
    # Extract data
    fieldname, field_values = extract_history(h, fieldname, dim, nsurface)

    # Plot attributes
    xlabel --> "t"
    # ylabel --> "magnitude"

    field_labels = hcat(String.(fieldname)...)
    labels --> field_labels

    return h.time_range, field_values
end

@recipe function f(run_name::String, fieldname=[:vehicle_force]; dim=0,
        path="", nsurface=1, nstep=0)

    # Read BSON file from path
    filename = joinpath(path, run_name * "_history.bson")
    data = BSON.load(filename)
    h = data[:history]

    # Extract data
    fieldname, field_values = extract_history(h, fieldname, dim, nsurface)

    # Plot attributes
    xlabel --> "t"
    # ylabel --> "magnitude"

    field_labels = hcat(String.(fieldname)...)
    labels --> field_labels

    return h.time_range, field_values
end
