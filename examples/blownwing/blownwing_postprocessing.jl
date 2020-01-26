#=##############################################################################
# DESCRIPTION
    Simulation postprocessing

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Jan 2020
  * License   : MIT
=###############################################################################

"""
    Given the path of a simulation `read_path`, it will read the particle field
in time steps `nums` and it will calculate and output the velocity and vorticity
field in a prescribed fluid domain.
"""
function postprocessing_fluiddomain(read_path::String, nums, save_path::String;
                                    b=1.693, R=0.127, Vvehicle=23.384*[-1, 0, 0],
                                    dt=0.195516/2160, ndx=10,
                                    run_name="blownwing", Vinf=(x,t)->zeros(3),
                                    verbose=true, v_lvl=0, prompt=true)

    gt.create_path(save_path, prompt)

    # ------------------- Generate fluid domain grid ---------------------------
    if verbose; println("\t"^(v_lvl)*"Creating grid..."); end;
    lx = 1.00*b                         # Length in x-direction
    ly = 1.10*b                         # Length in y-direction
    lz = 2.35*R                         # Length in z-direction

    dx = R/ndx                          # Element size in x-direction
    dy = dx                             # Element size in y-direction
    dz = dx                             # Element size in z-direction

    P_min = [0.0, -ly/2, -lz/2*1.1]         # Minimum bounding point of grid
    P_max = P_min + [lx, ly, lz]        # Maximum bounding point of grid

    NDIVS = ceil.(Int, [lx/dx, ly/dy, lz/dz])   # Number of cells in each direction

    grid = gt.Grid(P_min, P_max, NDIVS)

    # ------------------- Evaluate fuild domain --------------------------------
    if verbose; println("\t"^(v_lvl)*"Evaluating fluid domain...\t$(grid.nnodes) nodes"); end;

    prevX = zeros(3)

    for (i, num) in enumerate(nums)
        if verbose; print("\t"^(v_lvl+1)*"Processing step $i of $(length(nums)):"); end;

        curX = (dt*num)*Vvehicle  # Current position of grid

        # Translate grid
        gt.lintransform!(grid, eye(3), curX-prevX)

        # Read particle field
        pfield = uns.vpm.read_vtk(run_name*"_pfield", Vinf, nothing; path=read_path,
                                                num=num, solver_method="ExaFMM")

        # Eliminates particles outside the domain to save computation
        xmax = gt.get_node(grid, NDIVS+1)[1]
        for p in uns.vpm.get_np(pfield):-1:1
            if uns.vpm.get_x(pfield, p)[1]>xmax
                uns.vpm.delparticle(pfield, p)
            end
        end

        if verbose; println("\t$(uns.vpm.get_np(pfield)) particles"); end;

        # Evaluate particle field velocity
        nodes = [gt.get_node(grid, n) for n in 1:grid.nnodes]
        @time _, U, W, dUdX = uns.vpm.conv(pfield, "ExaFMM"; Uprobes=nodes, vorticity=true)

        gt.add_field(grid, "U", "vector", U, "node"; raise_warn=false)
        gt.add_field(grid, "W", "vector", W, "node"; raise_warn=false)
        gt.save(grid, run_name*"_Grid"; path=save_path, num=num)

        prevX .= curX
    end

    return nothing
end
