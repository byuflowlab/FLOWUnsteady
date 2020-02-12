#=##############################################################################
# DESCRIPTION
    Noise analysis and processing

# AUTHORSHIP
  * Author    : Eduardo J. Alvarez
  * Email     : Edo.AlvarezR@gmail.com
  * Created   : Jan 2020
  * License   : MIT
=###############################################################################

"""
    Generates loading files for PSU-WOPWOP from the indicated simulation in
`read_path` using the steps `nums`. If `woptype=Constant`, only one step must
be specified; if `woptype=Periodic`, it is assumed that the specified steps make
one and only one full revolution; if `woptype=Aperiodic`, it is assumed that the
specified steps make one and only one full revolution. This function outputs the
load per unit length in the global coordinate system if periodic, or in the
blade coordinate system if constant.

NOTE: If periodic, there should be steps from 0 degrees up to 360 degrees (or up
to whether the periodic angle is) where `nums[1]` matches the same conditions
than `nums[end]` (rotor simulations with this module will do this by default).
The period of one revolution should be indicated under `period`.

NOTE2: `dt` must be the same time step than specified in the PSU-WOPWOP driver.
"""
function generate_wopwoploading(read_path, save_path, nums;
                                # INPUT OPTIONS
                                # nums=collect(0:72*6-1),
                                filename="loading_Sys1", fieldname="Ftot",
                                filenameout="singlerotor_Sys1",
                                # OUTPUT OPTIONS
                                # save_path=extdrive_path*"val_piv_single16_cont3_wopwop01/",
                                v_lvl=0, verbose=true, prompt=true,
                                wopext="wop", wopbin=true, wopv=1.0,
                                # PROCESSING OPTIONS
                                period=nothing,
                                t0=0.0, dt=nothing,
                                structured=false
                               )

    # 1==Constant, 2==Periodic, 3==Aperiodic
    if length(nums)==1
        Tflag = 1
        woptype = "constant"
    elseif period != nothing
        Tflag = 2
        woptype = "periodic"
    else
        Tflag = 3
        woptype = "aperiodic"
    end

    # ERROR CASES
    if Tflag != 1 && dt == nothing
        error("Received non-constant load without a time step dt.")
    end

    # Binary / ASCII printing
    prnt(f, x) = wopbin ? write(f, x) : print(f, x)
    prntln(f, x) = wopbin ? write(f, x) : print(f, x, "\n")

    # Convertion to 4-bytes numbers
    # NOTE: 4 bytes = 4*8 bites = 32 bites
    fl(x) = Float32(x)
    nt(x) = Int32(x)
    # Convertion to n-bytes string
    st(x::String, n) = x * " "^(n-length(x))

    f = []             # f[i][j] is the file of the j-th blade in the i-th rotor

    for (i,num) in enumerate(nums)                      # Iterate over steps

        # Loading of every rotor in this step
        ftot = JLD.load(joinpath(read_path, filename*".$(num).jld"), fieldname)

        for ri in 1:size(ftot, 1)                     # Iterate over rotors
            for bi in 1:size(ftot[ri], 1)             # Iterate over blades

                if i==1                               # Initialize files

                    if bi==1; push!(f, []); end;

                    # Create file
                    fname = filenameout*"_Rotor$(ri)_Blade$(bi)_loading_"*woptype*"."*wopext
                    push!(f[ri], open(joinpath(save_path, fname), "w"))

                    if wopv==1.0
                        # Magic number
                        prntln(f[ri][bi], nt(42))
                        # Version number
                        prnt(f[ri][bi], nt(1))
                        prntln(f[ri][bi], nt(0))
                        # Comments
                        prntln(f[ri][bi],
                               st("Compact patch loading data file for PSU-WOPWOP (Format v1.0)\n"*
                                  "------------------------------------------------\n"*
                                  "Created for noise-prop-on-prop project (written by Eduardo Alvarez)\n"*
                                  "https://github.com/byuflowlab/alvarezcritchfield2020-noise-prop-on-prop\n"*
                                  "Creation date: $(Dates.now())\n"*
                                  "Units: SI\n"*
                                  "Format: $(Tflag !=1 ? "Global" : "Blade") coordinate system, $(structured ? "" : "un")"*"structured", 1024))

                        # Format string
                        prntln(f[ri][bi], nt(2))               # Functional file flag
                        prntln(f[ri][bi], nt(1))               # Number of zones
                        prntln(f[ri][bi], nt(2^!structured))   # 1==structured, 2==unstructured
                        prntln(f[ri][bi], nt(Tflag))           # 1==Constant, 2==Periodic, 3==Aperiodic
                        prntln(f[ri][bi], nt(2))               # Data centered on 1==node, 2==face
                        prntln(f[ri][bi], nt(2))               # Data 1==pressure, 2==loading, 3==flow
                        if Tflag==1
                            # NOTE: If constant loading, patch-fixed frame is assumed
                            prntln(f[ri][bi], nt(3))           # Frame 1==stationary, 2==rotating, 3==patch-fixed
                        else
                            # NOTE: If non-constant loading, stationary ground frame is assumed
                            prntln(f[ri][bi], nt(1))
                        end
                        prntln(f[ri][bi], nt(1))               # Floating point 1==single, 2==double
                        prntln(f[ri][bi], nt(0))               # WOPWOP secret conspiracy
                        prntln(f[ri][bi], nt(0))               # WOPWOP other secret conspiracy

                        # Zone specificiation
                        prnt(f[ri][bi], nt(1))                 # Number of zones with data
                        prntln(f[ri][bi], nt(-1))              # Zone with data (negative for no thickness)

                        # Patch header
                        # Name
                        prntln(f[ri][bi], st("liftinglineloading", 32))

                        if Tflag == 2
                            # period
                            prntln(f[ri][bi], fl(period) )
                            # nKey
                            # prntln(f[ri][bi], nt(size(nums, 1) + 1) )
                            prntln(f[ri][bi], nt(size(nums, 1)) )

                        elseif Tflag == 3
                            # nTimes
                            prntln(f[ri][bi], nt(size(nums, 1)) )
                        end

                        if !structured
                            # NOTE: Here I assume face-centered data
                            # nbFaces
                            prntln(f[ri][bi], nt( size(ftot[ri][bi], 1) ))
                        else
                            # iMax
                            prntln(f[ri][bi], nt( 1 ))
                            # jMax
                            prntln(f[ri][bi], nt( size(ftot[ri][bi], 1) ))
                        end

                    else
                        error("Got invalid WOPWOP version $wopv")
                    end
                end

                # Keyi or Timei
                if Tflag != 1
                    prntln(f[ri][bi], fl(t0 + (i-1)*dt) )
                end


                # imax × jmax floating point x coordinates
                # imax × jmax floating point y coordinates
                # imax × jmax floating point z coordinates
                for k in 1:3
                    for j in 1:size(ftot[ri][bi], 1)
                        prntln(f[ri][bi], fl(ftot[ri][bi][j][k]) )
                    end
                end


                if i==size(nums, 1)


                    # # Repeat the first step if periodic
                    # if Tflag==2
                    #     ftot0 = JLD.load(joinpath(read_path,"gammas.$(nums[1]).jld"), "ftot")
                    #
                    #     # Keyi
                    #     prntln(f[ri][bi], fl(t0 + i*dt) )
                    #
                    #     # imax × jmax floating point x coordinates
                    #     # imax × jmax floating point y coordinates
                    #     # imax × jmax floating point z coordinates
                    #     for k in 1:3
                    #         for j in 1:size(ftot[ri][bi], 1)
                    #             prntln(f[ri][bi], fl(ftot0[ri][bi][j][k]) )
                    #         end
                    #     end
                    # end

                    close(f[ri][bi])                # Close files
                end

            end
        end

    end
end


"""
Generate lifting lines for acoustic compact patches of rotors
"""
function generate_vtkliftinglines(rotors, rotor_name, save_path;
                                                num=nothing, suf="_compact")

    for (ri, rotor) in enumerate(rotors)

        nHS = vlm.get_mBlade(rotor)

        for bi in 1:rotor.B

            blade = vlm.get_blade(rotor, bi)
            points = [vlm.getHorseshoe(blade, k)[2] for k in 1:nHS]
            push!(points, vlm.getHorseshoe(blade, nHS)[3])
            vtk_line = [i-1 for i in 1:nHS+1]

            gt.generateVTK(rotor_name*"_Rotor$(ri)_Blade$(bi)$(suf)", points;
                                lines=[vtk_line], num=num, path=save_path)

        end
    end

end
