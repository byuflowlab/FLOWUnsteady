#=##############################################################################
# DESCRIPTION
    Postprocessing swept wing simulation and comparison to experimental data
=###############################################################################

import Printf: @printf

# ------------- POST-PROCESSING ------------------------------------------------
println("\nPostprocessing...\n")

# ------- Vehicle lift and drag
# Total lift and drag
L = sum(wing.sol["L"])
D = sum(wing.sol["D"])

# Lift and drag coefficients
CL = uns.norm(L) / (qinf*b^2/ar)
CD = uns.norm(D) / (qinf*b^2/ar)

# Experimental lift and drag (Weber 1951, Table 4)
CLexp = 0.238
CDexp = 0.005

# Error
CLerr = abs(CLexp-CL)/CLexp
CDerr = abs(CDexp-CD)/CDexp

@printf "%0s%10s\t%-11s\t%-11s\t%7s\n"    "\t" "PARAMETER"   "Experimental"  "  FLOWUnsteady"    "Error"
@printf "%0s%10s\t%11.4f\t%11.5f\t%7.2f ﹪\n" "\t" "CL"          CLexp           CL              100*CLerr
@printf "%0s%10s\t%11.4f\t%11.5f\t%7.2f ﹪\n" "\t" "CD"          CDexp           CD              100*CDerr


# Add experimental CL to monitor plot
ax = figaxs[1][3]
xlims = [0, ttot]
ylims = [0, 0.3]

ax.plot(xlims, CLexp*ones(2), ":k", label="Experimental", linewidth=1.5)

ax.set_xlim(xlims)
ax.set_ylim(ylims)
ax.set_yticks(ylims[1]:0.1:ylims[2])
ax.legend(loc="best", frameon=false)

# Add experimental CD to monitor plot
ax = figaxs[1][4]
ylims = [0.0035, 0.0060]

ax.plot(ax.get_xlim(), CDexp*ones(2), ":k", label="Experimental", linewidth=1.5)

ax.set_xlim(xlims)
ax.set_ylim(ylims)
ax.set_yticks(ylims[1]:0.0005:ylims[2])


# ------- Load distribution

# Weber's experimental loading distribution from Table 3
alphas_exp = [2.1, 4.2, 6.3, 8.4, 10.5]
y2b_exp = [0.0, 0.041, 0.082, 0.163, 0.245, 0.367, 0.510, 0.653, 0.898, 0.949]
cls_exp = [
            0.118 0.121 0.126 0.129 0.129 0.129 0.131 0.125 NaN 0.087;      # AOA = 2.1deg
            0.235 0.241 0.248 0.253 0.251 0.251 0.251 0.246 0.192 0.171;    # AOA = 4.2deg
            0.351 0.358 0.367 0.374 0.375 0.373 0.377 0.365 NaN 0.256;      # AOA = 6.3deg
            0.466 0.476 0.483 0.494 0.494 0.493 0.493 0.48 NaN 0.34;        # AOA = 8.4deg
            0.577 0.589 0.597 0.607 0.611 0.605 0.599 0.587 0.415 0.401     # AOA = 10.5deg
         ]
cds_exp = [
            0.044 0.014 0.007 0.002 0.0 -0.001 0.0 -0.001 -0.009 -0.01;     # AOA = 0deg
            0.047 0.016 0.01 0.004 0.002 0.001 0.002 0.001 NaN -0.009;
            0.059 0.025 0.016 0.009 0.007 0.006 0.006 0.004 -0.002 -0.007;
            0.078 0.039 0.028 0.017 0.015 0.012 0.012 0.009 NaN -0.005;
            0.104 0.058 0.045 0.029 0.026 0.023 0.022 0.016 NaN -0.001;
            0.138 0.084 0.065 0.044 0.041 0.037 0.035 0.026 0.009 0.004
         ]

# Add the zero-AOA drag that they substracted in the experiment
cds_exp = mapslices(x-> x .+ cds_exp[1, :], cds_exp[2:end, :]; dims=2)

# Integrated coefficients from Table 4B
CLs_exp = [0.121, 0.238, 0.350, 0.456, 0.559]
CDs_exp = [nothing, 0.005, 0.012, 0.022, 0.035]

# Add load distributions to monitor plots
for (ax, vals_exp) in [(figaxs[1][1], cls_exp), (figaxs[1][2], cds_exp)]
    for f in [-1, 1]
        ax.plot(f*y2b_exp, vals_exp[2, :], "o--k", label="Experimental"^(f==1),
                                    linewidth=0.75, alpha=1.0, markersize=6.0)
    end
end

# Format sectional lift plot
ax = figaxs[1][1]
xlims = [-1, 1]
ylims = [0, 0.3]

ax.set_xlim(xlims)
ax.set_xticks(xlims[1]:0.5:xlims[2])
ax.set_ylim(ylims)
ax.set_yticks(ylims[1]:0.1:ylims[2])


# Format sectional drag plot
ax = figaxs[1][2]
ylims = [-0.02, 0.06]

ax.set_xlim(xlims)
ax.set_xticks(xlims[1]:0.5:xlims[2])
ax.set_ylim(ylims)
ax.set_yticks(ylims[1]:0.02:ylims[2])
ax.legend(loc="best", frameon=false)


# Save figure
figs[1].tight_layout(pad=3.5)
