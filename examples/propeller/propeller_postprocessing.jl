#=##############################################################################
# DESCRIPTION
    Postprocessing APC 10x7 simulation and comparison to experimental data
=###############################################################################

import Printf: @printf

# ------------- POST-PROCESSING ------------------------------------------------
println("\nPostprocessing...\n")

# Experimental thrust, torque, and efficiency (McCrink & Gregory, Figs. 16 and 20)
CTexp = 0.0923
CQexp = 0.00912
etaexp = 0.656

# Add experimental CT to monitor plot
ax = figaxs[1][4]
xlims = ax.get_xlim()
ylims = [0, 0.15]

ax.plot(xlims, CTexp*ones(2), ":k", label="Experimental", linewidth=1.5)

ax.set_xlim(xlims)
ax.set_ylim(ylims)
ax.set_xticks(0:1:nrevs)
ax.set_yticks(ylims[1]:0.05:ylims[2])
ax.legend(loc="best", frameon=false)

# Add experimental CQ to monitor plot
ax = figaxs[1][5]
ylims = [0, 0.015]

ax.plot(xlims, CQexp*ones(2), ":k", label="Experimental", linewidth=1.5)

ax.set_xlim(xlims)
ax.set_ylim(ylims)
ax.set_xticks(0:1:nrevs)
ax.set_yticks(ylims[1]:0.005:ylims[2])

# Add experimental efficiency to monitor plot
ax = figaxs[1][6]
ylims = [0, 1.0]

ax.plot(xlims, etaexp*ones(2), ":k", label="Experimental", linewidth=1.5)

ax.set_xlim(xlims)
ax.set_ylim(ylims)
ax.set_xticks(0:1:nrevs)
ax.set_yticks(ylims[1]:0.25:ylims[2])

# figs[1].tight_layout(pad=3.5)
