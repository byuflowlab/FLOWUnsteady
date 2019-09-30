# APC Thin Electric 10x7 propeller as described in McCrink, M. H., & Gregory, J.
# W. (2017), *Blade Element Momentum Modeling of Low-Reynolds Electric
# Propulsion Systems*.
# Leading edge is my own digitization.

# This version uses the Clark Y and NASA 4412 airfoils as mentioned by McCrink.
# but with only 5 airfoils (speeds up simulation precomputation)

#= NOTE: Splie this as
spl_k = 4            # Spline order
spl_s = 5.0e-7     # Spline smoothing
=#

Rtip = 10*0.0254/2       # (m) Radius of blade tip
Rhub = 0.375*0.02542     # (m) Radius of hub
B = 2                    # Number of blades

# r/R c/R
chorddist = [0.0 0.134;
			0.086 0.137106;
			0.16 0.144606;
			0.2 0.154291;
			0.25 0.175;
			#0.3 0.19;
			#0.35 0.198;
			#0.4 0.202;
			#0.45 0.2;
			0.3 0.19+0.015;
			0.35 0.198+0.01;
			0.4 0.202+0.01;
			0.45 0.2+0.005;
			0.5 0.195;
			0.55 0.186;
			0.6 0.174;
			0.65 0.161;
			0.7 0.145;
			0.75 0.129;
			0.8 0.112;
			0.85 0.096;
			0.9 0.081;
			0.9245 0.071125;
			0.954 0.066125;
			#1.0 0.0233333]
			1.0 0.0375]

# r/R twist (deg)
pitchdist =[0.0 17.0;
			0.04715 22.0;
			0.088145 30.0;
			0.15 37.86;
			0.2 45.82;
			0.25 44.19;
			0.3 38.35;
			0.35 33.64;
			0.4 29.9;
			0.45 27.02;
			0.5 24.67;
			0.55 22.62;
			0.6 20.88;
			0.65 19.36;
			0.7 17.98;
			0.75 16.74;
			0.8 15.79;
			0.85 14.64;
			0.9 13.86;
			0.95 12.72;
			1.0 11.53]


# r/R y/R (y-distance of LE from the middle point of hub)
sweepdist = [0.0266906 -0.0675531;
			0.0993744 -0.0781078;
			0.16992 -0.0810668;
			0.209422 -0.0860351;
			0.260681 -0.0914966;
			0.310887 -0.0958101-0.01;
			0.352557 -0.0986618-0.01;
			0.39209 -0.101424-0.01;
			0.430601 -0.100831-0.01;
			0.473388 -0.100419-0.005;
			0.521551 -0.0980235-0.005;
			0.574004 -0.0947046-0.005;
			0.632903 -0.0894484-0.0025;
			0.69823 -0.083358;
			0.7518 -0.0767745;
			0.801094 -0.0700117;
			0.865384 -0.0616702+0.0025;
			#0.94041 -0.0504677+0.0025;
			0.96 -0.0504677+0.005;
			#0.973671 -0.0430323;
			#0.986802 -0.030395]
			0.98 -0.0430323;
			1.0 -0.02]

sweepdist[:, 2] *= -1

# r/R z/R  (height of leading edge from top face of hub)
heightdist = [0.0 -0.0150591;
			0.075 -0.003;
			0.12 0.016;
			0.2 0.044;
			0.4 0.024;
			0.6 0.00278494;
			0.8 -0.02;
			#0.95 -0.0388821;
			#1.0 -0.056
			0.95 -0.025;
			1.0 -0.035]

# Airfoil position with r=0 being the hub centerline
def_clcurve = "n4412-1500000.csv"
airfoil_files =   [ (Rhub/Rtip, "airfoils/clarky.csv", def_clcurve),
                    (0.400, "airfoils/naca4412.csv", def_clcurve),
                    (0.700, "airfoils/naca4412.csv", def_clcurve),
                    (0.900, "airfoils/naca4412.csv", def_clcurve),
                    (1.000, "airfoils/naca4412.csv", def_clcurve)]

# Correct airfoil positions to make r=0 the root
airfoil_files = [( (r-Rhub/Rtip)/(1-Rhub/Rtip) ,f,c) for (r,f,c) in airfoil_files]

# Airfoils along the blade as
# airfoil_contours=[ (pos1, contour1, polar1), (pos2, contour2, pol2), ...]
# with contour=(x,y) and pos the position from root to tip between 0 and 1.
# pos1 must equal 0 (root airfoil) and the last must be 1 (tip airfoil)
airfoil_contours = []
for (r, rfl_file, clcurve_file) in airfoil_files
    x,y = vlm.vtk.readcontour(rfl_file; delim=",", path=data_path, header_len=1)
    rfl = hcat(x,y)

    push!(airfoil_contours, (r, rfl, clcurve_file))
end
