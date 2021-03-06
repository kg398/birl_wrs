from math import pi

# home position, arbitrary
home = [90.0/1000,-500.0/1000,100.0/1000,0.0*pi/180,180.0*pi/180,0.0*pi/180]
homej = [87.61*pi/180,-87.40*pi/180,100.79*pi/180,-103.37*pi/180,-89.70*pi/180,-2.26*pi/180]

burt_home = [-0.111043, -0.290866, 0.65645, 5.10589e-05, 3.1412, -6.114650000000001e-05]
burt_homej = [-1.57255, -1.29794, -1.18718, -2.22769, 1.57521, -0.00545246]

urnie_home = [0.106656, -0.288412, 0.458283, -0.000117556, -3.14114, -0.000555322]
urnie_homej = [1.55372, -1.8502, 1.18642, -0.905986, -1.56306, -0.0165847]

# robot offset calibration
dual_rob_cal = [[-0.242463, -0.360618, 0.206584, 1.21812, -1.20251, -1.21928],   # burt pos-a
                [-0.342511, -0.510611, 0.256535, 1.21829, -1.20241, -1.21942],   # burt pos-b
                [-0.0425018, -0.460613, 0.306514, 1.21833, -1.20238, -1.21946],   # burt pos-c
                [0.557871, -0.359711, 0.20653, -0.00230235, 1.56974, 8.068289999999999e-05],   # urnie pos-a
                [0.457209, -0.510391, 0.254751, -0.00149064, 1.56984, 0.00129851],   # urnie pos-b
                [0.756917, -0.461719, 0.306706, -0.00651434, 1.56845, 0.0020755]]   # urnie pos-c

# gripper tool centre points (tcp)
rotary_tcp = [0.0015,0,0.1975,0,0,0]
pincher_tcp = [0.001,0,0.142,0,0,0]
suction_tcp = [0,0,0,0,0,0]
magnet_tcp = [0,0,0,0,0,0]
bluetack_tcp = [0,0,-0.215,0,0,0]

# gripper tool centre points for calibration
cal_rotary_tcp = [0.0015,0,0.2035,0,0,0]

# taskboard allen keys
tc_allen_m6 = [0.0170342, -0.699632, 0.0730104, -0.00145953, -3.1348, -0.00894802]
tc_allen_m4 = [-0.0218936, -0.702443, 0.0715422, -0.0169484, -3.13697, -0.00426133]
tc_allen_m3 = [-0.061991, -0.700791, 0.0739105, -0.0179077, -3.12842, -0.00270742]
tc_allen_m2 = [-0.102284, -0.703488, 0.0680846, -0.0271624, -3.13437, 0.000693775]

# taskboard urnie cal
tc_tl = [0.272457, -0.615973, 0.0412229, -0.0103236, -3.13346, -0.00597155]
tc_tr = [-0.127359, -0.62124, 0.0423387, -0.0181261, -3.1327, 0.000318126]
tc_br = [-0.1331, -0.22151, 0.0394859, 0.00983081, 3.12827, 0.0326293]

# assembly allen keys
ac_allen_m4_long = [0.28541, -0.605437, 0.140575, 0.000339453, -3.13851, -0.00584779]
ac_allen_m4_short = [0.363692, -0.603285, 0.0808225, 0.00187018, -3.13916, 0.00107713]
ac_allen_m3 = [0.443538, -0.600964, 0.0823504+0.017, 0.00142831, -3.1407, 0.00087902]
ac_allen_m2 = [0.523444, -0.601767, 0.0706781, 0.000959552, -3.13822, -0.00377845]

# assembly urnie cal
ac_tl = [0.461783, -0.440827, 0.0641026, -0.0133851, -3.13093, -0.00333733]
ac_tr = [0.342161, -0.441771, 0.0636883, -0.0225376, -3.13032, 0.00426378]
ac_br = [0.340832, -0.242211, 0.0627651, 0.00840569, 3.1323, 0.0327741]

# assembly kitting trays
ac_right_tray_tr = [0.0529578, -0.574669, 0.0244895, -0.0249155, -3.12729, 0.00265366]
ac_right_tray_br = [0.0517036, -0.294658, 0.0208683, 0.00710147, 3.12801, 0.0619638]
ac_right_tray_bl = [0.242463, -0.29383, 0.0205844, -0.011145, -3.12682, -0.0148994]
ac_left_tray_rel = [0.507673, 0.006554000000000004, 0.0021002999999999994, -0.013554, -3.12563, -0.0144431]

# taskboard two-handed waypoints
burt_passj = [-75.0*pi/180,-91.70*pi/180,-121.26*pi/180,-146.26*pi/180,15.17*pi/180,-1.58*pi/180]
urnie_passj = [102.36*pi/180,-84.11*pi/180,110.90*pi/180,-28.51*pi/180,12.86*pi/180,-88.25*pi/180]

# taskboard bracket waypoints
burt_bracketj = [-128.21*pi/180,-144.80*pi/180,-52.51*pi/180,-155.87*pi/180,52.05*pi/180,-4.46*pi/180]
urnie_bracketj = [161.18*pi/180,-92.60*pi/180,142.21*pi/180,-49.25*pi/180,161.76*pi/180,0.40*pi/180]

# assembly waypoints
urnie_pedestalj = [72.72*pi/180,-88.50*pi/180,110.77*pi/180,-20.84*pi/180,-16.83*pi/180,-91.47*pi/180]
burt_pedestalj = [-93.86*pi/180,-98.77*pi/180,-116.77*pi/180,-142.58*pi/180,-3.38*pi/180,-1.63*pi/180]
burt_pulleyj = [-117.02*pi/180,-128.79*pi/180,-56.44*pi/180,-115.96*pi/180,75.48*pi/180,-117.06*pi/180]
burt_beltj = [-137.33*pi/180,-134.75*pi/180,-29.40*pi/180,-147.70*pi/180,121.64*pi/180,-59.92*pi/180]
burt_motorj = [-77.75*pi/180,-108.75*pi/180,-115.35*pi/180,-140.74*pi/180,11.36*pi/180,5.82*pi/180]

# tcp with motor
motor_centroid = [0.001,0,0.1245,0,0,0]
             

