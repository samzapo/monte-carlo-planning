../../../build/Virtualization -j 8 --samples 100 --duration 0.3 --stepsize 0.001 \
  --parameters \
link-length.RF_FOOT,-0.01,0.01:\
link-length.RF_LEG_2,-0.01,0.01:\
link-length.LF_FOOT,-0.01,0.01:\
link-length.LF_LEG_2,-0.01,0.01:\
link-length.RH_FOOT,-0.01,0.01:\
link-length.RH_LEG_2,-0.01,0.01:\
link-length.LH_FOOT,-0.01,0.01:\
link-length.LH_LEG_2,-0.01,0.01:\
x,-0.01,0.01:\
y,-0.01,0.01\
 --executable ../../../build/sample.bin

mass.BODY0,-0.001,0.001\
#mass.RF_FOOT,-0.005,0.005:\
#mass.RF_LEG_2,-0.005,0.005:\
#mass.LF_FOOT,-0.005,0.005:\
#mass.LF_LEG_2,-0.005,0.005:\
#mass.RH_FOOT,-0.005,0.005:\
#mass.RH_LEG_2,-0.005,0.005:\
#mass.LH_FOOT,-0.005,0.005:\
#mass.LH_LEG_2,-0.005,0.005:\
