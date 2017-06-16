syms thetat lt thetac
L=190;
lc=30;
rt=400/(45/180*pi);
thetaini=30/180*pi;
thetac=thetaini+pi/2-thetat-lt/rt;
[0;0;-L]==rotx(-thetat)*[0;sin(lt/rt)*rt;-(rt-cos(lt/rt)*rt)]+rotx(-thetat)*rotx(-lt/rt)*[0;sin(thetac)*lc/thetac;-(lc/thetac-cos(thetac)*lc/thetac)]