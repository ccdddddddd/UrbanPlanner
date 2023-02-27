gcc UrbanPlanner.c rt_nonfinite.c rtGetInf.c rtGetNaN.c UrbanPlanner_emxutil.c -fPIC -shared -o UrbanPlanner.so
g++ Load.cpp -ldl -o Load
./Load


