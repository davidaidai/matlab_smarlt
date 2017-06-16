function cross_matrix=cross_matric(vec)
x=vec(1);y=vec(2);z=vec(3);
cross_matrix=[0 -z y;z 0 -x;-y x 0];
end