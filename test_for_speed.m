%% first test
record_gamma_degree=zeros(4,1e5,1e2);
record_gamma_q_c1=zeros(8,1e5,1e2);
record_gamma_tube_para1=zeros(5,1e5,1e2);
record_gamma_p_t1=zeros(3,1e5,1e2);
record_gamma_R_t1=zeros(3,3,1e5,1e2);
tic 
for i_tube=1:1:100
for i_gamma=1:1:60000
record_gamma_degree(:,i_gamma,i_tube)=[1/pi*180;1/pi*180;1/pi*180;1];
record_gamma_q_c1(:,i_gamma,i_tube)=[1;2;3;4;5;6;7;8];
record_gamma_tube_para1(:,i_gamma,i_tube)=[1;2;3;4;5];
record_gamma_p_t1(:,i_gamma,i_tube)=[1;2;3];
record_gamma_R_t1(:,:,i_gamma,i_tube)=[1,2,3;1,2,3;1,2,3];
end
end
toc

%% second test
tic 
for i_tube=1:1:100
for i_gamma=1:1:60000
gamma_degree(:,i_gamma,i_tube)=[1/pi*180;1/pi*180;1/pi*180;1];
gamma_q_c1(:,i_gamma,i_tube)=[1;2;3;4;5;6;7;8];
gamma_tube_para1(:,i_gamma,i_tube)=[1;2;3;4;5];
gamma_R_t1(:,:,i_gamma,i_tube)=[1,2,3;1,2,3;1,2,3];
end
end
toc