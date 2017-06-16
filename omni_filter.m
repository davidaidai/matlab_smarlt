





[omni_data,button]=getomni();
angle_axis=conv_ar(omni_data);
ar_store=ones(5,1)*angle_axis;
Omni_command=conv_back(ar_store);
[p_t,R_t,button]=cal_target(Omni_command,button);