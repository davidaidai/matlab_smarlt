%% analyze max q&cq for one eps
for i=1:size(record_q_cq_dot(:,:,:),3)
    Cq=max(record_q_cq_dot(1:6,:,i));
    Ccq=max(record_q_cq_dot(7:8,:,i));
    [maxq(1,i),Iq(i)]=max(Cq);
    [maxcq(1,i),Icq(i)]=max(Ccq);
    maxq(2,i)=record_q_cq_dot(9,Iq(i),i);
    maxcq(2,i)=record_q_cq_dot(9,Icq(i),i);
    maxq(3,i)=record_Srt(6,Iq(i),i);
    maxcq(3,i)=record_Srt(6,Icq(i),i);
    maxq(4,i)=max(record_q_cq_dot(10,:,i));
    maxq(5,i)=max(record_q_cq_dot(11,:,i));
end

for ii=1:size(record_q_cq_dot(:,:,:),3)
    mm=0;
    for jj=1:size(record_q_cq_dot(:,:,ii),2)
        if record_Srt(6,jj,ii)>0.04 && record_Srt(6,jj,ii)<(0.05)
            mm=mm+1;
        end
    end
    maxq(6,ii)=maxq(4,ii)+mm;
end
%% analyze max q&cq for eps0.01-0.06
i_06=size(record_q_cq_dot_eps06(:,:,:),3);
i_0503=size(record_q_cq_dot_eps0503(:,:,:),3);
i_0201=size(record_q_cq_dot_eps0201(:,:,:),3);
for i=1:i_06
    Cq06=max(record_q_cq_dot_eps06(1:6,:,i));
    Ccq06=max(record_q_cq_dot_eps06(7:8,:,i));
    [maxq(1,i),Iq(i)]=max(Cq06);
    [maxcq(1,i),Icq(i)]=max(Ccq06);
    maxq(2,i)=record_q_cq_dot_eps06(9,Iq(i),i);
    maxcq(2,i)=record_q_cq_dot_eps06(9,Icq(i),i);
    maxq(3,i)=record_Srt_eps06(6,Iq(i),i);
    maxcq(3,i)=record_Srt_eps06(6,Icq(i),i);
    maxq(4,i)=max(record_q_cq_dot_eps06(10,:,i));
    maxq(5,i)=max(record_q_cq_dot_eps06(11,:,i));
end
for i=1:i_0503
    Cq0503=max(record_q_cq_dot_eps0503(1:6,:,i));
    Ccq0503=max(record_q_cq_dot_eps0503(7:8,:,i));
    [maxq(1,i+i_06),Iq(i+i_06)]=max(Cq0503);
    [maxcq(1,i+i_06),Icq(i+i_06)]=max(Ccq0503);
    maxq(2,i+i_06)=record_q_cq_dot_eps0503(9,Iq(i+i_06),i);
    maxcq(2,i+i_06)=record_q_cq_dot_eps0503(9,Icq(i+i_06),i);
    maxq(3,i+i_06)=record_Srt_eps0503(6,Iq(i+i_06),i);
    maxcq(3,i+i_06)=record_Srt_eps0503(6,Icq(i+i_06),i);
    maxq(4,i+i_06)=max(record_q_cq_dot_eps0503(10,:,i));
    maxq(5,i+i_06)=max(record_q_cq_dot_eps0503(11,:,i));    
end
for i=1:i_0201
    Cq0201=max(record_q_cq_dot_eps0201(1:6,:,i));
    Ccq0201=max(record_q_cq_dot_eps0201(7:8,:,i));
    [maxq(1,i+i_06+i_0503),Iq(i+i_06+i_0503)]=max(Cq0201);
    [maxcq(1,i+i_06+i_0503),Icq(i+i_06+i_0503)]=max(Ccq0201);
    maxq(2,i+i_06+i_0503)=record_q_cq_dot_eps0201(9,Iq(i+i_06+i_0503),i);
    maxcq(2,i+i_06+i_0503)=record_q_cq_dot_eps0201(9,Icq(i+i_06+i_0503),i);
    maxq(3,i+i_06+i_0503)=record_Srt_eps0201(6,Iq(i+i_06+i_0503),i);
    maxcq(3,i+i_06+i_0503)=record_Srt_eps0201(6,Icq(i+i_06+i_0503),i);
    maxq(4,i+i_06+i_0503)=max(record_q_cq_dot_eps0201(10,:,i));
    maxq(5,i+i_06+i_0503)=max(record_q_cq_dot_eps0201(11,:,i));  
end
%% find num of exceed normal velocity in near-singularity region 
i_06=size(record_q_cq_dot_eps06(:,:,:),3);
i_0503=size(record_q_cq_dot_eps0503(:,:,:),3);
i_0201=size(record_q_cq_dot_eps0201(:,:,:),3);
j_06=size(record_q_cq_dot_eps06(:,:,:),2);
j_0503=size(record_q_cq_dot_eps0503(:,:,:),2);
j_0201=size(record_q_cq_dot_eps0201(:,:,:),2);

for ii=1:i_06
    mm=0;
    for jj=1:j_06
        if record_Srt_eps06(6,jj,ii)>0.06 && record_Srt_eps06(6,jj,ii)<(0.06+1e-3)
            mm=mm+1;
        end
    end
    maxq(6,ii)=maxq(4,ii)+mm;
end
for ii=1:5
    mm=0;
    for jj=1:j_0503
        if record_Srt_eps0503(6,jj,ii)>0.05 && record_Srt_eps0503(6,jj,ii)<0.06
            mm=mm+1;
        end
    end
    maxq(6,ii+i_06)=maxq(4,ii+i_06)+mm;
end
for ii=1+5:5+5
    mm=0;
    for jj=1:j_0503
        if record_Srt_eps0503(6,jj,ii)>0.04 && record_Srt_eps0503(6,jj,ii)<0.06
            mm=mm+1;
        end
    end
    maxq(6,ii+i_06)=maxq(4,ii+i_06)+mm;
end
for ii=1+5+5:i_0503
    mm=0;
    for jj=1:j_0503
        if record_Srt_eps0503(6,jj,ii)>0.03 && record_Srt_eps0503(6,jj,ii)<0.06
            mm=mm+1;
        end
    end
    maxq(6,ii+i_06)=maxq(4,ii+i_06)+mm;
end
for ii=1:5
    mm=0;
    for jj=1:j_0201
        if record_Srt_eps0201(6,jj,ii)>0.02 && record_Srt_eps0201(6,jj,ii)<0.06
            mm=mm+1;
        end
    end
    maxq(6,ii+i_06+i_0503)=maxq(4,ii+i_06+i_0503)+mm;
end
for ii=1+5:i_0201
    mm=0;
    for jj=1:j_0201
        if record_Srt_eps0201(6,jj,ii)>0.01 && record_Srt_eps0201(6,jj,ii)<0.06
            mm=mm+1;
        end
    end
    maxq(6,ii+i_06+i_0503)=maxq(4,ii+i_06+i_0503)+mm;
end
% %% analyze max q&cq in singularity condition
% for ii=1:size(record_q_cq_dot_eps06(:,:,:),3)
%     mm=0;
%     for jj=1:size(record_q_cq_dot_eps06(:,:,ii),2)
%         if record_q_cq_dot_eps06(9,jj,ii)==1
%             mm=mm+1;
%             record_sgl_q_cq_dot(:,mm,ii)=record_q_cq_dot_eps06(1:10,jj,ii);
%         end
%     end
% end
% for i_sgl=1:size(record_sgl_q_cq_dot(:,:,:),3)
%     Cq_sgl=max(record_sgl_q_cq_dot(1:6,:,i_sgl));
%     Ccq_sgl=max(record_sgl_q_cq_dot(7:8,:,i_sgl));
%     [maxq_sgl(1,i_sgl),Iq_sgl(i_sgl)]=max(Cq_sgl);
%     [maxcq_sgl(1,i_sgl),Icq_sgl(i_sgl)]=max(Ccq_sgl);
%     maxq_sgl(2,i_sgl)=record_sgl_q_cq_dot(9,Iq_sgl(i_sgl),i_sgl);
%     maxcq_sgl(2,i_sgl)=record_sgl_q_cq_dot(9,Icq_sgl(i_sgl),i_sgl);
% end