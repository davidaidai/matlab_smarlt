function  Result=HapticInit()
%�˺�����Omni�豸�ĳ�ʼ�����������е��ö�ȡOmni����ֵ�ĺ���֮ǰ������ô˺���
%load dll�ļ�����ʼ������Omni�豸

cd 'D:\DensoWork\matlab_demo';
if not(libisloaded('PhantomA'))
    copyfile('D:\DensoWork\Denso_matlab\PhantomA\PhantomA.h','D:\DensoWork\matlab_demo\PhantomA.h');
    copyfile('D:\DensoWork\Denso_matlab\Release\PhantomA.dll','D:\DensoWork\matlab_demo\PhantomA.dll');
    loadlibrary('PhantomA.dll','PhantomA.h')
end
%�鿴dll�е����к����б�
libfunctions PhantomA -full
%��ʼ��Omni����
Result=calllib('PhantomA','HapticInitA');
end

