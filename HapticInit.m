function  Result=HapticInit()
%此函数是Omni设备的初始化函数，所有调用读取Omni矩阵值的函数之前必须调用此函数
%load dll文件，初始化两个Omni设备

cd 'D:\DensoWork\matlab_demo';
if not(libisloaded('PhantomA'))
    copyfile('D:\DensoWork\Denso_matlab\PhantomA\PhantomA.h','D:\DensoWork\matlab_demo\PhantomA.h');
    copyfile('D:\DensoWork\Denso_matlab\Release\PhantomA.dll','D:\DensoWork\matlab_demo\PhantomA.dll');
    loadlibrary('PhantomA.dll','PhantomA.h')
end
%查看dll中的所有函数列表
libfunctions PhantomA -full
%初始化Omni函数
Result=calllib('PhantomA','HapticInitA');
end

