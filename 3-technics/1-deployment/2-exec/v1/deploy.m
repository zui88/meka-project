%% Overwiev Steps
% Deployment is specified in roughly four steps
% 1.)    Set up jetson and the host system
% 2.)    Verify if all needed libraryies are installed on the Jetson
% 3.)    Prepare Matlab Code -> Create the main function for detection
% 4.)    Generate the Code for the Jetson
% 5.)    Run the application on Jetson (right now not pursued)

%% Step 1
%%% Host %%%
% check C++ tools -> Linux gcc and Windows Visual Studio
mex -setup

% external libraryies
setenv('CUDA_PATH',fullfile("C:","Program Files","NVIDIA GPU Computing Toolkit","CUDA","v10.0"))
setenv('NVIDIA_CUDNN',fullfile("C:","Program Files","NVIDIA GPU Computing Toolkit","CUDA","cuDNN"));
setenv('NVIDIA_TENSORRT', fullfile("C:","Program Files","NVIDIA GPU Computing Toolkit","CUDA","TensorRT"))
setenv('OPENCV_DIR',fullfile("C:","Program Files","opencv","build"));
setenv('PATH', ... 
    [fullfile('C:','Program Files','NVIDIA GPU Computing Toolkit','CUDA','v10.0','bin') ';' ...
    fullfile('C:','Program Files','NVIDIA GPU Computing Toolkit','CUDA','cuDNN','bin') ';'...
    fullfile('C:','Program Files','opencv','build','x64','vc15','bin') ';'...
    fullfile('C:','Program Files','NVIDIA GPU Computing Toolkit','CUDA','TensorRT','lib') ';'...
    getenv('PATH')]);

%%% Jetson Nano %%%
% This is specific to your jetson, use '$ ifconfig' to find the ipaddress
% or set a static ip
ipaddress = '192.168.55.1';
username = 'jetson';
password = '1111';
hwobj = jetson(ipaddress, username, password);

%%% MetaData
% start putty session
openShell(hwobj);
% get a list of available cameras
camlist = getCameraList(hwobj);

% check requriements on jetson, if connection is established
gpuEnvObj = coder.gpuEnvConfig('jetson');
gpuEnvObj.BasicCodegen = 1;
gpuEnvObj.BasicCodeexec = 1;
gpuEnvObj.DeepLibTarget = 'tensorrt'; % either tensorrt or cudnn
gpuEnvObj.DeepCodeexec = 1;
gpuEnvObj.DeepCodegen = 1;
gpuEnvObj.HardwareObject = hwobj;
results = coder.checkGpuInstall(gpuEnvObj);

%% Step 3
% adjust or create from scratch
edit detectFunction.m

%% Step 4
%Set up the coder configurations
cfg                     = coder.gpuConfig('exe');
cfg.Hardware            = coder.hardware('NVIDIA Jetson');
cfg.Hardware.BuildDir   = '~/yoloCNNForDetection';
%cfg.GenerateExampleMain = 'GenerateCodeAndCompile';
cfg.GenerateExampleMain = 'GenerateCodeOnly';
% Execute the code generator
codegen -report -config cfg detectFunction main.cu main.h

%% Step 5
%Use the Jetson object to launch the application on the device
hwobj.runApplication('detectFunction');
%Clean-up
hwobj.killApplication('detectFunction');
close all
clear
clc
