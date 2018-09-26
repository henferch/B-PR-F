###########################################################################################
#
#   <This program loads the dataset from the simulation environment and provides it 
#   to the B-PR-F neural algorithm>
# 
#   Copyright (C) <2018>  <Hendry Ferreira Chame>
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, version 3.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <https://www.gnu.org/licenses/>.
#    
#   Programmed by: Hendry Ferreira Chame
#   Institution: Federal University of Rio Grande (FURG), Rio Grande, BRAZIL
#   E-mail: hendrychame@furg.br, hendryf@gmail.com
#     
#   Paper:  Neural network for black-box fusion of underwater robot localization under unmodeled noise
#   Authors: Hendry Ferreira Chame, Matheus Machado dos Santos, Sílvia Silva da Costa Botelho
#   Robotics and Autonomous Systems, Elsevier, 2018. doi: 10.1016/j.robot.2018.08.013
#   
#   E-mail: hendrychame@furg.br, hendryf@gmail.com
#
#   Description: This is the main program for executing the filter implemented
#
###########################################################################################


clear all;
close all;

display("#################################################################################################");
display("")
display("  <GNU Octave Implementation of B-PR-F>  Copyright (C) <2018>  <Hendry Ferreira Chame>         ");
display("  This program comes with ABSOLUTELY NO WARRANTY. This is free software, and you are welcome   "); 
display("  to redistribute it under certain conditions; for details consult the file 'LICENSE.txt'.     ");
display("")
display("  For more details about the framework, please consult the following reference:                "); 
display("")
display(" \"Neural network for black-box fusion of underwater robot localization under unmodeled noise\"");  
display("  Hendry Ferreira Chame, Matheus Machado dos Santos, Sílvia Silva da Costa Botelho             ");
display("  Robotics and Autonomous Systems, Elsevier, 2018. doi: 10.1016/j.robot.2018.08.013            ");
display("")
display("#################################################################################################");
display("")
display("Simulation main program start");
display("checking data ...");
fflush(stdout); 

#### Step One: Loading data 

display("Loading data ...");

load("data/dvlData.mat");
load("data/imuData.mat");
load("data/usblData.mat");
load("data/dgpsData.mat");
load("data/altimeterData.mat");
load("data/gtData.mat");

display("Data loaded!");
  
#### Step Two:loading the filter implementation

source("filterB-PR-F_simulator.m");
  
filterInfo();
fflush(stdout); 

nDvl = size(dvlData,1);
nImu = size(imuData,1);
nUsbl = size(usblData,1);
nDgps = size(dgpsData,1);
nAltimeter = size(altimeterData,1);

nGt = size(gtData,1);

meanDtDvl = mean(dvlData(2:end,1) - dvlData(1:end-1,1));
meanDtImu = mean(imuData(2:end,1) - imuData(1:end-1,1));
meanDtUsbl = mean(usblData(2:end,1) - usblData(1:end-1,1));
meanDtDgps = mean(dgpsData(2:end,1) - dgpsData(1:end-1,1));
meanDtAltimeter = mean(altimeterData(2:end,1) - altimeterData(1:end-1,1));
meanDtGt = mean(gtData(2:end,1) - gtData(1:end-1,1));

% adding non-Gaussian Noise to the USBL sensor
usblData(10,2:3) = [-150 -100];
usblData(20,2:3) = [-150 -100];
usblData(40,2:3) = [-150 -100];
usblData(60,2:3) = [-150 -100];
usblData(80,2:3) = [-150 -100];
usblData(100,2:3) = [-150 -100];
usblData(120,2:3) = [-150 -100];
usblData(140,2:3) = [-150 -100];  
  
times = imuData(:,1); # the time of the highest rate observation available is taken

  
iInitial = 1;
iFinal = nImu;
    
iDvl = 1;
iImu = 1;
iDgps = 1;
iUsbl = 1;
iAltimeter = 1;

tInitial = times(iInitial);

idx = find(dvlData(:, 1)>=tInitial);
iDvl = idx(1);

idx = find(imuData(:, 1)>=tInitial);
iImu = idx(1);

idx = find(usblData(:, 1)>=tInitial);
iUsbl = idx(1);

idx = find(dgpsData(:, 1)>=tInitial);
iDgps = idx(1);
  
idx = find(altimeterData(:, 1)>=tInitial);
iAltimeter = idx(1);

muIni = usblData(iUsbl,:);

tDvl = dvlData(iDvl,1);
tImu = imuData(iImu,1);
tUsbl = usblData(iUsbl,1);
tDgps = dgpsData(iDgps,1);
tAltimeter = altimeterData(iAltimeter,1);

thetaIni = 0;
  
filterInitialize(muIni', thetaIni, [meanDtImu, meanDtDvl, meanDtUsbl, meanDtDgps]');

#### Step Three: Simulation loop
c1 = time();
lastTime = times(iInitial);
for i=iInitial: iFinal

t = times(i);  
dt =   t - lastTime;
lastTime = t;

if (tDvl <= t)&&(iDvl < nDvl)

mesurement = dvlData(iDvl,:);    

filterReadDvl(mesurement');

iDvl = iDvl + 1;        
tDvl = mesurement(1);

endif


if (tImu <= t)&&(iImu < nImu)

mesurement = imuData(iImu,:);    

filterReadImu(mesurement');

iImu = iImu + 1;        
tImu = mesurement(1);

endif

if (tAltimeter <= t)&&(iAltimeter < nAltimeter)

mesurement = altimeterData(iAltimeter,:);    

filterReadAltimeter(mesurement');

iAltimeter = iAltimeter + 1;        
tAltimeter = mesurement(1);

endif

if (tUsbl <= t)&&(iUsbl < nUsbl)

mesurement = usblData(iUsbl,:);

filterReadUsbl(mesurement');   

iUsbl = iUsbl + 1;
tUsbl = mesurement(1);

endif

if (tDgps <= t)&&(iDgps < nDgps)

mesurement = dgpsData(iDgps,:);

filterReadDgps(mesurement');   

iDgps = iDgps + 1;
tDgps = mesurement(1);  

endif

filterProcessing(dt);

endfor
c2 = time();
#### Step Four: Plot generation

filterPlotData(gtData');
fflush(stdout); 

#### Step Five: Program termination
display(strcat("Total execution time: ", num2str(c2-c1)));
fflush(stdout); 

display("Main program end");
