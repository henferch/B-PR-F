###########################################################################################
#
#   <This program implements the Augmented Monte Carlo (A-MCL) for the experimental 
#   localization task with the ROV robot>
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
#   Description: Implementation of the algorithm 'Augmented-MCL' as defined in Thrun et al. 2005.
#              -------------------------------------------------------------------------
#              Filters are assumed to implement the following methods:
#
#              filterInfo() ................ Displays the filter name by console 
#              filterInitialize( data ) .... Initializes the filter parameters
#              filterReadCompass( data ) ... Input compass data to the filter
#              filterReadOdometry( data ) .. Input odometry data to the filter   
#              filterReadUsbl( data ) ...... Input USBL data to the filter   
#              filterReadDgps( data ) ...... Input DGPS data to the filter   
#              filterProcessing( time ) .... Filter's loop processing 
#              filterPlotData() ............ Filter's plot generation 
#              filterRecordData() .......... Filter's data saving
#
#   Requirement: This program must be loaded within the 'main.m' program
#                       
###########################################################################################

global ns = 2; % state dimension

### A_MCL parameters

global wSlow = 0.01;
global wFast = 1.0;
global alphaSlow = 0.001;
global alphaFast = 1.0;

global M = 1000; % particle number

global sigmasZ = [0.3 0.3]'; % observation sig

global sigmaLocalSearch = [2.0 2.0]'; % local particule generation
global sigmasN = [0.05 0.05]'; % particle sampling error

global X = zeros(ns,M);   % particle stqte representqtion
global Xhat = zeros(ns,1);   % global state estimation

global w = ones(1,M)*1/M; % particle weights
global wNorm = ones(1,M)*1/M; % normalized particle weights

### Kalman filter 1: theta estimate
global ns1 = 1; % theta

sigmaCompass = [0.02]*(pi/180); % deg
sigmaThetaODOM = [0.7]*(pi/180); % deg

global R1 = eye(ns1)*sigmaCompass; %; observation noise covariance 
global Q1 = eye(ns1)*sigmaThetaODOM; %; process noise covariance 

global F1 = eye(ns1); % transition state model
global B1 = eye(ns1); % control input model
global H1 = eye(ns1); % observation model
global P1 = R1;

global x1 = 0;

global newMeasurement = false;


### A-MCL functions

function [_x] = sampleMotionModel(_u, _X)
   _x = _X .+ _u;
endfunction  

function [_w] = measurementModel(_z, _X, _sig)
  a = 1;
  N = a * exp(-((_X.-_z).^ 2) ./ (2*(_sig.^2)));
  _w = prod(N, 1);
endfunction 

### Data containers

global allUsbl = [];
global allDgps = [];
global allA_MCL = [];
global allCompass  = [];


### Filter interface functions 

function filterInfo()
  display("A-MCL filter loaded!");
  fflush(stdout);
endfunction
  
  
function filterInitialize(mu, theta)

  global X sigmasZ ns M;

  X = ones(ns,M).*mu(2:3)' + randn(ns,M).*sigmasZ;
  x1 = theta;

endfunction


function filterReadCompass(measurement)

  global H1 P1 R1 x1 ns1 allCompass;   
  
    % correction step filter 1
    theta = measurement(2);
    z1 = theta;
    y1 = z1 - H1*x1;
    S1 = H1*P1*H1' + R1;
    K1 = P1*H1'*inv(S1);
    x1 = x1 + K1*y1;
    P1 = (eye(ns1)-K1*H1)*P1;
    allCompass = [allCompass ; measurement];

endfunction

function filterReadOdometry(measurement)
       
    global x1 F1 B1 P1 Q1 ;
    global X M wNorm allA_MCL;   
    
    tOdom = measurement(1);
    mes = measurement(2:3)';    
    dTheta  = measurement(4);
    
    angle = wrapTo2Pi(x1);
    bTr = [cos(angle) -sin(angle); sin(angle) cos(angle)];           
    frameAngle = pi/2;
    rTs = [cos(frameAngle) -sin(frameAngle); sin(frameAngle) cos(frameAngle)];
    wTb = [1 0; 0 -1];
    dX = (wTb*bTr*rTs*mes)';
       
    %prediction step filter 1
    u1 = [dTheta];
    x1 = F1*x1 + B1*u1;
    P1 = F1*P1*F1' + Q1; % predicted covariance    
  
    %prediction step filter 2
    X = sampleMotionModel(dX', X);                          
    Xhat =  sum(X.*wNorm,2);
    
    allA_MCL = [allA_MCL; [tOdom Xhat']];
    
endfunction

function filterReadUsbl(measurement)

    global allUsbl w X sigmasZ newMeasurement;   
    
    muUSBL = measurement(2:3);
    
    newMeasurement = true;
  
    w = measurementModel(muUSBL', X, sigmasZ);
               
    allUsbl = [allUsbl ; measurement];
    newMeasurement = true;
        
endfunction


function filterProcessing(t)
  
    global newMeasurement M X Xhat w wSlow alphaSlow wFast alphaFast;
    global ns sigmasN sigmaLocalSearch wNorm allAvg allA_MCL;
    
    if (newMeasurement ==  true)           
    
      wNorm = ones(1,M)*1/M; 
      div = sum(w,2);
      if (div > 1.0e-5) % consider probs if they are high enough, othewise use uniform dist to avoid underflows
        wNorm = w/div;
      endif      
    
      Xhat =  sum(X.*wNorm,2);  
    
      wAvg = (1/M)*sum(w);
      allAvg = [allAvg wAvg];

      wSlow = wSlow + alphaSlow*(wAvg - wSlow);
      wFast = wFast + alphaFast*(wAvg - wFast);  

      % sampling
      newX =  X.*0;
     
      [vals idxSort] = sortrows(w,1); 
      sumProb = cumsum(vals,2);

      vals(:) = 1/M; % uniform distribution
      vals = cumsum(vals);

      div = sumProb(end);      
      if (div > 1.0e-5) % Probabilities are big enough as to be considered
        vals = sumProb/div;        
      endif

      pRandSel = max(1.0 - wFast/wSlow , 0.0);
      sampledPose = zeros(ns,1);
      
      for m = 1 : M
        r = rand(1);      
        if(r <= pRandSel)
          rPose = rand(ns,1)-0.5;
          sampledPose= Xhat + randn(ns,1).*sigmaLocalSearch;
        else
          sel = find(vals >= r);
          selIdx = max(sel(1)-1, 1);
          rnoise = randn(ns, 1).*sigmasN;
          sampledPose = X(:,selIdx) + rnoise;    
        endif
        
        newX(:,m) = sampledPose;  
        
      endfor
      
      X = newX;    
      newMeasurement= false;
      
    endif 

endfunction

function filterReadDgps(measurement)

  global allDgps;  
  
  allDgps = [allDgps; measurement];
  
endfunction

function filterPlotData()

  global allDgps allA_MCL allUsbl;

  figure(1)
  hold on;
  title("Aumented-MCL");    
  plot (allDgps(:,2),  allDgps(:,3), 'k',"LineWidth", 1);
  plot (allA_MCL(:,2),  allA_MCL(:,3), 'm',"LineWidth", 1);
  plot (allUsbl(:,2), allUsbl(:,3), '.b',"LineWidth", 1);
  axis("equal", "tight");
  print("-deps", "-color", "output/plots/filterA_MCL.eps");

endfunction

function filterRecordData()

  global allA_MCL;

  save 'output/mat/filterA_MCL.mat' allA_MCL;

endfunction




