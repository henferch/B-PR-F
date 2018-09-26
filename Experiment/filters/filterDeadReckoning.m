###########################################################################################
#
#   <This program loads the experiment dataset and compares the implementation of the  
#   B-PR-F neural network to the Kalman Filter (KF), the Augmented Monte Carlo (A-MCL) 
#   algorithms>
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
#   Description: Implementation of dead reckoning localization.
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

ns = 3; % state dimension

### Kalman filter: theta estimate
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

global muOdom = zeros(1,ns); 

### data containers 

global allDeadReckoning = [];
global allUsbl = [];
global allDgps = [];
global allCompass  = [];

### Filter interface functions 

function filterInfo()
  display("filter Dead Reckoning loaded!");
  fflush(stdout);
endfunction
  
  
function filterInitialize(mu, theta)
    
  muOdom = [mu(2:3) 0]';
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
       
    global x1 F1 B1 P1 Q1 muOdom allDeadReckoning;   
    
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
    
    r(1) = a(1) = 1;     
    
    muOdom = [[muOdom(1:2) + dX] x1];
    allDeadReckoning = [allDeadReckoning; [tOdom muOdom]];
    
    
    
endfunction

function filterReadUsbl(measurement)

    global allUsbl;   
    allUsbl = [allUsbl ; measurement];         
    
endfunction

function filterReadDgps(measurement)

  global allDgps;  
  
  allDgps = [allDgps; measurement];
  
endfunction

function filterProcessing(t)
  % no neet to do anything
endfunction

function filterPlotData()

  global allDgps allDeadReckoning allUsbl;

  figure(1);
  title("Dead reckoning");    
  hold on;
  plot (allDgps(:,2),  allDgps(:,3), 'k',"LineWidth", 1);
  plot (allDeadReckoning(:,2),  allDeadReckoning(:,3), 'm',"LineWidth", 1);
  plot (allUsbl(:,2), allUsbl(:,3), '.b',"LineWidth", 1);
  axis("equal", "tight");
  print("-deps", "-color", "output/plots/filterDeadReckoning.eps");

endfunction

function filterRecordData()

  global allDgps allUsbl allDeadReckoning allCompass;

  save 'output/mat/filterDgps.mat' allDgps;
  save 'output/mat/filterUsbl.mat' allUsbl; 
  save 'output/mat/filterDeadReckoning.mat' allDeadReckoning;
  save 'output/mat/filterCompass.mat' allCompass; 

endfunction

