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
#   Description: Implementation of the B-PR-F neural network.
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


### ARF neural network parameters

global ns = 2; % state dimension
global nb = 1; % Behaviour profiles number
global ne = 2; % Estimator processes available 

global Wbp = zeros(ne*nb,nb);      
global Wpr = zeros(ne*nb,ne*nb);      
global Wrf = zeros(ne,ne*nb);  
global Wpp = zeros(ne,ne*nb);
global Wneigh = [0.2 0.8]
global mapRtoNodes = zeros(ne*nb,1);  

global b = zeros(nb,1);   % Layer B (Behavour)
global p = ones(ne*nb,1); % Layer P (Prediction)
global r = zeros(ne*nb,1); % Layer R (Reliability)
global f = ones(ne,1);    % Layer R (Fusion)
global active = zeros(ne,1); % active nodes vector

global phi1 = 0.2; % Reliability threshold
global phi2 = 13; % steepness of the sigmoid 

global mu = zeros(ns,1); % global mu

global pDts = zeros(nb*ne,1);
global tEstimators = zeros(1,ne); % processes internal clock

global SIGMA_E1_Init = 8*eye(ns); % Dead Reckoning
global SIGMA_E2_Init = 0.2*eye(ns); % KF

global SIGMAS_Init = [SIGMA_E1_Init SIGMA_E2_Init];
global SIGMAS = SIGMAS_Init;

global clb = zeros(ne*nb,1);
global mus = zeros(ns,ne);
global ft = 0 ; % filter time


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

### Kalman filter 2: 2D pose estimate
global ns2 = 3; % x y theta

sigmaUSBL = [0.1 0.1 sigmaCompass]; %[m  deg] [distance, bearing]
sigmaODOM = [0.5 0.5 sigmaThetaODOM]; % m/sec

global R2 = eye(ns2).*sigmaUSBL; %; observation noise covariance 
global Q2 = eye(ns2).*sigmaODOM; %; process noise covariance 
global H2 = eye(ns2); % observation model
global P2 = R2;

global F2 = eye(ns2); % transition state model
F2 = [1 0 0; 0 1 0; 0 0 0]; % transition state model
global B2 = eye(ns2); % control input model
global x2 = zeros(ns2,1);

### Data containers

global allB_PR_F = [];
global allUsbl = [];
global allDgps = [];
global allCompass  = [];

% Additional data structure for plotting
global allB_PR_F_layerB = [];
global allB_PR_F_layerP = [];
global allB_PR_F_layerR = [];
global allB_PR_F_layerF = [];
global allB_PR_F_Sigmas = [];

### ARF Neural network functions

function [result] = gamma(_sigma, _SIGMA, _x, _phi)
  
  [V lambda] = eig(_SIGMA);
  G = _sigma * lambda;
  beta = _x' * V' * inv(G) * V * _x; 
  result = (1 / (1 + exp(-_phi.*(_sigma - beta))));
 
endfunction

# Calculates Psy( ) conforming to Equation (8)
function [mu, sigma, SIGMA, dt] = Psy(_leftBrotherId, _mus, _sigmas, _SIGMAS, _tEstimators)
 
 global ns;
 
 sigma = _sigmas(_leftBrotherId);
 SIGMA = _SIGMAS(1:ns,ns*(_leftBrotherId-1)+1:ns*_leftBrotherId);
 mu = _mus(_leftBrotherId,:);
 dt = _tEstimators(_leftBrotherId);
     
endfunction 

### Filter interface functions 

function filterInfo()
  display("B-PR-F Neural Network filter loaded!");
  fflush(stdout); 
endfunction
  
  
function filterInitialize(_mu, _theta, _estimatorDt)
  
  global  x1 x2 ne ns mu mus nb ne;
  global Wneigh Wbp Wpr Wrf mapRtoNodes clb Wpp;
  global pDts;
  
  mu = _mu;
  x2 = [_mu(2:3) 0]';
  x1 = _theta;
  mus = ones(ne,ns).*_mu(2:3);  
    
  % initializing the network
  Wpr = ones(ne*nb,ne*nb).*-1.0e10;      

  maxf = max(Wneigh, [], 2);
  minf = min(Wneigh, [], 2);

  for b = 1 : nb
      W = zeros(ne,ne);
      for i = 1: ne      
        pivot = Wneigh(b,i);  
        for j = 1: ne
            if i == j
              W(i,j)  =  0;
            else
              diff = pivot - Wneigh(b,j);
              W(i,j)  =  1/diff;      
            endif
          endfor
      endfor
    
    bi = (b-1)*ne+1;
    bf = b*ne;
    Wpr(bi:bf,bi:bf)= W;
    
    
    Wbp(bi:bf,b) = ones(ne,1);      
    Wrf(1:ne,bi:bf) = diag(Wneigh(b,:));    
    
  endfor 
  
  mapRtoNodes =  repmat([1:ne]',nb,1); # mapping "nb*ne" space to "ne" space
  
  crb = zeros(ne*nb,1); # right neighborhood
  [ v crb] = max(Wpr);
  crb = crb'; 
  
  clb = zeros(ne*nb,1); # left neighborhood
  [ v clb] = max(Wpr');
  clb = clb';
  
  %estimators time constant
  _estimatorDt(2) = _estimatorDt(2)*10; % USBL heuristics
  eDts = repmat(_estimatorDt,nb,1);
  
  for i = 1 : nb*ne
    pDts(i) = eDts(crb(i));
  endfor
  
  pDts = pDts.^-1;
  
  % P recurrent weights
  Wpp = eye(ne);
  
endfunction

function updateEstimator(_ft, _dX, _estimatorId)
       
    global mus tEstimators SIGMAS;
    global ns active;
    
    active(_estimatorId) = 1;
   
    mus(:,_estimatorId) = mus(:,_estimatorId) + _dX;
           
    noiseE = abs(_dX).*eye(ns); % noise added from motion
    siODOMc = SIGMAS(1:ns,ns*(_estimatorId-1)+1:ns*_estimatorId);              
    SIGMAS(1:ns,ns*(_estimatorId-1)+1:ns*_estimatorId) = siODOMc + noiseE;
     
    tEstimators(_estimatorId) = _ft;
        
endfunction

function filterReadCompass(_measurement)

  global H1 P1 R1 x1 ns1 allCompass;   
    %correctoin step KF 1 (yaw orientation estimation)
    theta = _measurement(2);
    z1 = theta;
    y1 = z1 - H1*x1;
    S1 = H1*P1*H1' + R1;
    K1 = P1*H1'*inv(S1);
    x1 = x1 + K1*y1;
    P1 = (eye(ns1)-K1*H1)*P1;
    allCompass = [allCompass ; _measurement];

endfunction

function filterReadOdometry(_measurement)
       
    global x1 F1 B1 P1 Q1 F2 P2 B2 P2 Q2 x2;   
        
    estimatorId = 1;
    ft = _measurement(1);
    fmes = _measurement(2:3)';    
    dTheta  = _measurement(4);
    
    angle = wrapTo2Pi(x1);
    bTr = [cos(angle) -sin(angle); sin(angle) cos(angle)];           
    frameAngle = pi/2;
    rTs = [cos(frameAngle) -sin(frameAngle); sin(frameAngle) cos(frameAngle)];
    wTb = [1 0; 0 -1];
    dX = (wTb*bTr*rTs*fmes);
    
    updateEstimator(ft, dX, estimatorId);
    
    % prediction step KF 1 (yaw orientation estimation)
    u1 = [dTheta];
    x1 = F1*x1 + B1*u1;
    P1 = F1*P1*F1' + Q1; % predicted covariance    

    % prediction step KF 2 (2D position estimation)
    u2 = [dX; x1];        
    x2 = F2*x2 + B2*u2;
    P2 = F2*P2*F2' + Q2; % predicted covariance
    
    
endfunction

function filterReadUsbl(_measurement)

    global H2 P2 R2 x1 x2 ns2 allUsbl mus;   
    
    ft = _measurement(1);
    fmes = _measurement(2:3); 
  
    % correction step KF 2 (2D position estimation)
    z2 = [ fmes x1]'; 
    y2 = z2 - H2*x2;
    S2 = H2*P2*H2' + R2;
    K2 = P2*H2'*inv(S2);
    x2 = x2 + K2*y2;
    P2 = (eye(ns2)-K2*H2)*P2;
    
    estimatorId = 2;
    muUSBL = x2(1:2);
    dX = muUSBL - mus(:,estimatorId);    
    updateEstimator(ft, dX, estimatorId);
    allUsbl = [allUsbl; _measurement];
    
endfunction

function filterReadDgps(_measurement)

  global allDgps;  
  
  allDgps = [allDgps; _measurement];
  
endfunction

function filterProcessing(_t)

 global active b p r f Wbp Wpr Wrf Wpp pDts;
 global SIGMAS_Init tEstimators SIGMAS mu mus phi1 phi2 clb;
 global mapRtoNodes nb ne ns;
  
 global allB_PR_F_layerB;
 global allB_PR_F_layerP;
 global allB_PR_F_layerR;
 global allB_PR_F_layerF;
 global allB_PR_F_Sigmas;
 global allB_PR_F;
 global ft;
 
 dt = _t - ft;
 
 
 if sum(active) > 0  
  
    % Layer B
    b = zeros(nb,1); 
    b(1) = 1; % only one behavior available
    
    % Reset signal     
    reset = zeros (ne*nb,1);
    for k = 1 : ne*nb   

        if r(k) > 0 && clb(k)~=k;
        
          ileft = clb(k);
          reset (ileft) = 1;
          ileft = mapRtoNodes(ileft);        
          mus(:,ileft) = mu;     
          SIGMAS(1:ns,ns*(ileft-1)+1:ns*ileft) = SIGMAS_Init(1:ns,ns*(ileft-1)+1:ns*ileft);                
          tEstimators(ileft) = 0;
          
        endif
        
    endfor  
    
    % Layer P prediction      
    p = (Wbp*b) .* max((1.-reset).*(Wpp*p + dt*pDts), 1);
    
    % Layer R    
    h = zeros(ne*nb,1);
    
    a = (repmat(active,2,1))'; % active nodes
    for i = 1: ne*nb
    
      iLeft = clb(i);
      
      % node's index in the estimator space in [1 ... ne]
      eNode = mapRtoNodes(i); 
      eLeft = mapRtoNodes(iLeft);
      
      if (p(iLeft)>0&&a(i)) % to improve efficiency
             
        eLeft1 = (eLeft-1)*ns+1;
        eLeft2 = eLeft*ns;
        g = gamma(p(iLeft), SIGMAS(:,eLeft1:eLeft2), mus(:,eNode)- mus(:,eLeft), phi2);
        h(i) = ((g - phi1)>0)*g;       
 
      endif 
    endfor
        
    r = h;

    % Layer F
    f = Wrf*r;
    f = f/sum(f);
    
    % Estimation fusion          
    mu = mus*f; 
   
    % logging
    allB_PR_F = [allB_PR_F; [ft mu']];  
    allB_PR_F_layerB = [allB_PR_F_layerB; [ft b']];
    allB_PR_F_layerP = [allB_PR_F_layerP; [ft p']];
    allB_PR_F_layerR = [allB_PR_F_layerR; [ft r']];
    allB_PR_F_layerF = [allB_PR_F_layerF; [ft f']];
    allB_PR_F_Sigmas = [allB_PR_F_Sigmas ; [ft diag(SIGMAS(1:ns,1:ns))']];
    
    active = zeros(ne,1); 
    
  endif
  
  ft = ft + dt;
  
endfunction
    
    
function filterPlotData()

  global allDgps allB_PR_F allUsbl;

  figure(1);
  hold on;
  title("B-PR-F Neural Network");    
  plot (allDgps(:,2),  allDgps(:,3), 'k',"LineWidth", 1);
  plot (allB_PR_F(:,2),  allB_PR_F(:,3), 'm',"LineWidth", 1);
  plot (allUsbl(:,2), allUsbl(:,3), '.b',"LineWidth", 1);
  axis("equal", "tight");
  print("-deps", "-color", "output/plots/filterB_PR_F.eps");

endfunction

function filterRecordData()

  global allB_PR_F;
  global allB_PR_F_layerP;
  global allB_PR_F_Sigmas;

  save 'output/mat/filterB_PR_F.mat' allB_PR_F;
  
  % Additional data saving for plotting
  save 'output/mat/filterB_PR_F_LayerP.mat' allB_PR_F_layerP;
  save 'output/mat/filterB_PR_F_Sigmas.mat' allB_PR_F_Sigmas;

endfunction


