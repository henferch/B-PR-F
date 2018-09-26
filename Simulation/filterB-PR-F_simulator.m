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

global ns = 3; % state dimension
global nb = 2; % Behaviour profiles number
global ne = 4; % Estimator processes available 

global Wbp = zeros(ne*nb,nb);      
global Wpr = zeros(ne*nb,ne*nb);      
global Wrf = zeros(ne,ne*nb);  
global Wpp = zeros(ne,ne*nb);
global Wneigh = [0.001 0.0001 2.0 10.0; 0.01 0.8 1.8 0.0001]
global mapRtoNodes = zeros(ne*nb,1);  

global b = zeros(nb,1);     % Layer B (Behavour)
global p = zeros(ne*nb,1);  % Layer P (Prediction)
global r = zeros(ne*nb,1);  % Layer R (Reliability)
global f = ones(ne,1);      % Layer R (Fusion)
global active = zeros(ne,1); % active nodes vector

global phi1 = 0.2;  % Reliability threshold
global phi2 = 13;   % steepness of the sigmoid 

global mu = zeros(ns,1); % global mu

global altitude = 100000; % robot altitude from the seabed

global pDts = zeros(nb*ne,1);
global tEstimators = zeros(1,ne); % processes internal clock

SIGMA_E11_Init = 10*eye(ns);    % b=1: IMU in meters
SIGMA_E12_Init = 10000*eye(ns); % b=1: DVL in meters (big variance, deactivated)
SIGMA_E13_Init = 0.4*eye(ns);   % b=1: USBL in meters
SIGMA_E14_Init = 0.1*eye(ns);   % b=1: DGPS in meters

SIGMA_E21_Init = 10000*eye(ns); % b=2: IMU in meters (big variance, deactivated)
SIGMA_E22_Init = 2*eye(ns);     % b=2: DVL in meters
SIGMA_E23_Init = 0.4*eye(ns);   % b=2: USBL in meters
SIGMA_E24_Init = 0.1*eye(ns);   % b=2: DGPS in meters

global SIGMAS_1_Init = [SIGMA_E11_Init SIGMA_E12_Init SIGMA_E13_Init SIGMA_E14_Init];
global SIGMAS_2_Init = [SIGMA_E21_Init SIGMA_E22_Init SIGMA_E23_Init SIGMA_E24_Init];

global SIGMAS = SIGMAS_1_Init; % just for initialization

global clb = zeros(ne*nb,1); % left neighbors 

global mus = zeros(ns,ne); % estimations mean

global altTreshold = 30.0; % threshold to switch behavior policy

global t = 0 ;
global bId = 1;

### ARF Neural network functions Eq. ()
function [result] = gamma(_sigma, _SIGMA, _x, _phi)
  
  [V lambda] = eig(_SIGMA);
  G = _sigma * lambda;
  f = _x' * V' * inv(G) * V * _x; 
  result = (1 / (1 + exp(-_phi.*(_sigma - f))));
 
endfunction

# Calculates Psy( ) conforming to Eq. (9)
function [mu, sigma, SIGMA, dt] = Psy(_leftBrotherId, _mus, _sigmas, _SIGMAS, _tEstimators)
 
 global ns;
 
 sigma = smallSigmas(_leftBrotherId);
 SIGMA = _SIGMAS(1:ns,ns*(_leftBrotherId-1)+1:ns*_leftBrotherId);
 mu = _mus(_leftBrotherId,:);
 dt = _tEstimators(_leftBrotherId);
     
endfunction 
 
% Computes the sparse encoding of a number into a binary vector
function spaVec = sparseEncode(_lenght, _number)
  v = _number;
  while (v  > _lenght )
    v = v - _lenght;
  endwhile
  spaVec = zeros(1,_lenght);
  spaVec(v) = 1;
endfunction

### Data containers

global allB_PR_F_Mus = [];
global allUsbl = [];
global allDgps = [];
global allCompass  = [];
global allAltimeter  = [];

global allB_PR_F_layerB = [];
global allB_PR_F_layerP = [];
global allB_PR_F_layerR = [];
global allB_PR_F_layerF = [];
global allB_PR_F_Sigmas = [];

### Filter interface functions 

function filterInfo()
  display("B-PR-F Neural Network filter loaded!");
endfunction
  
function filterInitialize(_mu, _theta, _estimatorDt)
  
  global  x1 x2 ne ns mu mus nb ne;
  global Wneigh Wbp Wpr Wrf mapRtoNodes clb Wpp;
  global pDts;
  
  mu = _mu(2:end);
  mus = ones(ns,ne).*_mu(2:ns+1);  
    
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
  eDts = repmat(_estimatorDt,nb,1);
  
  for i = 1 : nb*ne
    pDts(i) = eDts(crb(i));
  endfor
  pDts(1) = pDts(1)/3.0;
  pDts(6) = pDts(6)/3.0; 
  
  pDts = pDts.^-1;
  
  % P recurrent weights
  sparse_12 = [];
  sparse_21 = [];
  
  for i = 1: ne
    sparse_12 = [sparse_12 ; sparseEncode(ne,clb(crb(i)+ne)) ];
    sparse_21 = [sparse_21 ; sparseEncode(ne,clb(crb(i+ne)-ne))];
  endfor
   
   Wpp = [eye(ne) sparse_12;
          sparse_21 eye(ne)];
  
endfunction

function updateEstimator(_t, _dX, _estimatorId)
       
    global mus tEstimators SIGMAS;
    global ns active;
    
    active(_estimatorId) = 1;
   
    mus(:,_estimatorId) = mus(:,_estimatorId) + _dX;
           
    noiseE = abs(_dX).*eye(ns); % noise added from motion
    siODOMc = SIGMAS(1:ns,ns*(_estimatorId-1)+1:ns*_estimatorId);              
    SIGMAS(1:ns,ns*(_estimatorId-1)+1:ns*_estimatorId) = siODOMc + noiseE;
     
    _tEstimators(_estimatorId) = _t;
        
endfunction

function filterReadImu(_measurement)
    
    global ns;
    
    estimatorId = 1;
    t = _measurement(1);
    dX = _measurement(2:ns+1);    
    updateEstimator(t, dX, estimatorId);
        
endfunction

function filterReadDvl(_measurement)
    
    global altitude altTreshold ns;
    
    if(altitude < altTreshold )    
      estimatorId = 2;
      t = _measurement(1);
      dX = _measurement(2:ns+1);    
      updateEstimator(t, dX, estimatorId);
    endif
        
endfunction


function filterReadUsbl(_measurement)

    global ns mus allUsbl;
    
    estimatorId = 3;
    t = _measurement(1);
    muUSBL = _measurement(2:ns+1);
    dX = muUSBL - mus(:,estimatorId);    
    updateEstimator(t, dX, estimatorId);
    allUsbl = [allUsbl _measurement];
    
endfunction

function filterReadDgps(_measurement)

    global ns mus allDgps;
    
    estimatorId = 4;
    t = _measurement(1);
    muDGPS = _measurement(2:ns+1); 
    dX = muDGPS - mus(:,estimatorId);    
    updateEstimator(t, dX, estimatorId);
    allDgps = [allDgps _measurement];     
endfunction

function filterReadAltimeter(_measurement)
  
    global altitude allAltimeter;
    altitude = _measurement(2);
    allAltimeter = [allAltimeter _measurement];

endfunction

function [_b, _SIGMAS_Init] = getBehavior(_bIdPrev, _mu)
  
  global SIGMAS_1_Init SIGMAS_2_Init mus ne active altitude altTreshold;
  # two behaviors available 
  # 1: navigating near the surface (altitude >= -40 m )
  # 2: deep navigation in deep water (depth < -40 m )
  
  _b = 1;
  _SIGMAS_Init = SIGMAS_1_Init;
  if (altitude < altTreshold && active(2))
    _b = 2;
    _SIGMAS_Init = SIGMAS_2_Init;
  endif
  if (_b~=_bIdPrev)
    # when behavior profile changes, initialize the estimators with the global estimate available
    for i = 1 : ne
      mus(:,i) = _mu;
    endfor
  endif
endfunction

function filterProcessing(_dt)
 
 global active b p r f Wbp Wpr Wrf Wpp pDts;
 global tEstimators SIGMAS mu mus phi1 phi2 clb;
 global mapRtoNodes nb ne ns;
 global allB_PR_F_layerB;
 global allB_PR_F_layerP;
 global allB_PR_F_layerR;
 global allB_PR_F_layerF;
 global allB_PR_F_Mus;
 global t bId;
 
 if sum(active) > 0  
  
    % Layer B
    b = zeros(nb,1); 
    [bId, SIGMAS_Init] = getBehavior(bId, mu); 
    b(bId) = 1;
    
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
    
    p = (Wbp*b) .* max((1.-reset).*(Wpp*p + _dt*pDts), 1);
    
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
    allB_PR_F_Mus = [allB_PR_F_Mus [t; mu; bId]];  
    allB_PR_F_layerB = [allB_PR_F_layerB; [t b']];
    allB_PR_F_layerP = [allB_PR_F_layerP; [t p']];
    allB_PR_F_layerR = [allB_PR_F_layerR; [t r']];
    allB_PR_F_layerF = [allB_PR_F_layerF; [t f']];
    
    active = zeros(ne,1); 
    
  endif
  
  t = t +_dt;

endfunction
    
    
function filterPlotData(gtData)

  global allDgps allB_PR_F_Mus allUsbl allAltimeter ns
  global allB_PR_F_layerB allB_PR_F_layerP allB_PR_F_layerR allB_PR_F_layerF;

  figure(1);
  hold on;
  title("B-PR-F Neural Network");    
  
  plot3 (gtData(2,:), gtData(3,:), gtData(4,:), 'g',"LineWidth", 1);
  if size(allDgps,2) > 0
    plot3 (allDgps(2,:), allDgps(3,:), allDgps(4,:), 'ok',"LineWidth", 2);
  endif
  if size(allUsbl,2) > 0
    plot3 (allUsbl(2,:), allUsbl(3,:), allUsbl(4,:), '.k',"LineWidth", 2);
  endif
  
  n = size(allB_PR_F_Mus,2);
  p = c = allB_PR_F_Mus(:,1);
  i = 1;
  flag = 1;
  while i<n
    while c(end) == p(end) && i<n
      c = allB_PR_F_Mus(:,i);
      i = i + 1;
    endwhile
    if p(end) == 1
        plot3 (allB_PR_F_Mus(2,flag:i-1), allB_PR_F_Mus(3,flag:i-1), allB_PR_F_Mus(4,flag:i-1), 'b',"LineWidth", 1);
    else
        plot3 (allB_PR_F_Mus(2,flag:i-1), allB_PR_F_Mus(3,flag:i-1), allB_PR_F_Mus(4,flag:i-1), 'r',"LineWidth", 1);
    endif
    flag = i;
    p = c;
  endwhile
  if p(end) == 1
      plot3 (allB_PR_F_Mus(2,flag-1:flag), allB_PR_F_Mus(3,flag-1:flag), allB_PR_F_Mus(4,flag-1:flag), 'b',"LineWidth", 1);
  else
      plot3 (allB_PR_F_Mus(2,flag-1:flag), allB_PR_F_Mus(3,flag-1:flag), allB_PR_F_Mus(4,flag-1:flag), 'r',"LineWidth", 1);
  endif
  
  v = [-40,35];
  axis("equal", "tight");
  legend("GT", "DGPS", "USBL","Behavour1", "Behavour2");
  view(v);
  print("-deps", "-color", "output/filterB_PR_F.eps");

  figure(2)
  plot3 (gtData(2,:), gtData(3,:), gtData(4,:), 'g',"LineWidth", 1);
  hold on;
  plot3 (allUsbl(2,:), allUsbl(3,:), allUsbl(4,:), 'b',"LineWidth", 2);
  axis("equal", "tight");
  legend("GT", "USBL");
  title("USBL sensor trajectory");    
  view(v);
  print("-deps", "-color", "output/USBLTrajectorySim.eps");
  
  
  figure(3)
  plot3 (gtData(2,:), gtData(3,:), gtData(4,:), 'g',"LineWidth", 1);
  axis("equal", "tight");
  title("Ground truth ");    
  view(v);
  print("-deps", "-color", "output/GTTrajectorySim.eps");
 
  figure(4);
  hold on;
  title("Time evolution of RMS error");
  gtData = gtData(:,1:end-1);
  err1 =  allB_PR_F_Mus(2:4,:) - gtData(2:4,:);
  err1 = (sum((err1.*err1),1)/ns).^0.5;
  plot(gtData(1,:), err1(1,:), "r");  
  xlabel("Time in sec");
  ylabel("Error in m");
  
  meanError = mean(err1);
  stdError = std(err1);
  display(strcat("Estimation RMS error: ", num2str(meanError)));
  display(strcat("Estimation stdev error: : ", num2str(stdError)));
  
  nGt = size(gtData,2);
  nUsbl = size(allUsbl,2);
  subSamSteps = round(nGt / nUsbl);
  idx = [1:subSamSteps:nGt];
  idx = idx(1:end-1);
  
  err2 =  gtData(2:4,idx) - allUsbl(2:4,:);
  err2 = (sum((err2.*err2),1)/ns).^0.5;
  hold on;
  plot(gtData(1,idx), err2(1,:), "b");  
  legend("B-PR-F", "USBL");
  print("-deps", "-color", "output/ErrorFilterVsUSBL.eps");
  
  figure(6)
  hold on;
  n = size(allAltimeter,2);
  plot(allAltimeter(1,:), ones(1,n)*22, 'g');
  plot(allAltimeter(1,:), ones(1,n)*30, 'b');
  plot(allAltimeter(1,:), allAltimeter(2,:), 'r');
  title("Time evolution of the robot altitude");
  xlabel("time in sec");
  ylabel("Altitude in m");
  print("-deps", "-color", "output/AltimeterTreshold.eps");

endfunction
