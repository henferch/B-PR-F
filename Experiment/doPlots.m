###########################################################################################
#
#   <This program loads the experiment dataset and compares the implementation of the  
#   B-PR-F neural network to the Kalman Filter (KF), and the Augmented Monte Carlo (A-MCL) 
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
#   Description: This script generates the additional plots for data visualization
#   Requirement: This program must be executed after generating data with the script 
#   "main.m" for all the implemented filters in folder "filters".
#
###########################################################################################

clear all;
pkg load geometry;

display("Begining aditional plot generation ...");
fflush(stdout); 
### Step One: check wheter data has been generated

if  ~(exist("output/mat/filterKF.mat"))
    display("Error: Please run 'filterKF.m'");
elseif  ~(exist("output/mat/filterKF_post.mat"))
    display("Error: Please run 'filterKF_post.m'");
elseif  ~(exist("output/mat/filterA_MCL.mat"))
    display("Error: Please run 'filterA_MCL.m'");
elseif  ~(exist("output/mat/filterDgps.mat"))
    display("Error: Please run 'filterDgps.m'");
elseif  ~(exist("output/mat/filterCompass.mat"))
    display("Error: Please run 'filterDeadReckoning.m'");
elseif  ~(exist("output/mat/filterUsbl.mat"))
    display("Error: Please run 'filterDeadReckoning.m'");
elseif  ~(exist("output/mat/filterDeadReckoning.mat"))
    display("Error: Please run 'filterDeadReckoning.m'");
elseif  ~(exist("output/mat/filterB_PR_F.mat"))
    display("Error: Please run 'filterB_PR_F.m'");
elseif  ~(exist("output/mat/filterB_PR_F_LayerP.mat")&&exist("output/mat/filterB_PR_F_Sigmas.mat")&&exist("output/mat/filterB_PR_F.mat"))
    display("Error: Please run 'filterB_PR_F.m'");
elseif ~(exist("data/odometryData.mat")&&exist("data/dgpsData.mat"))
    display("Error: Please run 'loadData.m'");
else
  load("output/mat/filterDgps.mat");
  load("output/mat/filterUsbl.mat"); 
  load("output/mat/filterCompass.mat");
  load("output/mat/filterDeadReckoning.mat");  
  load("output/mat/filterA_MCL.mat");
  load("output/mat/filterKF.mat");
  load("output/mat/filterKF_post.mat");
  load("output/mat/filterB_PR_F.mat");
  load("output/mat/filterB_PR_F_LayerP.mat");
  load("output/mat/filterB_PR_F_Sigmas.mat"); 
  load("data/odometryData.mat");   
  load("data/dgpsData.mat"); 
    
  nCompass = size(allCompass, 1);
  nUsbl = size(allUsbl,1);
  nDgps = size(allDgps,1);
  nDeadReckoning = size(allDeadReckoning,1);  
  nA_MCL = size(allA_MCL,1);
  nKF = size(allKF,1);
  nKF_post = size(allKF_post,1);
  nB_PR_F = size(allB_PR_F,1);
  

  ### Step Two: prepare data for visualization

  times = allDgps(:,1);
  errorUsbl = [];
  for i = 1 : nUsbl
    t = allUsbl(i,1);
    [m ix] = min(abs(times.-t));
    errorUsbl = [errorUsbl; [allDgps(ix,:) allUsbl(i,:)]];
  endfor

  errorUsbl(:,7) = (((errorUsbl(:,2)-errorUsbl(:,5)).^2 + (errorUsbl(:,3)-errorUsbl(:,6)).^2)/2).^0.5;

  errorKF = [];
  for i = 1 : nKF
    t = allKF(i,1);
    [m ix] = min(abs(times.-t));
    errorKF = [errorKF; [allDgps(ix,:) allKF(i,:)]];
  endfor

  errorKF(:,7) = (((errorKF(:,2)-errorKF(:,5)).^2 + (errorKF(:,3)-errorKF(:,6)).^2)/2).^0.5;

  errorKF_post = [];
  for i = 1 : nKF_post
    t = allKF_post(i,1);
    [m ix] = min(abs(times.-t));
    errorKF_post = [errorKF_post; [allDgps(ix,:) allKF_post(i,:)]];
  endfor

  errorKF_post(:,7) = (((errorKF_post(:,2)-errorKF_post(:,5)).^2 + (errorKF_post(:,3)-errorKF_post(:,6)).^2)/2).^0.5;

  errorA_MCL = [];
  for i = 1 : nA_MCL
    t = allA_MCL(i,1);
    [m ix] = min(abs(times.-t));
    errorA_MCL = [errorA_MCL; [allDgps(ix,:) allA_MCL(i,:)]];
  endfor

  errorA_MCL(:,7) = (((errorA_MCL(:,2)-errorA_MCL(:,5)).^2 + (errorA_MCL(:,3)-errorA_MCL(:,6)).^2)/2).^0.5;

  errorDeadReckoning = [];
  for i = 1 : nDeadReckoning
    t = allDeadReckoning(i,1);
    [m ix] = min(abs(times.-t));
    errorDeadReckoning = [errorDeadReckoning; [allDgps(ix,:) allDeadReckoning(i,:)]];
  endfor
   
  errorDeadReckoning(:,7) = (((errorDeadReckoning(:,2)-errorDeadReckoning(:,5)).^2 + (errorDeadReckoning(:,3)-errorDeadReckoning(:,6)).^2)/2).^0.5;
  
  errorB_PR_F = [];
  for i = 1 : nB_PR_F
    t = allB_PR_F(i,1);
    [m ix] = min(abs(times.-t));
    errorB_PR_F = [errorB_PR_F; [allDgps(ix,:) allB_PR_F(i,:)]];
  endfor
   
  errorB_PR_F(:,7) = (((errorB_PR_F(:,2)-errorB_PR_F(:,5)).^2 + (errorB_PR_F(:,3)-errorB_PR_F(:,6)).^2)/2).^0.5;

  ### Step Three: Chosing the desired plots

  % Options: 
  % 1) No plot, just statistics
  % 2) Error vs time
  % 3) Accum. error vs time
  % 4) USBL error vs time
  % 5) Layer P effect
  % 6) DeadReckoning vs Compass
  % 7) X DeadReckoning vs DGPS
  % 8) Y DeadReckoning vs DGPS
  % 9) All available plots
  

  plots= zeros(7) ; % set here the desired choice position to '1', or '0' otherwise

  display("Please select a choice ");
 
  choice = menu ("Which aditional plot do you want to generate?", {"1) None, just statistics", "2) Error vs time", "3) Accum. error vs time","4) USBL error vs time", "5) Layer P effect", "6) Dead Reckoning vs Compass", "7) X Dead Reckoning vs DGPS", "8) Y Dead Reckoning vs DGPS", "9) All available plots"});

  switch (choice)
    case 1      display("Choice: No plot, just statistics");
    case 2      plots(1) = 1;
    case 3      plots(2) = 1;
    case 4      plots(3) = 1;
    case 5      plots(4) = 1;
    case 6      plots(5) = 1;
    case 7      plots(6) = 1;
    case 8      plots(7) = 1;
    case 9      plots(1:7) = 1;    
    otherwise   display("Invalid option selections!");
  endswitch  

  display("Please wait ...");
  fflush(stdout); 
  
 if plots(1)>0  

    figure(2);
    hold on;
    plot (errorUsbl(:,4)./60, errorUsbl(:,7), 'r');
    plot (errorKF(:,4)./60, errorKF(:,7), 'k');
    plot (errorKF_post(:,4)./60, errorKF_post(:,7), 'm');
    plot (errorA_MCL(:,4)./60, errorA_MCL(:,7), 'g');
    plot (errorB_PR_F(:,4)./60, errorB_PR_F(:,7), 'c');
    title('Error vs time');
    xlabel("time in min");
    ylabel("Error in m");
    legend(" Usbl ", " KF", " KF Post", " MCL ", " B-PR-F ");
    %axis("equal","tight");
    axis([0 62 0 80]);
    print("-deps", "-color", "output/plots/ErrorVsTime.eps");
   
  endif

   
  ### Step Four: Chosing the desired plots
  if plots(2)>0  
    
    figure(3);
    hold on;
    pe = cumsum(errorDeadReckoning(:,7));
    tt = errorDeadReckoning(:,4)./60;
    [id v] = find(pe<65000);
    cropedErrorDeadReckoning = [tt(id) pe(id)];
    
    merrorKF = [errorKF(:,4)./60 cumsum(errorKF(:,7))];
    merrorKF_post = [errorKF_post(:,4)./60 cumsum(errorKF_post(:,7))];
    merrorA_MCL = [errorA_MCL(:,4)./60 cumsum(errorA_MCL(:,7))];
    merrorB_PR_F = [errorB_PR_F(:,4)./60 cumsum(errorB_PR_F(:,7))];
    
    plot (cropedErrorDeadReckoning(:,1), cropedErrorDeadReckoning(:,2), 'r');
    plot (merrorKF(:,1), merrorKF(:,2), 'k');
    plot (merrorA_MCL(:,1), merrorA_MCL(:,2), 'g');  
    plot (merrorKF_post(:,1), merrorKF_post(:,2), 'm');
    plot (merrorB_PR_F(:,1), merrorB_PR_F(:,2), 'c');
    title('Accumulated error vs time');
    xlabel("time in min");
    ylabel("Error in m");
    legend("DR", "KF", "A-MCL", "KF-P", "B-PR-F",   "location", "northeast", "orientation", "horizontal");
    axis([0 62 0 70000]);
    print("-deps", "-color", "output/plots/AccumErrorVsTime.eps");
    
   
  endif
   
  if plots(3)>0
    figure(4);
    plot (errorUsbl(:,4)./60, errorUsbl(:,7), 'r');
    %axis("equal")
    title('USBL error vs time');
    xlabel("time in min");
    ylabel("Error in m");
    axis([0 62]);
    %axis("equal", "tight");
    print("-deps", "-color", "output/plots/USBLErrorVsTime.eps");
        
  endif 
  
  if plots(4) > 0
      
    iInit = 1;
    iFin = size(allB_PR_F,1);
      
    figure(5);
    hold on;
    
    step = 4/(iFin - iInit);
    sChange = [0 step 0];
    color = [0 0.5 0];
    
    ellis = [];
    for i = iInit :10: iFin
      
      ## draw the ellipse 
      elli = cov2ellipse (diag(allB_PR_F_Sigmas(i,2:3)*allB_PR_F_layerP(i,2)));
      mu = allB_PR_F(i,2:3);
      elli(1:2) = mu(1:2); 
      ellis = [ellis; elli];
      ## plot the ellipse
      %drawEllipse(elli, 'g', "LineWidth", 1);            
      drawEllipse(elli, 'color' , color, "LineWidth", 1);            
      color = color + sChange;
       
    endfor  
    
    ## plot the trajectory
    plot(allB_PR_F(iInit:iFin,2), allB_PR_F(iInit:iFin,3), 'm');
    
    tIni = allB_PR_F(iInit,1);
    tFin = allB_PR_F(iFin,1);
    allUsbl(:,1)>=tIni;
    usbl = allUsbl(allUsbl(:,1)>=tIni&allUsbl(:,1)<= tFin,:);
    
    plot(usbl(:,2), usbl(:,3), '.b');

    title("Layer P: Context prediction");    
    axis("equal", "tight");
    print("-deps", "-color", "output/plots/filterB_PR_F_layerP.eps");

     
  endif  
  
  if plots(5) > 0
  
    figure(6);
    hold on;
    
    ## removing the 0-2pi representation difference
    
    dtCompass = allCompass(2:end,2) - allCompass(1:end-1,2);
    id = dtCompass > pi;
    dtCompass(id) = dtCompass(id) - 2*pi;
    id = dtCompass < -pi;
    dtCompass(id) = dtCompass(id) + 2*pi;
    
    ## getting the same time interval
    
    tInit = allCompass(1,1);
    tFin = allCompass(end,1);
    deadReckoningData = odometryData((odometryData(:,1)>=tInit&odometryData(:,1)<=tFin),:);
    
    ## getting the same number of observations (integrating within the interval)
    
    iMax = size(allCompass,1)-1;
    iDeadReckoning = 1;
    intOdomCompass = [];
    tDeadReckoning = odometryData(iDeadReckoning, 1);
    
    for iCompass = 1: iMax      
      tCompass = allCompass(iCompass,1);
      dt = 0;
      
      while tDeadReckoning <= tCompass
        dt = dt + odometryData(iDeadReckoning,4);
        iDeadReckoning = iDeadReckoning + 1;
        tDeadReckoning = odometryData(iDeadReckoning, 1);
      endwhile
      
      intOdomCompass  = [intOdomCompass; [tCompass dt]];
      
    endfor
    
    plot(allCompass(1:end-1,1)/60, cumsum(dtCompass), 'r');  
    plot(intOdomCompass(:,1)/60, cumsum(intOdomCompass(:,2)), 'b');  
    
    title("Compass vs. Dead reckoning");    
    xlabel("Time in min");
    ylabel("Orientation in rad");
    axis("tight");    
    print("-deps", "-color", "-S320,240", "output/plots/deadReckoningVsCompass.eps");
    display (strcat("Theta dead reckoning correlation: ", num2str(corr(cumsum(dtCompass), cumsum(intOdomCompass(:,2))))));
    
    
  endif
  
  if (plots(6) > 0 || plots(7) > 0)
  
    ## getting the same time interval
    
    tInit = max([dgpsData(1,1) allDeadReckoning(1,1)]);
    tFin = min([dgpsData(end,1) allDeadReckoning(end,1)]);
    
    dgpsData = dgpsData((dgpsData(:,1)>=tInit&dgpsData(:,1)<=tFin),:);    
    allDeadReckoning = allDeadReckoning((allDeadReckoning(:,1)>=tInit&allDeadReckoning(:,1)<=tFin),:);
    
    ## getting the same number of observations (integrating within the interval)
    
    iMax = size(dgpsData,1);
    iDeadReckoning = 1;
    iDeadReckoningMax = size(allDeadReckoning,1);
    intOdomXY = [];    
    tDeadReckoning = allDeadReckoning(iDeadReckoning, 1);
    
    for iDgps = 1: iMax      
    
      tDgps = dgpsData(iDgps,1);
      
      xy = allDeadReckoning(iDeadReckoning,2:3);
      
      while (tDeadReckoning <= tDgps)&&(iDeadReckoning < iDeadReckoningMax)
        
        iDeadReckoning = iDeadReckoning + 1;
        tDeadReckoning = allDeadReckoning(iDeadReckoning, 1);
        
      endwhile
      
      intOdomXY = [intOdomXY; [tDgps xy]];      
          
    endfor
    
    if plots(6) > 0
    
      figure(7);
      hold on;
    
      plot(dgpsData(:,1)/60, dgpsData(:,2), 'r');  
      plot(intOdomXY(:,1)/60, intOdomXY(:,2), 'b');  
      
      display (strcat("X Dead reckoning correlation: ", num2str(corr(dgpsData(:,2), intOdomXY(:,2) ))));
      
      title("DGPS vs. Dead reckoning (X dimension)");    
      xlabel("Time in min");
      ylabel("Distance in m");
      axis("tight");    
      print("-deps", "-color", "-S320,240", "output/plots/deadReckoningX_VsDgps.eps");
      
      
    endif
    
    if plots(7)>0
    
      figure(8);
      hold on;
      
      plot(dgpsData(:,1)/60, dgpsData(:,3), 'r');  
      plot(intOdomXY(:,1)/60, intOdomXY(:,3), 'b');  
      
      title("DGPS vs. Dead reckoning (Y dimension)");    
      xlabel("Time in min");
      ylabel("Distance in m");
      axis("tight");      
      
      print("-deps", "-color", "-S320,240", "output/plots/deadReckoningY_VsDgps.eps");
         
      display (strcat("Y Dead reckoning correlation: ", num2str(corr(dgpsData(:,3), intOdomXY(:,3) ))));
      
    endif
    
  endif    

  
  ### Step Five: displaying the mean and standard deviation of 'error vs time' data
   
  display("Filters error comparisson:"); 
  display(strcat("Dead reckoning \t mean: ", num2str(mean(errorDeadReckoning(:,7))), ", std dev: ", num2str(std(errorDeadReckoning(:,7)))," "));
  display(strcat("KF \t\t mean: ", num2str(mean(errorKF(:,7))), ", std dev: ", num2str(std(errorKF(:,7)))," "));
  display(strcat("A_MCL \t\t mean: ", num2str(mean(errorA_MCL(:,7))), ", std dev: ", num2str(std(errorA_MCL(:,7)))," "));
  display(strcat("KF_post \t mean: ", num2str(mean(errorKF_post(:,7))), ", std dev: ", num2str(std(errorKF_post(:,7)))," "));
  display(strcat("B-PR-F\t\t mean: ", num2str(mean(errorB_PR_F(:,7))), ", std dev: ", num2str(std(errorB_PR_F(:,7)))," "));


endif

display("Plot generation end!");
