T = 0.1; 
t = timer('TimerFcn',{@marker_rviz},...
    'Period',T,'ExecutionMode','fixedSpacing');
start(t);