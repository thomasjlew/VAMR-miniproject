% t = timer('TimerFcn', 'stat=false; disp(''Timer!'')',... 
%                  'StartDelay',10);
% start(t)
% stat=true;
% while(stat==true)
%   disp('.')
%   pause(1)
% end
% addpath('..');
disp('asdf')
try
    main_duckie_no;
end
disp('asdf')
try
    main_duckie_metricRec;
end
try
    main_duckie_no2;
end
try
    main_duckie_metricRec2;
end


try
    main_kitti_no;
end
try
    main_kitti_BA;
end
try
    main_kitti_no2;
end
try
    main_kitti_BA2;
end

try
    main_malaga_no;
end
try
    main_parking_no;
end
try
    main_malaga_no2;
end
try
    main_parking_no2;
end



