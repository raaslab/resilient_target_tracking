%=== set the address for the connection
% IP address of the computer running roscore (ros server)
% setenv('ROS_MASTER_URI','http://192.168.1.101:11311'); 
% rosinit; % connect to the ros server
  
function marker_rviz(~,~)

%=== specify data to publish
markerPub = rospublisher('/visualization_marker','visualization_msgs/Marker');
marker = rosmessage(markerPub);
   
%- set the frame information and names of marker
marker.Header.FrameId = 'my_frame';
%marker.Ns = 'basic_shapes';
%marker.Text = 'cube';
   
%- set the time
%marker.Header.Stamp = rostime('now','system');
   
%- set the ID of the shape
marker.Id = 0;
marker.Type = 2;
   
%- set the scale of the shape
marker.Scale.X = 1;
marker.Scale.Y = 1;
marker.Scale.Z = 1;
   
%- set position and orientation
Pos = rosmessage('geometry_msgs/Point');
Pos.X = 0;
Pos.Y = 0;
Pos.Z = 0;                
Ori = rosmessage('geometry_msgs/Quaternion');
Ori.W = 1;
marker.Pose.Position = Pos; 
marker.Pose.Orientation = Ori;
   
%- set color
Color = rosmessage('std_msgs/ColorRGBA');
Color.R = 0.1;
Color.G = 0.1;
Color.B = 0.1;
Color.A = 1;
marker.Color = Color;

marker.Header.Stamp = rostime('now','system');
send(markerPub, marker);
pause(1);

end
