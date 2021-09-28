%% Simulating the geometric image formation model (pinhole)
%Hanieh Shabanian
% Computer Vision COurse
% University of Memphis
% 8-3-2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function o_PointsEstimated=geometric_simulation(w1,w2,w3,w4,w5,w6)

% Initialization
tic; % Start stopwatch timer
 % Initialization world point given with respect to world origin
if nargin==0
    w1=[0;0;0;1];
    w2=[1;0;0;1];
    w3=[0;1;0;1];
    w4=[0;0;1;1];
    w5=[1;0;1;1];
    w6=[0;1;1;1]; 
 end


i_Points=[w1,w2,w3,w4,w5,w6]; % input points


%% Doing Transformation 
% Initialization required parameters for Interensic parameters

phi_X=1;
phi_Y=1;
gamma_parameter=0;
delta_X=0;
delta_Y=0;

Lambda_parameter=[ phi_X gamma_parameter delta_X 0; 0    phi_Y  delta_Y 0;  0      0     1    0 ];
% Initialization required parameters for Exterinsic parameters

n_parameter=[0;0;1];
theta_parameter=pi/4;
Du=eye(3);
S_parameter=[ 0  -n_parameter(3) n_parameter(2) ; n_parameter(3) 0   -n_parameter(1) ; -n_parameter(2) n_parameter(1) 0 ;    ];
Omega_parameter= (Du + sin(theta_parameter)*S_parameter + (1-cos(theta_parameter))* (S_parameter*S_parameter))';
Tau_parameter=[0;0;0]; % Offset from the world origin


%% Doing Rotation and finding the output and getting the image

Rotation_process=[Omega_parameter,Tau_parameter;[0 0 0 1]];
o_Points1= Lambda_parameter * Rotation_process * i_Points;
o_Points1=o_Points1./(repmat(o_Points1(3,:),[3,1]));

%% estimating Process:  estimating  Extrinsic Parameters by input points and output points

[ OmegaE1,TauE1] = EstimateExtrinsic( i_Points,o_Points1); % Recall Estimate Exterinsic Function

Rotation_process=[OmegaE1,TauE1;[0 0 0 1]];
o_PointsEstimated= Lambda_parameter * Rotation_process * i_Points;
o_PointsEstimated=o_PointsEstimated./(repmat(o_PointsEstimated(3,:),[3,1]));


%% Desplaying  results

% plot original points
subplot(2,3,1);  scatter3(i_Points(1,:),i_Points(2,:),i_Points(3,:),50,i_Points(1:3,:)','filled'); title('Original Points');
xlabel('U'); ylabel('V'); zlabel('W');
%   Doing Rotation and finding the output and getting the image
subplot(2,3,2); scatter(o_Points1(1,:),o_Points1(2,:),50,i_Points(1:3,:)','filled'); title('Getting Image');


% Desplaying estimating Process in order to estimating  Extrinsic Parameters by input points and output points
subplot(2,3,3); scatter(o_PointsEstimated(1,:),o_PointsEstimated(2,:),50,i_Points(1:3,:)','filled'); title('Estimated  Extrinsic Parameters');
