
%% Estimating Extrinsic parameters by input and output variables that we give from Main function
function [ Omega_Parameter,Tau_Parameter] = EstimateExtrinsic( i_Points,o_Points)

% Creating larg matrix
Matrix_L=[];

Padding_Process=[0 0 0 0];
for i=1: size(i_Points,2) % up to input size
    w=i_Points(1:3,i)';
    u=o_Points(1:2,i);
    
    if(sum(isnan(w))==0 && sum(isnan(u))==0 && sum(isinf(w))==0 && sum(isinf(u))==0 )
        First_P=[ -w(1)*u(1) , -w(2)*u(1) , -w(3)*u(1) , -u(1)];
        Second_P=[ -w(1)*u(2) , -w(2)*u(2) , -w(3)*u(2) , -u(2)];
        Section_One=[w,1,Padding_Process,First_P];
        Secition_Two=[Padding_Process,w,1,Second_P];
        Matrix_L=[Matrix_L;Section_One;Secition_Two];
    end
end

[~,~,V] = svd(Matrix_L); %Singular value decomposition
V=V';
estimatedParams=V(:,end);
estimatedParams=(reshape(estimatedParams,[4,3]))'; %reshapes using  estimate parameter vector
TauE_Parameter=estimatedParams(:,end);
OmegaE=estimatedParams(:,1:3);
[UO,~,VO] = svd(OmegaE); %Singular value decomposition
Omega_Parameter=UO*VO;
ra_parameter=sum(sum(Omega_Parameter./Omega_Parameter));
Tau_Parameter=  ra_parameter*TauE_Parameter;
end

