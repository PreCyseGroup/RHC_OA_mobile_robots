clear; close all;

%% ROBOT PARAMETERS
%Sampling time
Tc=0.1;

r=0.02; %wheels radius
d=0.05;  %wheels-axis length

T=[[r/2 r/2];[r/d -r/d]]; %Transformation Matrix
Tinv=inv(T); % Inverse Transformation Matrix


%Linear system matrices
A=[0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B=[0 0; 0 0; 1 0; 0 1];
C=eye(4);
D=zeros(4,2);

sys_c=ss(A,B,C,D);
sys_d=c2d(sys_c,Tc);

Ad=sys_d.A;
Bd=sys_d.B;


% Ad=eye(4)+A*Tc;
% Bd=B*Tc;


compute_path=true;
if compute_path==true
    load exampleMaps.mat
    safe_distance=0.5;
    toll=0.3;
    
    map = binaryOccupancyMap(complexMap,2);
    map2=binaryOccupancyMap(complexMap,2);
    inflate(map2,safe_distance);
    
    show(map2)
    
    
    stateSpace = stateSpaceSE2;
    stateValidator = validatorOccupancyMap(stateSpace,'Map',map2);
    stateValidator.ValidationDistance = 0.01;
    planner = plannerHybridAStar(stateValidator,'InterpolationDistance', safe_distance-toll,'MinTurningRadius', 1);
    
    startLocation = [3 1 0];
    endLocation = [11.75 18.25 pi/2];
    
    path=plan(planner,startLocation,endLocation);
    

    
    path=path.States;
    save('path_data','map','path','startLocation','endLocation','safe_distance','toll','planner')
else
    load('path_data')
end


figure
show(planner)

figure
show(map)



%% TRAJECTORY PARAMETERS
%% TRAJECTORY PARAMETERS
%Initial conditions
x0=startLocation(1); y0=startLocation(2); theta0=startLocation(3);


n_wpts=length(path);
j=3;
xr=path(2,1);
yr=path(2,2);

%Initial condition
q0=[x0;y0;theta0];

%Extending the trajectory vector for prediction over the vector length
%i.e. K+N>length(vr)

%Index for trajectory
K=1;

%% Variables to store and plot the results
tt=[];
useq=[];
qseq=q0;
wrwlseq=[];
Hseq=[];
xiseq=[];
vwseq=[];
gammaseq=[];

%% MPC PARAMETERS
%State Variables
n=4;
%Input Variables
m=2;
%LQ Matrices
Qx=diag([100 100 0 0]); Ru=0.01*eye(2);
warning('off','MATLAB:sqrtm:SingularMatrix');
%Input Constraints
% umin=[-3;-3];


%Dynamic compensator initialization
xi_min=0.01;
xi0=0.01;
xi=xi0;

xi_pre=xi_min;



%Computation time
avg_comp_time=0;

wrmax=10;
wlmax=wrmax;

Hd=[-1/wrmax 0; 0 -1/wrmax;
    1/wlmax 0 ; 0 1/wlmax];
S=[Tc 0; 0 1];


%% Plot parameters

grid
hold on
p_i=plot(path(1,1),path(1,2),'b-p','MarkerIndices',[1 1],'MarkerFaceColor','red','MarkerSize',20);

p_f=plot(path(n_wpts,1),path(n_wpts,2),'b-p','MarkerIndices',[1 1],'MarkerFaceColor','yellow','MarkerSize',20);


plot_ell=true;
%% RESULTS COLLECTION
compute_convergence_time=1;
energy=0;
enrg_seq=[];
warning('off','optim:quadprog:HessianNotSym')
%% MPC ALGORITHM SIMULATION
for i=0:Tc:10000000
    
    tt=[tt i];
    
    %Immagazino la sequenza di stato
    x=qseq(1,end);
    y=qseq(2,end);
    theta=qseq(3,end);
    
    %Current portion of the trajectory x(k|k), x(k+1|k) .... x(k+N-1|k)
    
    
    %Updating State of the feedback-linearized system
    z_k=[x; y; xi*cos(theta); xi*sin(theta)];
    %Tracking error xtilde
    ztilde=z_k-[xr; yr; 0; 0];
    
    
    T_FL=[cos(theta) -xi*sin(theta);
        sin(theta) xi*cos(theta)];
    
    T_FL_inv=inv(T_FL);
    
    H=Hd*Tinv*S*T_FL_inv;
    g=wrmax*ones(4,1);
    
    
    
    
    [Qkot,Fkot,comp_time]=dyn_FL_kothare_constraint_OA(Ad,Bd,ztilde,Qx,Ru,H,safe_distance);
    %        [Qkot,Fkot,comp_time]=dyn_FL_kothare_constraint_lmilab_cuzzola(Ad,Bd,ztilde,Qx,Ru,H);
    uk=Fkot*ztilde;
    
    useq=[useq uk];
    Hseq=[Hseq H];
    
    
    %Traslation to recover the original inputs of the linear system
    gammak=[xi_pre/Tc;0];
    
    uk=uk-T_FL*gammak;
    
    
    gammaseq=[gammaseq gammak];
    
    u1=uk(1);
    u2=uk(2);
    
    
    %%%%%%%%ATTENZIONE%%%%%%%%%%
    %     xip=u1*cos(theta)+u2*sin(theta)-xi/Tc;
    % %     xip=u1*cos(theta)+u2*sin(theta)+transl(1);
    
    
    xip_w=T_FL_inv*uk;
    
    xip=xip_w(1);
    w=xip_w(2);
    
    
    
    xi=xi+xip*Tc;
    
    xi_pre=xi;
    
    
    %     if xi<xi_min
    %        xi=xi_min;
    %     end
    
    v=xi;
    %     w=(u2*cos(theta)-u1*sin(theta))/xi;
    
    
    xiseq=[xiseq xi];
    
    
    vw=[v;w];
    
    vwseq=[vwseq vw];
    
    
    
    %Unicycle/Diffdrive input transformation
    wrwl=Tinv*vw;
    
    %aggiorno la sequenza di ingresso per il differential drive
    wrwlseq=[wrwlseq wrwl];
    
    
    %Applico la legge di controllo al sitema non lineare
    v1=vw(1); w1=vw(2);
    t=0:0.00001:Tc;
    [t,q]= ode45(@(t,q,v,w)DiffDrive(t,q,v1,w1),t,qseq(:,end));
    
    %aggiorno la sequenza di stato e ingresso
    qseq=[qseq q(end,:)'];
    
    
    ell=ellipsoid([xr; yr; 0; 0],Qkot);
    
    Qb=(1/safe_distance^2*eye(4));
    B_d=ellipsoid([xr; yr; 0; 0],inv(Qb));
    
    if plot_ell== true && (mod(K,20)==0 || K==1)
       
        plot(projection(ell,[1 0; 0 1; 0 0 ; 0 0]));
        plot(projection(B_d,[1 0; 0 1; 0 0 ; 0 0]),'g');
        
    end
    
    
    %plotto la traiettoria del robot
    pause(0.001)
    p_traj=plot(qseq(1,end),qseq(2,end),'b.');
    
    %Plotto la traiettoria di riferimento
    
    
    if sqrt(([x;y]-[xr;yr])'*([x;y]-[xr;yr]))<toll
        if j<n_wpts
            xr=path(j,1);
            yr=path(j,2);
            j=j+1;
           
        elseif j==n_wpts
            xr=path(j,1);
            yr=path(j,2);
            toll=0.005;
            j=j+1;
   
        else
            break
        end
    end
    
    
    K=K+1;
    
end

r=0.001;
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
p_ball= plot(xunit, yunit,'g');
p_kot= plot(xunit, yunit,'r');



l=legend([p_i p_f p_traj p_ball p_kot ],{'Initial Location','Final Location','Robot Trajectory','Safe Ball','Invariant Ellipsoid'});
l.AutoUpdate='off';


figure
subplot(3,1,1);
hold on;
p1=plot(tt,wrwlseq(1,:));
p2=plot(tt,ones(1,length(tt))*wrmax);
p1.LineWidth=2;
p2.LineStyle='--';
p2.LineWidth=0.5;
p2.Color='red';


l1=legend([p1,p2],{'$\omega_r(t)$','$\omega_{r,max}$'},'Interpreter','latex');
l1.set('FontSize',10)
grid;
xlbl1=xlabel('$t[sec]$','Interpreter','latex');
ylbl1=ylabel('$\omega_r(t)[RAD/sec]$','Interpreter','latex');
ylbl1.FontSize=13;
xlbl1.FontSize=13;

axis([0 174 0 10.5])


subplot(3,1,2);
hold on;
p3=plot(tt,wrwlseq(2,:));
p4=plot(tt,ones(1,length(tt))*wlmax);
p3.LineWidth=2;
p4.LineStyle='--';
p4.LineWidth=0.5;
p4.Color='red';
axis([0 174 0 10.5])

l2=legend([p3,p4],{'$\omega_l(t)$','$\omega_{l,max}$'},'Interpreter','latex');
l2.FontSize=10;
grid;
xlbl2=xlabel('$t[sec]$','Interpreter','latex');
ylbl2=ylabel('$\omega_l(t)[RAD/sec]$','Interpreter','latex');
ylbl2.FontSize=13;
xlbl2.FontSize=13;



subplot(3,1,3)
pl=plot(tt,qseq(3,1:end-1));
pl.LineWidth=2;
grid
% axis([0 tt(length(tt)) 0 6])
xlbl2=xlabel('$t [sec]$','Interpreter','latex');
ylbl2=ylabel('$\theta(t)[RAD]$','Interpreter','latex');

axis([0 174 -0.5 2.5])

l2=legend(pl,'$\theta(t)$','Interpreter','latex');



figure
subplot(2,1,1)
p10=plot(tt,vwseq(1,1:end));
pl0.LineWidth=2;
xlbl10=xlabel('$t[sec]$','Interpreter','latex');
ylbl10=ylabel('$v[m/s]$','Interpreter','latex');
grid

subplot(2,1,2)
p2=plot(tt,vwseq(2,1:end));
p2.LineWidth=2;
xlbl2=xlabel('$t[sec]$','Interpreter','latex');
ylbl2=ylabel('$\omega[RAD/s]$','Interpreter','latex');
grid


save('poly_data','useq','Hseq','gammaseq');

save('data_plot','qseq','wrwlseq','planner','path','map','tt')