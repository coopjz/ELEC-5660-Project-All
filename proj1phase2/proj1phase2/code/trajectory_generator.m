    function s_des = trajectory_generator(t, path, h)

persistent P

if nargin > 1 % pre-process can be done here (given waypoints)
    %nargin >1, we give all three parameter, process the trajectory
    %function
    %nargin =1, we give only t time, output the desire states

    t_total = 25;           %total trajectory time 25s
    P.t_total = t_total;
    n_wp= size(path,1);     %number of waypoints
    n_seg = n_wp -1;        %number of segments
    P.n_seg =n_seg;
    dim_wp =size(path,2);
    n_coeff  = 8;            %minimum snap, 7th degree with 8 coefficient
    P.n_coeff =n_coeff;
    n_order  = 7;
    interval=path(2:end,:) - path(1:end-1,:);
                            %interval between each waypoints
    dis_interval = vecnorm(interval,2,2);
    dis_total    = sum(dis_interval);
    P.t_seg = t_total * dis_interval/dis_total;
    P.cum_time=[0;cumsum(P.t_seg)];
    
    %% construct Q matrix PQP
    Q =zeros(n_coeff*n_seg);

    for k = 1:n_seg
%         T = P.t_seg(i);
%         Q_i=zeros(n_coeff);
        for i = 4:n_order
            for j=4:n_order
                Q(n_coeff *(k-1)+i+1,n_coeff *(k-1)+j+1) = ...
                (factorial(i)/factorial(i-4)*factorial(j)/factorial(j-4))/(i+j-7)*P.t_seg(k)^(i+j-7);
            end
        end
    end

    %% construct two constraints, waypoints and continuity A*x = b
    % 1. waypoints constraints (n_seg+1)

    A_wp=zeros(2*n_seg,n_coeff*n_seg);
    b_wp=zeros(2*n_seg,dim_wp);
    %start point
    for i=1:n_seg
        T = P.t_seg(i);
        p_t_0 = [1, 0, 0, 0, 0, 0, 0, 0];
        p_t_1=polyBasis(T,n_order,0);
        p_t =[p_t_0;p_t_1];
        A_wp(1+2*(i-1):2+2*(i-1) , (i-1)*n_coeff +(1:n_coeff)) = p_t;
        b_wp(1+2*(i-1):2+2*(i-1),:) = [path(i,:);path(i+1,:)];
    end
    % 2. continuity constraints

    %hyperparam d_num, order of derivatives
    d_num =4;
    A_cont=zeros(d_num*(n_seg-1),n_seg*n_coeff);
    b_cont=zeros(d_num*(n_seg-1),dim_wp);
    index_=1;
    for i=1:(n_seg-1)
        T=P.t_seg(i);
        for j=1:d_num
            row = zeros(1, n_coeff*n_seg);
            row((i-1)*n_coeff+(1:n_coeff))= polyBasis(T,n_order,j);
            row(i*n_coeff+(1:n_coeff)) = -polyBasis(0,n_order,j);
            A_cont(index_,:) =row;
            b_cont(index_,:) = zeros(1,dim_wp);
            index_ =index_+1;
        end

    end
    A_eq =[A_wp;A_cont];
    b_eq =[b_wp;b_cont];
    

    %% construct Quadratic Program & sovle it
    %min 1/2 *P'QP   s.t. A*P=b;
    for d = 1:dim_wp
        P.coeffs(:,d) = quadprog(Q,[],[],[],A_eq,b_eq(:,d));
    end


    %P.coeffs =reshape(x,n_coeff,n_seg);



else % output desired trajectory here (given time)
    
    % s_des(1:3) desire position
    % s_des(4:6) desire velocity
    % s_des(7:9) desire acceleration
    % s_des(10) desire yaw
    % s_des(11) desire yaw rate
    s_des = zeros(13,1);
    T = 0;
    seg =1;
    if t > P.t_total
       % seg = P.n_seg;
       % = P.t_seg(end);
        return
    end   
    for i=1:size(P.cum_time,1)
        if t >= P.cum_time(i)
            t_p =P.cum_time(i);
            seg =i;
        end
    end
    T = t-t_p;
    coeffs_eval = P.coeffs((1+P.n_coeff*(seg -1)):(P.n_coeff +P.n_coeff*(seg-1)),:);
    s_des(1:3) = [1,T,T^2,T^3,T^4,T^5,T^6,T^7]*coeffs_eval;
    s_des(4:6) = [0,1*T^0,2*T^1,3*T^2,4*T^3,5*T^4,6*T^5,7*T^6]*coeffs_eval;
    s_des(7:9) = [0,0*T^0,2*T^0,6*T^1,12*T^2,20*T^3,30*T^4,42*T^5]*coeffs_eval;
 
end


end

function b = polyBasis(t,n,d)
%   t - Evaluation time.
%   n - Maximum exponent (polynomial order).
%   d - Derivative order.
    b = zeros(1, n+1);
    for i = 0:n
        if i < d
            b(i+1) = 0;
        else
            b(i+1) = factorial(i)/factorial(i-d) * t^(i-d);
        end
    end
end



