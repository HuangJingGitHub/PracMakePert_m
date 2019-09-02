classdef teleOp < handle
  properties(Access = public)
    publisher;
    jointStateMsg;
    tStart = tic;
    R2D = 180/pi;
    dt = 0.001;   
%     T_psm_mtm = [ -1         0        0   -0.0001;
%                    0        -1        0   -0.3639;
%                    0         0        1   -0.0359;
%                    0         0        0    1 ];
    T_psm_mtm = [ -1         0        0;
                   0        -1        0;
                   0         0        1];

    psm_q;
    psm_x_cur;
    psm_x_dsr_pre;
    psm_x_dsr;
    mtm_x;
    %%% difference map
    psm_x_initial;
    mtm_x_initial;
    mtm_R;
    mtm_x_dif;
    %%%
    lambda = diag([500 500 500 200 200 200]);
    
    counter;
    origin_pos;
    cannula_init;
    cannula_cur;
    cannula_rot;
    fig;

  end

    methods(Access = public)
        
        function obj = teleOp(mtm_q_initial,psm_q_initial,publisher,jointStateMsg)  % Constructor
            obj.psm_q   = psm_q_initial;
            mtm         = dVRK_MTMmodel(mtm_q_initial);
            psm         = dVRK_PSMmodel_SS(psm_q_initial);
            obj.mtm_x   = MTM_FK(mtm);
            %%%
            obj.mtm_x_initial = obj.mtm_x;
            obj.mtm_R   = eye(3);
            obj.mtm_x_dif = zeros(3, 1);
            %%%
            [obj.psm_x_cur, obj.origin_pos, ~] = PSM_FK(psm);
            obj.psm_x_initial   = obj.psm_x_cur;
            obj.psm_x_dsr       = obj.psm_x_initial;
            obj.psm_x_dsr_pre   = obj.psm_x_initial;
            %obj.psm_x_dsr       = obj.T_psm_mtm * obj.mtm_x;
            %obj.psm_x_dsr_pre   = obj.psm_x_dsr; 
            
            obj.counter = 1;
            obj.fig     = init_fig();
            
            L = 0.0879;
            W = 0.0462;
            twist = pi/6;
            qubic_a = (L - 2*W) / L^3;
            qubic_b = (3*W - L) / L^2;
            caula = zeros(3, 200);
            tau   = linspace(0, L, 100);
            caula(3, 1:100) = linspace(0, 0.25, 100);
            caula(3, 101:end) = -tau;
            caula(2, 101:end) = (qubic_a * tau.^3 + qubic_b * tau.^2) * (-cos(twist));
            caula(1, 101:end) = (qubic_a * tau.^3 + qubic_b * tau.^2) * sin(twist);
            obj.cannula_init  = caula;
            obj.cannula_rot   = eye(3);
            obj.cannula_cur   = caula;
            %obj.cannula_pos = Cannula_pos(psm_q_initial);
            %obj.cannula_cur = roty(-obj.psm_q(1)*obj.R2D) * rotx(-obj.psm_q(2)*obj.R2D) *  obj.cannula;   
            
            fig_update(obj.fig, obj.psm_x_cur, obj.origin_pos, obj.cannula_rot, obj.cannula_cur);

            if (nargin > 3)
                obj.publisher = publisher;
                obj.jointStateMsg = jointStateMsg;
            end    
        end

        function  [psm_q, tracking_err] = run(obj, mtm_q)
            mtm             = dVRK_MTMmodel(mtm_q);
            psm             = dVRK_PSMmodel_SS(obj.psm_q);
            [obj.psm_x_cur, obj.origin_pos, J] = PSM_FK(psm);
            obj.cannula_rot = roty(-obj.psm_q(1)*obj.R2D) * rotx(-obj.psm_q(2)*obj.R2D);
            obj.cannula_cur = obj.cannula_rot * obj.cannula_init;
            obj.mtm_x       = MTM_FK(mtm);
            %obj.psm_x_dsr   = obj.T_psm_mtm * obj.mtm_x;
            %%%
            [obj.mtm_R, obj.mtm_x_dif]  = xdif_matrix(obj.mtm_x, obj.mtm_x_initial);
            obj.psm_x_dsr(1:3, 1:3)     = obj.mtm_R * obj.psm_x_initial(1:3, 1:3);
            obj.psm_x_dsr(1:3, 4)       = obj.T_psm_mtm * obj.mtm_x_dif + obj.psm_x_initial(1:3, 4);
            %%%
            psm_xdot_dsr    = PSM_Vel_Cal(obj.psm_x_dsr, obj.psm_x_dsr_pre);
            
            tracking_err    = xdif_lite(obj.psm_x_dsr, obj.psm_x_cur); 
            qdot_psm        = pinv(J) * (psm_xdot_dsr + obj.lambda * tracking_err);
            obj.psm_q       = obj.psm_q + qdot_psm * obj.dt;
            psm_q           = obj.psm_q;
            obj.psm_x_dsr_pre = obj.psm_x_dsr;
            
            obj.counter = obj.counter + 1;
            if ~mod(obj.counter, 20)
                fig_update(obj.fig, obj.psm_x_cur, obj.origin_pos, obj.cannula_rot, obj.cannula_cur);
                obj.counter = 0;
            end                
            %transform mtm_q to psm desired Cartesian position and psm desired Cartesian velocity
            %and then call inverse kinematics function

        end    
        
        function  callback_update_mtm_q(obj, q)
            obj.jointStateMsg.Position = obj.run(q);
            tElapsed = toc(obj.tStart);
            if (tElapsed > 0.033)
                obj.tStart = tic;
                obj.publisher.send(obj.jointStateMsg);
            end    
        end

    end
end
