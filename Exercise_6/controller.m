classdef controller < matlab.mixin.SetGet
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Controller gains
        K1
        K2
        Km
        KP_force
        KD_force
        S
        S_1
        l1_memories
        e1_memories
        l2_memories
        e2_memories
        A
        B
        Kp_tele
        Kv_tele
        % Pendulum system
        pendulum
    end
    
    methods
        function obj = controller(K1, K2, Kp_force, Kd_force, Km, Kp_t, Kv_t, num, den, pendulum)
            %UNTITLED3 Construct an instance of this class
            %   Detailed explanation goes here
            % Internal states controller
            obj.K1 = K1;
            obj.K2 = K2;
            
            % Force Gains
            obj.Km = Km;
            obj.pendulum = pendulum;
            
            % Hibrid controller gains
            obj.S = [1,0;0,1];
            obj.S_1 = [0,0;0,-1];
            
            % IMpedance controller
            obj.KP_force = Kp_force;
            obj.KD_force = Kd_force;
            
            % Learning parameters
            obj.B = den(2:end);
            obj.A = num(2:end);
            
            % memories values
            
            obj.l1_memories = zeros(length(obj.B), 1);
            obj.e1_memories = zeros(length(obj.A), 1);
            
            obj.l2_memories = zeros(length(obj.B), 1);
            obj.e2_memories = zeros(length(obj.A), 1);
            
            obj.Kp_tele = Kp_t;
            obj.Kv_tele = Kv_t;
        end
        
        function xe = error(obj,xd)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            % Position error definition
            x = obj.pendulum.get_positions();
            xe = xd - x;
        end
        
        function xep = error_p(obj, xdp)
            % Velocity Error Definition
            xp = obj.pendulum.get_velocities();
            xep = xdp -xp;
        end
        
        function xe = error_cartesian(obj, xd)
            % Get general position cartesian space
            x = obj.pendulum.get_general_position();
            
            % Get error Position cartesian Space
            xe = xd -x;
        end
        
        function xep = error_p_cartesian(obj, xdp)
            % Get general position cartesian space
            xp = obj.pendulum.get_general_velocities();
            
            % Get error velocity cartesian Space
            xep = xdp -xp;
        end
        
        function u = get_control_PD(obj, xd, xdp)
            % Get Error Value
            
            xe = obj.error(xd);
            
            % Get Error dot Value
            xep = obj.error_p(xdp);
            
            % Control Law
            u = obj.K1*xe + obj.K2*xep;
        end
        function u = get_control_PD_Gravity(obj, xd, xdp)
            % Get Error Value
            
            xe = obj.error(xd);
            
            % Get Error dot Value
            xep = obj.error_p(xdp);
            
            % Control Law
            u = obj.K1*xe + obj.K2*xep + obj.pendulum.G_matrix();
        end
        
        function u = get_control_inverse_full(obj, xd, xdp, xdpp)
           
            % Auxiliar operator
            v = obj.operator_control(xd, xdp, xdpp);
            
            % Matrices of the system
            M = obj.pendulum.M_matrix();
            C = obj.pendulum.C_matrix();
            G = obj.pendulum.G_matrix();
            F = obj.pendulum.F_matrix();
            
            % Error In matrices
            
            % Get system states 
            x = obj.pendulum.get_positions();
            xp = obj.pendulum.get_velocities();
            
            u = M*v + C*xp + G + F*xp;
            
        end
        function u = get_control_inverse_full_states(obj, x, xd, xdp, xdpp)
            
            % Auxiliar operator
            xe = xd - x(1:2);
            xep = xdp - x(3:4);
            
            v = xdpp + obj.K1*xe + obj.K2*xep;
            
            % Matrices of the system
            M = obj.pendulum.get_M_matrix(x);
            C = obj.pendulum.get_C_matrix(x);
            G = obj.pendulum.get_G_matrix(x);
            F = obj.pendulum.get_F_matrix(x);
            
            % Error In matrices
            
            % Get system states
            xp = x(3:4);
            
            u = M*v + C*xp + G + F*xp;
            
        end
        function u = get_control_tele(obj, x, xd)
            
            % Auxiliar operator
            xe = xd(1:2) - x(1:2);

            
            % Matrices of the system
            G = obj.pendulum.get_G_matrix(x);
            
            % Error In matrices
            
            % Get system states
            xp = x(3:4);
            
            % Control law
            u = G + obj.Kp_tele*(xe) - obj.Kv_tele*xp;
            
        end
        
        function u = get_control_tele_cartesian(obj,x_q, x, xp, xd)
            
            % Auxiliar operator
            xe = xd(1:2) - x(1:2);
            
            
            % Matrices of the system
            G = obj.pendulum.get_G_matrix(x_q);
            J_t = obj.pendulum.get_Jacobian_t(x_q);
            Gx = inv(J_t)*G;
            % Error In matrices
            
           
            
            % Control law
            u = J_t*(Gx + obj.Kp_tele*(xe) - obj.Kv_tele*(xp));
            
        end
        
        function u = get_control_tele_cartesian_force_master(obj,x_q, x, xp, xd, Force, kf)
            
            % Auxiliar operator
            xe = xd(1:2) - x(1:2);
            
            
            % Matrices of the system
            G = obj.pendulum.get_G_matrix(x_q);
            J_t = obj.pendulum.get_Jacobian_t(x_q);
            Gx = inv(J_t)*G;
            % Error In matrices
            
            
            
            % Control law
            u = J_t*(Gx + obj.Kp_tele*(xe) - obj.Kv_tele*(xp) + (kf)*Force);
            
        end
        
        function u = get_control_tele_cartesian_force_slave(obj,x_q, x, xp, xd, Force, kf)
            
            % Auxiliar operator
            xe = xd(1:2) - x(1:2);
            
            
            % Matrices of the system
            G = obj.pendulum.get_G_matrix(x_q);
            J_t = obj.pendulum.get_Jacobian_t(x_q);
            Gx = inv(J_t)*G;
            % Error In matrices
            
            
            
            % Control law
            u = J_t*(Gx + obj.Kp_tele*(xe) - obj.Kv_tele*(xp) + (1/kf)*Force);
            
        end
        
        
        
        function u = get_control_inverse_full_cartesian(obj, xd, xdp, xdpp)
            
            % Auxiliar operator
            v = obj.operator_control_cartesian(xd, xdp, xdpp);
            
            % Matrices of the system
            M = obj.pendulum.M_matrix();
            C = obj.pendulum.C_matrix();
            G = obj.pendulum.G_matrix();
            F = obj.pendulum.F_matrix();
            
            J = obj.pendulum.Jacobian_p();
            J_t = obj.pendulum.Jacobian_t();
            J_dot = obj.pendulum.Jacobian_dot();
            
            % Cartesian Space Matrices
            Mx = inv(J_t)*M*inv(J);
            
            Cx = inv(J_t)*C*inv(J) - Mx*J_dot*inv(J) + inv(J_t)*F*inv(J);
            
            Gx = inv(J_t)*G;
            
            % Error In matrices
            
            % Get system states
            x = obj.pendulum.get_general_position();
            xp = obj.pendulum.get_general_velocities();
            
            u = Mx*v + Cx*xp + Gx;
            u = J_t*u;
            
        end
        
        function u = get_control_impedance(obj, xd, xdp, xdpp, Fa)
            
            % Auxiliar operator
            v = obj.operator_control_impendance(xd, xdp, xdpp, Fa);
            
            % Matrices of the system
            M = obj.pendulum.M_matrix();
            C = obj.pendulum.C_matrix();
            G = obj.pendulum.G_matrix();
            F = obj.pendulum.F_matrix();
            
            J = obj.pendulum.Jacobian_p();
            J_t = obj.pendulum.Jacobian_t();
            J_dot = obj.pendulum.Jacobian_dot();
            
            % Cartesian Space Matrices
            Mx = inv(J_t)*M*inv(J);
            
            Cx = inv(J_t)*C*inv(J) - Mx*J_dot*inv(J) + inv(J_t)*F*inv(J);
            
            Gx = inv(J_t)*G;
            
            % Error In matrices
            
            % Get system states
            x = obj.pendulum.get_general_position();
            xp = obj.pendulum.get_general_velocities();
            
            u = Mx*v + Cx*xp + Gx + Fa;
            u = J_t*u;
            
        end
        
        function u = get_control_hibrid(obj, xd, xdp, xdpp, Fa, R, r_c, Fa_c, Fa_cp, Fd, Fdp, Fdpp)
            
            % Auxiliar operator
            vx = obj.operator_control_hibrid(xd, xdp, xdpp, Fa, R, r_c);
            vf = obj.operator_control_hibrid_force(Fa_c, Fa_cp, Fd, Fdp, Fdpp);
            % Matrices of the system
            M = obj.pendulum.M_matrix();
            C = obj.pendulum.C_matrix();
            G = obj.pendulum.G_matrix();
            F = obj.pendulum.F_matrix();
            
            J = obj.pendulum.Jacobian_p();
            J_t = obj.pendulum.Jacobian_t();
            J_dot = obj.pendulum.Jacobian_dot();
            
            % Cartesian Space Matrices
            Mx = inv(J_t)*M*inv(J);
            
            Cx = inv(J_t)*C*inv(J) - Mx*J_dot*inv(J) + inv(J_t)*F*inv(J);
            
            Gx = inv(J_t)*G;
            
            % Error In matrices
            
            % Get system states
            x = obj.pendulum.get_general_position();
            xp = obj.pendulum.get_general_velocities();
            
            u = Mx*(R*(vx + vf)) + Cx*xp + Gx + Fa;
            u = J_t*u;
            
        end
        
        function v = operator_control(obj, xd, xdp, xdpp)
            % Get control error value
            xe = obj.error(xd);
            
            % Get Error dot Value
            xep = obj.error_p(xdp);
            
            % Control operator
            v = xdpp + obj.K1*xe + obj.K2*xep;
        end
        
        function v = operator_control_cartesian(obj, xd, xdp, xdpp)
            % Get control error value
            xe = obj.error_cartesian(xd);
            
            % Get Error dot Value
            xep = obj.error_p_cartesian(xdp);
            
            % Control operator
            v = xdpp + obj.K1*xe + obj.K2*xep;
        end
        function v = operator_control_impendance(obj, xd, xdp, xdpp, Fa)
            % Get control error value
            xe = obj.error_cartesian(xd);
            
            % Get Error dot Value
            xep = obj.error_p_cartesian(xdp);
            
            
            % Control operator
            v = xdpp + inv(obj.Km)*(obj.K1*xe + obj.K2*xep + Fa);
        end
        
        function v = operator_control_hibrid(obj, xd, xdp, xdpp, Fa, R, r_c)
            % Get control error value
            xe = xd - inv(R)*(obj.pendulum.get_general_position()-r_c);
            
            % Get Error dot Value
            xep = xdp - inv(R)*obj.pendulum.get_general_velocities();
            
            
            % Control operator
            v = obj.S*xdpp + obj.K1*obj.S*xe + obj.K2*obj.S*xep;
        end
        
        function v = operator_control_hibrid_force(obj, Fa, Fap, Fd, Fdp, Fdpp)
            % Get control error value
            fe = 1*(Fd - Fa);
            
            % Get Error dot Value
            fep = 1*(Fdp - Fap);
            
            
            % Control operator
            v = inv(obj.Km)*(obj.S_1*Fdpp + 1*(obj.KP_force)*obj.S_1*fe + 1*(obj.KD_force)*obj.S_1*fep);
        end
        
        function learning = learning_control(obj, xd)
            xe = 10*obj.error(xd);
            % Learning algorithm
            l1_k = -obj.B*obj.l1_memories + obj.A*obj.e1_memories;
            l2_k = -obj.B*obj.l2_memories + obj.A*obj.e2_memories;
            
            % Vector of learning parameters
            learning = [l1_k;...
                        l2_k];
             
            % Update values memories learning 1
            obj.update_l_memories(learning);
            obj.update_e_memories(xe)
             
        end
        
        function update_l_memories(obj, l)
            for k = length(obj.l1_memories):-1:2
                
                obj.l1_memories(k) = obj.l1_memories(k-1);
                obj.l2_memories(k) = obj.l2_memories(k-1);
            end
            obj.l1_memories(1) = l(1);
            obj.l2_memories(1) = l(2);
        end
        
        function update_e_memories(obj,xe)
            for k = length(obj.e1_memories):-1:2
                
               obj.e1_memories(k) = obj.e1_memories(k-1); 
               obj.e2_memories(k) = obj.e2_memories(k-1);
            end
            obj.e1_memories(1) = xe(1);
            obj.e2_memories(1) = xe(2);
        end
    end
end

