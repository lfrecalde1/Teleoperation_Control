classdef manipulator_system < matlab.mixin.SetGet
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        % Constant system
        t_s
        g
        % Constant values pendulum 1
        b_1
        m_1
        l_1
        I_z1
        
        % Constant values pendulum 2
        b_2 
        m_2
        l_2
        I_z2
        
        % General vector of the states of the system
        q
    end
    
    methods
        function obj = manipulator_system(L_1, L_2, values, x)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            
            % Constante values pendulum 1
            obj.b_1 = L_1(1);
            obj.m_1 = L_1(2);
            obj.l_1 = L_1(3);
            obj.I_z1 = L_1(4);
            
            
            % Constant values pendulum 2
            obj.b_2 = L_2(1);
            obj.m_2 = L_2(2);
            obj.l_2 = L_2(3);
            obj.I_z2 = L_2(4);
            
            % General values of the system
            obj.g = values(1);
            obj.t_s = values(2);
            
            % States vector of the system
            obj.q = x;
            
        end
        
        function M = M_matrix(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            % Split vector of the states
            q1 = obj.q(1);
            q2 = obj.q(2);
            q1p =obj.q(3);
            q2p = obj.q(4);
            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            
            M_11 = Iz1 + Iz2 + l1^2*m1 +l1^2*m2 + l2^2*m2 + 2*l1*l2*m2*cos(q2);
            M_12 = m2*l2^2 + l1*l2*m2*cos(q2) + Iz2; 
            M_21 = m2*l2^2 + l1*l2*m2*cos(q2) + Iz2;
            M_22 = m2*l2^2 + Iz2;
            
            M = [M_11, M_12;...
                 M_21, M_22];
        end
        
        function M = get_M_matrix(obj, x)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            % Split vector of the states
            q1 = x(1);
            q2 = x(2);
            q1p = x(3);
            q2p = x(4);
            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            
            M_11 = Iz1 + Iz2 + l1^2*m1 +l1^2*m2 + l2^2*m2 + 2*l1*l2*m2*cos(q2);
            M_12 = m2*l2^2 + l1*l2*m2*cos(q2) + Iz2; 
            M_21 = m2*l2^2 + l1*l2*m2*cos(q2) + Iz2;
            M_22 = m2*l2^2 + Iz2;
            
            M = [M_11, M_12;...
                 M_21, M_22];
        end
        
        
        function C = C_matrix(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            % Split vector of the states
            q1 = obj.q(1);
            q2 = obj.q(2);
            q1p =obj.q(3);
            q2p = obj.q(4);
            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            
            % Matrix Generation parts
            C_11 = -l1*l2*m2*q2p*sin(q2);
            C_12 = -l1*l2*m2*sin(q2)*(q1p+q2p);
            C_21 = l1*l2*m2*q1p*sin(q2);
            C_22 = 0;
            
            C = [C_11, C_12;...
                 C_21, C_22];
        end
        
        function C = get_C_matrix(obj, x)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            % Split vector of the states
            q1 = x(1);
            q2 = x(2);
            q1p = x(3);
            q2p = x(4);
            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            
            % Matrix Generation parts
            C_11 = -l1*l2*m2*q2p*sin(q2);
            C_12 = -l1*l2*m2*sin(q2)*(q1p+q2p);
            C_21 = l1*l2*m2*q1p*sin(q2);
            C_22 = 0;
            
            C = [C_11, C_12;...
                 C_21, C_22];
        end
        
        function F = F_matrix(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            % Split vector of the states
            q1 = obj.q(1);
            q2 = obj.q(2);
            q1p =obj.q(3);
            q2p = obj.q(4);
            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            % Matrix Generation parts
            F_11 = b1;
            F_12 = 0;
            F_21 = 0;
            F_22 = b2;
            
            F = [F_11, F_12;...
                F_21, F_22];
        end
        
        function F = get_F_matrix(obj, x)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            % Split vector of the states
            q1 = x(1);
            q2 = x(2);
            q1p = x(3);
            q2p = x(4);
            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            % Matrix Generation parts
            F_11 = b1;
            F_12 = 0;
            F_21 = 0;
            F_22 = b2;
            
            F = [F_11, F_12;...
                F_21, F_22];
        end
        
        function G = G_matrix(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            % Split vector of the states
            q1 = obj.q(1);
            q2 = obj.q(2);
            q1p =obj.q(3);
            q2p = obj.q(4);
            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            % Matrix Generation parts
            G_11 = obj.g*m2*(l2*sin(q1+q2)+l1*sin(q1)) + l1*obj.g*m1*sin(q1);
            G_21 = l2*obj.g*m2*sin(q1+q2);
            
            G = [G_11;...
                 G_21];
        end
        
        function G = get_G_matrix(obj, x)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            % Split vector of the states
            q1 = x(1);
            q2 = x(2);
            q1p = x(3);
            q2p = x(4);
            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            % Matrix Generation parts
            G_11 = obj.g*m2*(l2*sin(q1+q2)+l1*sin(q1)) + l1*obj.g*m1*sin(q1);
            G_21 = l2*obj.g*m2*sin(q1+q2);
            
            G = [G_11;...
                 G_21];
        end
        
        function J_t = Jacobian_p(obj)
            
            % Split vector fo the states
            q1 = obj.q(1);
            q2 = obj.q(2);
            q1p =obj.q(3);
            q2p = obj.q(4);
            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            % Matrix generation Parts
            J11 = l2*cos(q1 + q2) + l1*cos(q1);
            J12 = l2*cos(q1 + q2);
            J21 = l2*sin(q1 + q2) + l1*sin(q1);
            J22 = l2*sin(q1 + q2);
            
            J_t = [J11, J12;...
                   J21, J22];
            
        end
        
        function J_t = get_Jacobian_p(obj, x)
            
            % Split vector fo the states
            q1 = x(1);
            q2 = x(2);
            q1p = x(3);
            q2p = x(4);
            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            % Matrix generation Parts
            J11 = l2*cos(q1 + q2) + l1*cos(q1);
            J12 = l2*cos(q1 + q2);
            J21 = l2*sin(q1 + q2) + l1*sin(q1);
            J22 = l2*sin(q1 + q2);
            
            J_t = [J11, J12;...
                   J21, J22];
            
        end
        
         function J_t = Jacobian_dot(obj)
            
            % Split vector fo the states
            q1 = obj.q(1);
            q2 = obj.q(2);
            q1p =obj.q(3);
            q2p = obj.q(4);
            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            % Matrix generation Parts
            J11 = -q1p*(l2*sin(q1 + q2) + l1*sin(q1)) - q2p*l2*sin(q1 + q2);
            J12 = -l2*sin(q1 + q2)*(q1p + q2p);
            J21 =  q1p*(l2*cos(q1 + q2) + l1*cos(q1)) + q2p*l2*cos(q1 + q2);
            J22 = l2*cos(q1 + q2)*(q1p + q2p);
            
            J_t = [J11, J12;...
                   J21, J22];
         end
        
         function J_t = get_Jacobian_dot(obj, x)
            
            % Split vector fo the states
            q1 = x(1);
            q2 = x(2);
            q1p = x(3);
            q2p = x(4);
            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            % Matrix generation Parts
            J11 = -q1p*(l2*sin(q1 + q2) + l1*sin(q1)) - q2p*l2*sin(q1 + q2);
            J12 = -l2*sin(q1 + q2)*(q1p + q2p);
            J21 =  q1p*(l2*cos(q1 + q2) + l1*cos(q1)) + q2p*l2*cos(q1 + q2);
            J22 = l2*cos(q1 + q2)*(q1p + q2p);
            
            J_t = [J11, J12;...
                   J21, J22];
        end
        
        function J_t = Jacobian_t(obj)
            % Transpose Jacobian
            J = obj.Jacobian_p;
            J_t = J';
        end
        
        function J_t = get_Jacobian_t(obj, x)
            % Transpose Jacobian
            J = obj.get_Jacobian_p(x);
            J_t = J';
        end
        
        function xp = f_model(obj, x, u, u_ext)
            
            
            % System matrices
            M_1 =inv(obj.M_matrix);
            C = obj.C_matrix;
            G = obj.G_matrix;
            F = obj.F_matrix;
            J_t = obj.Jacobian_t;
            % Space State Representation
            A = [zeros(2,2), eye(2,2);...
                zeros(2,2), -M_1*C];
            
            B = [zeros(2,2);...
                M_1];
            
            B_ext = [zeros(2,2);...
                -M_1*J_t];
            aux = [zeros(2,1);...
                -M_1*G];
            
            aux_F = [zeros(2,2), zeros(2,2);...
                zeros(2,2),-M_1*F];
            
            xp =  A*x + B*u + aux + aux_F*x + B_ext*u_ext;
            %xp =  A*x + B*u + aux;

        end
        
        function [x] = system_f(obj, u, u_ext)
            % Sample Time
            T_s = obj.t_s;
            
            % General States System
            x = obj.q;
            
            k1 = obj.f_model(x, u, u_ext);
            k2 = obj.f_model(x + T_s/2*k1, u, u_ext);
            k3 = obj.f_model(x + T_s/2*k2, u, u_ext);
            k4 = obj.f_model(x + T_s*k3, u, u_ext);
            x = x +T_s/6*(k1 +2*k2 +2*k3 +k4);
            
            % Update values system
            obj.q = x;
            
        end
        
        function x = get_values(obj)
           x = obj.q; 
        end
        function x = get_positions(obj)
           x = obj.q;
           x = x(1:2);
        end
        function x = get_velocities(obj)
           x = obj.q;
           x = x(3:4);
        end
        function xp = get_general_velocities(obj)
           J_t = obj.Jacobian_p;
           qp = obj.get_velocities;
           xp = J_t*qp;
        end
        function x = get_general_position(obj)
            % Split vector fo the states
            q1 = obj.q(1);
            q2 = obj.q(2);
            q1p =obj.q(3);
            q2p = obj.q(4);
            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            % Get general Positions of the system
            hx = l2*sin(q1 + q2) + l1*sin(q1);
            hy = -l2*cos(q1 + q2) - l1*cos(q1);
            
            % General vector
            x = [hx;hy];
        end
    end
end

