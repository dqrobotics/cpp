%   A = [theta1 ... thetan;
%            d1  ...   dn;
%            a1  ...   an;
%         alpha1 ... alphan]

classdef DQ_kin   
    properties (GetAccess = 'private', SetAccess = 'private')
        % private read and write access respectively
        DH_param;
        DH_convention;
    end
    
    properties
        base;
        effector;
    end

    methods
        function obj = DQ_kin(A,type)
            if nargin == 0
                error('Input: matrix whose columns contain the DH parameters')
            end
            
            obj.DH_param = A;
            
            
            if nargin==1
                obj.DH_convention = 'standard';  
           
            else
                obj.DH_convention = type;
            end
            
            obj.base = DQ(1); %Default base's pose
            obj.effector = DQ(1); %Default effector's pose
            
        end
        
        function set_base(obj,base)
            % dq.set_base(base) sets the pose of the robot's base
            obj.base = DQ(base);
        end
        
        function set_effector(obj,effector)
            % dq.set_effector(effector) sets the pose of the effector
            obj.effector = DQ(effector);
        end
        
        function q = raw_fkm(obj,theta, ith)
            %   dq = raw_fkm(theta) calculates the forward kinematic model and
            %   returns the dual quaternion corresponding to the
            %   last joint (the displacements due to the base and the effector are not taken into account).
            %   theta is the vector of joint variables
            %   This is an auxiliary function to be used mainly with the
            %   Jacobian function.
            if nargin == 3
                n = ith;
            else
                n = size(obj.DH_param,2); %number of links
            end
            
            n_dummy = 0; 
            if size(obj.DH_param,1) == 5
                %There are dummy joints                
                n_dummy = sum(obj.DH_param(5,:) == 1);
            end
            
            if length(theta) ~= (size(obj.DH_param,2) - n_dummy) %number of links - number of dummies
                error('Incorrect number of joint variables');
            end
            
            q_vec = CMEX_raw_fkm(obj.DH_param, obj.DH_convention, theta, n);
            q = DQ(q_vec);
            
        end
       
       function q = fkm(obj,theta, ith)
            %   dq = fkm(theta) calculates the forward kinematic model and
            %   returns the dual quaternion corresponding to the
            %   end-effector pose. This function takes into account the
            %   displacement due to the base's and effector's poses.
            %
            %   theta is the vector of joint variables
            %   
            %   dq = fkm(theta, ith) calculates the FKM up to the ith link.
            
            if nargin == 3
                q = obj.base*obj.raw_fkm(theta, ith); %Takes into account the base displacement
            else
                q = obj.base*obj.raw_fkm(theta)*obj.effector;
            end
       end
            
       function J = jacobian(obj,theta)
            % J = jacobian(theta) returns the dual quaternion Jacobian, where
            % theta is the vector of joint variables
            J = CMEX_jacobian(obj.DH_param, obj.DH_convention, theta);
       end
    end
end