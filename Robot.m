classdef Robot < handle

    properties
        % Save vector of tubes and size of vector
        tubes = []
        num_tubes = 0

        % Save link lengths, phi values, and kappa values vectors (1 x num
        % links)
        lls = []
        phi = []
        kappa = []
    end

    methods
        % Constructor. This creates an instance of the robot class 
        % Must pass in a vector of tubes
        function self = Robot(tubes)
            self.tubes = tubes;
            self.num_tubes = size(tubes, 2);
        end

        % Here we calculate the kinematics of a full CTR
        % Pass in the raw joint variables
        % Return transformation matrix that goes from home frame to end
        % effector frame
        % See functions below for each step
        function T = fkin(self, q_var)  
            % First we get the rho and theta avlues from q_var
            rho = get_rho_values(self, q_var);
            theta = get_theta(self, q_var);

            % Next, we use rho to get the link lengths
            self.lls = get_links(self, rho);

            % Now we calculate the phi and kappa values
            [self.phi,self.kappa] = calculate_phi_and_kappa(self, theta, rho);

            % Finally we calculate the base to end effector transform
            T = calculate_transform(self, self.lls, self.phi, self.kappa);
        end

        % Get rho values from joint positions
        % Return rho (1 x i vector, i is num tubes)
        function rho = get_rho_values(self, q_var)
            % Here we extract rho values from the q_var vector
            
            % Initialize a vector to hold the result
            rho = zeros([1 self.num_tubes]);

            for i=2:self.num_tubes
                rho(i) = (q_var(i) - q_var(1)) * 10^-3;
            end

        end

        % Function to find the link lengths, in order
        % Returns link lengths (1 x j vector, where j is num links)
        function s = get_links(self, rho)
            % ********************************************************
            %                          TODO
            % ********************************************************
            for i = 1:self.num_tubes
                curvature(i) = rho(i);
                terminal(i) = rho(i) +  self.tubes(i).d;
            end

            transition_points = sort(cat(2,curvature,terminal));
            for i = 1:length(transition_points) - 1
                s(i) = transition_points(i+1) - transition_points(i);
            end
        end

        % Function to get theta values
        % Returns theta (1 x j vector where j is num links)
        function theta = get_theta(self, q_var)
            % Here we extract theta values from the q_var vector

            % Initialize a vector to hold the result
            theta = zeros([1 self.num_tubes]);

            for i=1:self.num_tubes
                theta(i) = deg2rad(q_var(i+self.num_tubes));
            end
        end
        % Function to calcualte phi values for two or three tube
        % configurations
        % Should return phi (1 x j vector, where j is num links)
        % and K (1 x j vector)
        function [phi,K] = calculate_phi_and_kappa(self, theta, rho)
            % ********************************************************
            %                          TODO
            % ********************************************************
            if (self.num_tubes == 2)
                chi(1) = self.tubes(1).E*pi*0.25*((self.tubes(1).od/2)^4 - ...
                    (self.tubes(1).id/2)^4)*self.tubes(1).k*cos(theta(1)) + ... 
                    self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                    (self.tubes(2).id/2)^4)*0*cos(theta(2));
                
                gamma(1) = self.tubes(1).E*pi*0.25*((self.tubes(1).od/2)^4 - ...
                    (self.tubes(1).id/2)^4)*self.tubes(1).k*sin(theta(1)) + ... 
                    self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                    (self.tubes(2).id/2)^4)*0*sin(theta(2));
                
                denominator = self.tubes(1).E*pi*0.25*((self.tubes(1).od/2)^4 - ...
                    (self.tubes(1).id/2)^4) + ... 
                    self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                    (self.tubes(2).id/2)^4);
                
                chi(1) = chi(1)/denominator;
                gamma(1) = gamma(1)/denominator;
                
                chi(2) = self.tubes(1).E*pi*0.25*((self.tubes(1).od/2)^4 - ...
                    (self.tubes(1).id/2)^4)*self.tubes(1).k*cos(theta(1)) + ... 
                    self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                    (self.tubes(2).id/2)^4)*self.tubes(2).k*cos(theta(2));
                
                gamma(2) = self.tubes(1).E*pi*0.25*((self.tubes(1).od/2)^4 - ...
                    (self.tubes(1).id/2)^4)*self.tubes(1).k*sin(theta(1)) + ... 
                    self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                    (self.tubes(2).id/2)^4)*self.tubes(2).k*sin(theta(2));
                
                denominator = self.tubes(1).E*pi*0.25*((self.tubes(1).od/2)^4 - ...
                    (self.tubes(1).id/2)^4) + ... 
                    self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                    (self.tubes(2).id/2)^4);
                
                chi(2) = chi(2)/denominator;
                gamma(2) = gamma(2)/denominator;
                
                chi(3) = self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                    (self.tubes(2).id/2)^4)*self.tubes(2).k*cos(theta(2));
                
                gamma(3) = self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                    (self.tubes(2).id/2)^4)*self.tubes(2).k*sin(theta(2));

                denominator = self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                    (self.tubes(2).id/2)^4);
                
                chi(3) = chi(3)/denominator;
                gamma(3) = gamma(3)/denominator;
               
            end
        
            if self.num_tubes == 3
                
                chi(1) = self.tubes(1).E*pi*0.25*((self.tubes(1).od/2)^4 - ...
                    (self.tubes(1).id/2)^4)*self.tubes(1).k*cos(theta(1)) + ... 
                    self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                    (self.tubes(2).id/2)^4)*0*cos(theta(2));
                
                gamma(1) = self.tubes(1).E*pi*0.25*((self.tubes(1).od/2)^4 - ...
                    (self.tubes(1).id/2)^4)*self.tubes(1).k*sin(theta(1)) + ... 
                    self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                    (self.tubes(2).id/2)^4)*0*sin(theta(2));
                
                denominator = self.tubes(1).E*pi*0.25*((self.tubes(1).od/2)^4 - ...
                    (self.tubes(1).id/2)^4) + ... 
                    self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                    (self.tubes(2).id/2)^4) + self.tubes(3).E*pi*0.25*((self.tubes(3).od/2)^4 - ...
                    (self.tubes(3).id/2)^4);
                
                chi(1) = chi(1)/denominator;
                gamma(1) = gamma(1)/denominator;

                chi(2) = self.tubes(1).E*pi*0.25*((self.tubes(1).od/2)^4 - ...
                    (self.tubes(1).id/2)^4)*self.tubes(1).k*cos(theta(1)) + ... 
                    self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                    (self.tubes(2).id/2)^4)*self.tubes(2).k*cos(theta(2));
                
                gamma(2) = self.tubes(1).E*pi*0.25*((self.tubes(1).od/2)^4 - ...
                    (self.tubes(1).id/2)^4)*self.tubes(1).k*sin(theta(1)) + ... 
                    self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                    (self.tubes(2).id/2)^4)*self.tubes(2).k*sin(theta(2));
                
                denominator = self.tubes(1).E*pi*0.25*((self.tubes(1).od/2)^4 - ...
                    (self.tubes(1).id/2)^4) + ... 
                    self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                    (self.tubes(2).id/2)^4) + self.tubes(3).E*pi*0.25*((self.tubes(3).od/2)^4 - ...
                    (self.tubes(3).id/2)^4);

                chi(2) = chi(2)/denominator;
                gamma(2) = gamma(2)/denominator;

                if (rho(3) < self.tubes(1).d + rho(1))

                    chi(3) = self.tubes(1).E*pi*0.25*((self.tubes(1).od/2)^4 - ...
                    (self.tubes(1).id/2)^4)*self.tubes(1).k*cos(theta(1)) + ... 
                    self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                    (self.tubes(2).id/2)^4)*self.tubes(2).k*cos(theta(2)) + ... 
                    self.tubes(3).E*pi*0.25*((self.tubes(3).od/2)^4 - ...
                    (self.tubes(3).id/2)^4)*self.tubes(3).k*cos(theta(3));
                
                    gamma(3) = self.tubes(1).E*pi*0.25*((self.tubes(1).od/2)^4 - ...
                        (self.tubes(1).id/2)^4)*self.tubes(1).k*sin(theta(1)) + ... 
                        self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                        (self.tubes(2).id/2)^4)*self.tubes(2).k*sin(theta(2)) + ... 
                    self.tubes(3).E*pi*0.25*((self.tubes(3).od/2)^4 - ...
                    (self.tubes(3).id/2)^4)*self.tubes(3).k*sin(theta(3));
                
                    denominator = self.tubes(1).E*pi*0.25*((self.tubes(1).od/2)^4 - ...
                        (self.tubes(1).id/2)^4) + ... 
                        self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                        (self.tubes(2).id/2)^4) + self.tubes(3).E*pi*0.25*((self.tubes(3).od/2)^4 - ...
                        (self.tubes(3).id/2)^4);

                    chi(3) = chi(3)/denominator;
                    gamma(3) = gamma(3)/denominator;

                    chi(4) = self.tubes(3).E*pi*0.25*((self.tubes(3).od/2)^4 - ...
                        (self.tubes(3).id/2)^4)*self.tubes(3).k*cos(theta(3)) + ... 
                        self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                        (self.tubes(2).id/2)^4)*self.tubes(2).k*cos(theta(2));
                
                    gamma(4) = self.tubes(3).E*pi*0.25*((self.tubes(3).od/2)^4 - ...
                        (self.tubes(3).id/2)^4)*self.tubes(3).k*sin(theta(3)) + ... 
                        self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                        (self.tubes(2).id/2)^4)*self.tubes(2).k*sin(theta(2));
                
                    denominator = self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                        (self.tubes(2).id/2)^4) + self.tubes(3).E*pi*0.25*((self.tubes(3).od/2)^4 - ...
                        (self.tubes(3).id/2)^4);

                    chi(4) = chi(4)/denominator;
                    gamma(4) = gamma(4)/denominator;

                else

                    chi(3) = self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                    (self.tubes(2).id/2)^4)*self.tubes(2).k*cos(theta(2)) + ... 
                    self.tubes(3).E*pi*0.25*((self.tubes(3).od/2)^4 - ...
                    (self.tubes(3).id/2)^4)*0*cos(theta(3));
                
                    gamma(3) = self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                        (self.tubes(2).id/2)^4)*self.tubes(2).k*sin(theta(2)) + ... 
                    self.tubes(3).E*pi*0.25*((self.tubes(3).od/2)^4 - ...
                    (self.tubes(3).id/2)^4)*0*sin(theta(3));
                
                    denominator = self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                        (self.tubes(2).id/2)^4) + self.tubes(3).E*pi*0.25*((self.tubes(3).od/2)^4 - ...
                        (self.tubes(3).id/2)^4);

                    chi(3) = chi(3)/denominator;
                    gamma(3) = gamma(3)/denominator;

                    chi(4) = self.tubes(3).E*pi*0.25*((self.tubes(3).od/2)^4 - ...
                        (self.tubes(3).id/2)^4)*self.tubes(3).k*cos(theta(3)) + ... 
                        self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                        (self.tubes(2).id/2)^4)*self.tubes(2).k*cos(theta(2));
                
                    gamma(4) = self.tubes(3).E*pi*0.25*((self.tubes(3).od/2)^4 - ...
                        (self.tubes(3).id/2)^4)*self.tubes(3).k*sin(theta(3)) + ... 
                        self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                        (self.tubes(2).id/2)^4)*self.tubes(2).k*sin(theta(2));
                
                    denominator = self.tubes(2).E*pi*0.25*((self.tubes(2).od/2)^4 - ...
                        (self.tubes(2).id/2)^4) + self.tubes(3).E*pi*0.25*((self.tubes(3).od/2)^4 - ...
                        (self.tubes(3).id/2)^4);

                    chi(4) = chi(4)/denominator;
                    gamma(4) = gamma(4)/denominator;

                end
                chi(5) = self.tubes(3).E*pi*0.25*((self.tubes(3).od/2)^4 - ...
                        (self.tubes(3).id/2)^4)*self.tubes(3).k*cos(theta(3));
                
                gamma(5) = self.tubes(3).E*pi*0.25*((self.tubes(3).od/2)^4 - ...
                        (self.tubes(3).id/2)^4)*self.tubes(3).k*sin(theta(3));
                
                denominator = self.tubes(3).E*pi*0.25*((self.tubes(3).od/2)^4 - ...
                        (self.tubes(3).id/2)^4);

                chi(5) = chi(5)/denominator;
                gamma(5) = gamma(5)/denominator;
            end

            for j=1:2*self.num_tubes - 1
                K(j) = (chi(j)^2 + gamma(j)^2)^0.5;
                phi_t(j) = atan2(gamma(j), chi(j));
                if j > 1
                    phi(j) = phi_t(j) - phi_t(j-1);
                else
                    phi(j) = phi_t(j);
                end
            end

        end

        % Take in all robot dependent parameters (lls, phi, kappa) and
        % compelte the robot independent constant curvature kinamtatics
        % Returns a 4x4 transformation matrix from base frame to end
        % effector
        function T = calculate_transform(self, s, phi, K)
            % ********************************************************
            %                          TODO
            % ********************************************************
            for i = 1:2*self.num_tubes - 1
          
                t_matrix_1r(i, 1:4) = [cos(phi(i))*cos(K(i)*s(i)), -sin(phi(i)), cos(phi(i))*sin(K(i)*s(i)), ... 
                cos(phi(i))*(1 - cos(K(i)*s(i)))/K(i)];
                t_matrix_2r(i, 1:4) = [sin(phi(i))*cos(K(i)*s(i)), ...
                cos(phi(i)), sin(phi(i))*sin(K(i)*s(i)), sin(phi(i))*(1 - cos(K(i)*s(i)))/K(i)];
                t_matrix_3r(i, 1:4) = [-sin(K(i)*s(i)), 0, cos(K(i)*s(i)), sin(K(i)*s(i))/K(i)];
                t_matrix_4r(i, 1:4) = [0, 0, 0, 1];
            end
            
            T0_1 = [t_matrix_1r(1,1:4);t_matrix_2r(1,1:4);t_matrix_3r(1,1:4); t_matrix_4r(1,1:4)];
            T1_2 = [t_matrix_1r(2,1:4);t_matrix_2r(2,1:4);t_matrix_3r(2,1:4); t_matrix_4r(2,1:4)];
            T2_3 = [t_matrix_1r(3,1:4);t_matrix_2r(3,1:4);t_matrix_3r(3,1:4); t_matrix_4r(3,1:4)];
            T0_3 = T0_1*T1_2*T2_3;
            T = T0_3;
            if(self.num_tubes == 3)
                T3_4 = [t_matrix_1r(4,1:4);t_matrix_2r(4,1:4);t_matrix_3r(4,1:4); t_matrix_4r(4,1:4)];
                T4_5 = [t_matrix_1r(5,1:4);t_matrix_2r(5,1:4);t_matrix_3r(5,1:4); t_matrix_4r(5,1:4)];
                T = T0_3*T3_4*T4_5;
            end
        end

        % function T = transformation_matrix(s, phi, K)
        % 
        %     T = [cos(phi)*cos(K*s), -sin(phi), cos(phi)*sin(K*s), ... 
        %         cos(phi)*(1 - cos(K*s))/K; sin(phi)*cos(K*s), ...
        %         cos(phi), sin(phi)*sin(K*s), sin(phi)*(1 - cos(K*s))/K; 
        %         -sin(K*s), 0, cos(K*s), sin(K*s)/K;
        %         0, 0, 0, 1];
        %     disp(T)
        % end
    end
end

