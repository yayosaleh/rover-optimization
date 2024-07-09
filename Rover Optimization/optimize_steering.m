function [steering_map, casing_bolt_d, Flange_Bolt_Size, log] = optimize_steering(F_impact, length_y)
    
    % ----------------------- Output flange bolt optimization -----------------------

    function [Flange_Bolt_Size, flange_bolt_log] = optimize_flange_bolt(F_impact, length_y)
     
        % Fixed parameters
        x_a = 10; % mm , distance from pivot point of flange to inner bolt
        x_b = 27.5; % mm , distance from pivot point of flange to outer bolt
        S_p_flange = 650; % MPa , SAE Class 9.8 Bolts
        desired_SF_flange_bolt = 2; % Desired safety factor
        
        % Calculating allowable force on the most vulnerable bolt
        
        F_bending = (desired_SF_flange_bolt * F_impact * length_y) / (2 * (x_b + (x_a^2) / x_b));
        
        % Calculating area of bolt required
        
        A_t_flange = F_bending / S_p_flange;
        
        % Selecting bolt size based on stressed area of bolt
        
        if A_t_flange > 6.78
            A_t_flange = 8.78;
            Flange_Bolt_Size = 4; % mm , M4 metric bolt
        elseif A_t_flange < 5.03
            A_t_flange = 5.03;
            Flange_Bolt_Size = 3; % mm , M3 metric bolt
        else
            A_t_flange = 6.78;
            Flange_Bolt_Size = 3.5; % mm , M3.5 metric bolt
        end
        
        % TESTING
        SF_flange_bolt = (A_t_flange*S_p_flange*(2 * (x_b + (x_a^2) / x_b)))/(F_impact*length_y); 

        % logging outputs for flange bolt
    
        %flange_bolt_log = sprintf("Required flange bolt size: %s, Safety factor of flange bolt: %s", Flange_Bolt_Size, SF_flange_bolt);
        flange_bolt_log = sprintf("Required flange bolt size: M%.1f (SF = %.2f)\n", Flange_Bolt_Size, SF_flange_bolt);

    end
    
    % ------------------------ Steering Shaft Optimization -------------------------
    
    function [small_d, large_D, Rb2, steering_shaft_log] = optimize_shaft(F_impact, length_y)
        
        % Process input
        length_a = (length_y / 1000) + 0.029875; % m, length_y_converted_to_m + 0.029875 m (bottom of flange to half bearing)
        
        % Fixed inputs
        length_b = 0.02925;  % m
        Bearing_Thickness = 0.01325; % m
        step_radius = 0.001; % m
        S_y_shaft = 9.1*10^8; % Pa
        
        % Initialize shaft diameters
        small_d = 0.014;   % m
        large_D = 0.020;   % m
        
        % Initialize safety factor
        SF_shaft = 0;
        
        while (SF_shaft < 2)
            small_d = small_d + 0.001;
            large_D = large_D + 0.001;
        
            % Calculate gearbox bearing forces
        
            % Rb1 = F_impact*(length_a + length_b) / length_b; % remove?
            Rb2 = F_impact*(length_a) / length_b;
        
            % Calculating moment on steering shaft
        
            M_max = F_impact * length_a;
            Slope = M_max / length_b;
            M_step = -Slope * (1/2) * Bearing_Thickness + M_max;
        
            %Calculating nominal stress in steering shaft
        
            Stress_nom = (32 * M_step) / (pi * small_d^3);
        
            % -----------Calculating stress concentration-----------------
        
            Diameter_ratio = large_D / small_d;     % D/d
            Radius_ratio = step_radius / small_d;   % r/d
        
            % column 1 = r/d
            if round(Radius_ratio,2) == 0.05
                row = 1;
            elseif round(Radius_ratio,2) == 0.06
                row = 2;
            elseif round(Radius_ratio,2) == 0.07
                row = 3;
            end
        
            % column 2-6 = K_t for D/d of 1.1 to 1.5
            if round(Diameter_ratio,1) == 1.1
                column = 2;
            elseif round(Diameter_ratio,1) == 1.2
                column = 3;
            elseif round(Diameter_ratio,1) == 1.3
                column = 4;
            elseif round(Diameter_ratio,1) == 1.4
                column = 5;
            elseif round(Diameter_ratio,1) == 1.5
                column = 6;
            end
        
            % Stress concentration table
        
            K_t_array = [0.05 1.8480 1.8924 1.9368 1.9813 2.0257;
                         0.06 1.7682 1.8072 1.8462 1.8852 1.9242;
                         0.07 1.7067 1.7416 1.7765 1.8114 1.8463];
        
            K_t = K_t_array(row,column);
        
            %---------------------------------------------------------------------
        
            % Calculating maximum stress in steering shaft
        
            Stress_max = K_t * Stress_nom;
        
            % Calculating safety factor for steering shaft
        
            SF_shaft = S_y_shaft / Stress_max;
        
        end
    
        % logging outputs for steering shaft
    
        steering_shaft_log = sprintf("Required small diameter of shaft: %.2fmm\n" + ...
            "Required large diameter of shaft: %.2fmm\n" +...
            "Safety factor of shaft: %.2f\n", small_d*1000, large_D*1000, SF_shaft);
    end

    % ----------------------- Casing lid bolt optimization -----------------------
    
    function [casing_bolt_d, casing_bolt_log] = optimize_casing_bolt(Rb2)
          
        % Fixed parameters
        S_p_casing = 310; % MPa , SAE Class 4.8 Bolts
        S_y_casing = 240; % MPa , SAE Class 4.8 Bolts
        mu = 0.3; % Coefficient of friction assumed for aluminum to aluminum contact
        
        % Set Rb2 to new parameter
        Force = Rb2;

        % Define values for A_t_casing and corresponding casing_bolt_d
        % Each row represents a pair of A_t_casing and casing_bolt_d
        parameters = [
            % A_t_casing, casing_bolt_d
            3.23, 2.5;
            5.03, 3;
            6.78, 3.5
        ];
        
        % Initialize variable to store the resulting casing_bolt_d
        % optimal_casing_bolt_d = [];
        
        % Iterate over each pair of A_t_casing and casing_bolt_d
        for i = 1:size(parameters, 1)
            % Extract A_t_casing and casing_bolt_d from parameters
            A_t_casing = parameters(i, 1);
            casing_bolt_d = parameters(i, 2);
            
            % Calculating full proof load
            F_i_casing = 9 * (3/4) * A_t_casing * S_p_casing;
        
            % Calculating friction force
            F_f = mu * F_i_casing;
        
            % Calculating area at shear plane
            Shear_area = (1/4) * pi * casing_bolt_d^2;
        
            % Calculating force required to shear 1 bolt
            F_shearbolt = 0.58 * S_y_casing * Shear_area;
        
            % Calculating force required to overcome friction and shear 9 bolts
            F_required = F_f + (9 * F_shearbolt);
        
            % Calculating safety factor for casing lid bolts
            SF_casing_bolt = F_required / Force;
            
            % Check if the safety factor is greater than 2
            if SF_casing_bolt > 2
                % Store the casing_bolt_d value
                % optimal_casing_bolt_d = casing_bolt_d;
                break; % Exit the loop since we found the optimal value
            end
        end
    
        % logging outputs for casing bolt
    
        casing_bolt_log = sprintf("Required casing bolt size: M%.1f (SF = %.2f)\n", casing_bolt_d, SF_casing_bolt);
    end
    
    % Call optimization functions 
    [Flange_Bolt_Size, flange_bolt_log] = optimize_flange_bolt(F_impact, length_y);
    [small_d, large_D, Rb2, steering_shaft_log] = optimize_shaft(F_impact, length_y);
    [casing_bolt_d, casing_bolt_log] = optimize_casing_bolt(Rb2);

    % Populate maps
    steering_map = containers.Map('KeyType', 'char', 'ValueType', 'double');
    steering_map('Flange_Bolt_Size') = Flange_Bolt_Size;
    steering_map('Casing_Bolt_Size') = casing_bolt_d;
    steering_map('Half_Large_D') = 1000*(large_D / 2);
    steering_map('Half_Small_d') = 1000*(small_d / 2);
    
    % Construct log
    geo_log = sprintf("Bearing bore diameter: %.2fmm\n" + "Worm wheel bore diameter: %.2fmm\n" + "Casing hole diameter for shaft: %.2fmm\n" + "Output flange hole diameter for shaft: %.2fmm\n" + "Shaft nut diameter: %.2fmm\n", small_d*1000, large_D*1000, (small_d)*1000 + 1, small_d*1000, small_d*1000);

    log = "***Steering System***" + newline + newline + "Structural:" + newline + flange_bolt_log + casing_bolt_log + steering_shaft_log + newline + "Geometric:" + newline + geo_log;

end