classdef Susp < handle
    
    %%% PROPERTIES %%%
    
    % Static inputs 
    properties(Constant)
        rover_width = 800
        rover_length = 1000
        
        frame_width = 300
        frame_height = 200
        frame_wall_thickness = 1.5
        
        swingarm_thickness = 5
        
        steering_asm_height = 77.5
        rear_steering_mount_neck_height = Susp.linkage_width / 2
        front_steering_mount_neck_height = Susp.linkage_width
        
        middle_wheel_shaft_diameter = 60.3
        middle_wheel_shaft_length = 112.071
        middle_wheel_shaft_overhang = 6.582
        middle_wheel_clearance = 4
        wheel_thickness = 80
        
        linkage_thickness = 18
        linkage_width = 46
        linkage_mount_base_length = 10
        linkage_mount_bolt_diameter = 6.5
        
        upper_shaft_diameter = 50
        upper_shaft_frame_clearance = 4
        upper_shaft_overhang = 4
        
        lower_shaft_diameter = 35
        lower_shaft_overhang = 4.5
        
        upper_bearing_diameter = 65
        upper_bearing_outer_race_inner_diameter = 61
        upper_bearing_thickness = 7
        lower_bearing_diameter = 47
        lower_bearing_outer_race_inner_diameter = 44
        lower_bearing_thickness = 7
        
        upper_ret_ring_inner_diameter = 45.8
        upper_ret_ring_thickness = 2
        lower_ret_ring_inner_diameter = 32.2
        lower_ret_ring_thickness = 1.5

        % Bolt spacing factors (relative to bolt diameters)
        pivot_bolt_spacing = 4
        linkage_bolt_spacing = 1
        
        INCREMENT = 0.1
    end
    
    % Dynamic inputs, optimized dimensions, and outputs
    properties
        % Inputs
        F_h
        F_v
        ground_clearance
        corner_wheel_asm_height
        wheel_diameter
        
        % Optimized dimensions
        upper_shaft_inner_diameter
        pivot_housing_bolt_diameter
        linkage_wall_thickness

        % Outputs
        M_hp
        M_vp
        upper_shaft_map
        linkages_map
        upper_pivot_housing_map
        lower_pivot_housing_map
        upper_spacer_map
        lower_spacer_map
        differential_clevis_map
        front_steering_mount_map
        rear_steering_and_middle_wheel_mounts_map
        bolts_and_nuts_map
        log
    end
    
    %%% DIMENSIONING METHODS %%%
    % Dimensions in mm and degrees
    
    methods
        
        % Returns true if rover is within allowable width
        function [is_valid, frame_to_wheel_width] = validate_rover_width(~)
            target = (Susp.rover_width - Susp.frame_width) / 2;
            frame_to_wheel_width = Susp.upper_shaft_frame_clearance + ...
                Susp.swingarm_thickness + ...
                2 * Susp.linkage_thickness + ...
                2 * Susp.middle_wheel_clearance + ...
                Susp.upper_shaft_overhang + ...
                Susp.middle_wheel_shaft_length + ...
                Susp.wheel_thickness;
            is_valid = frame_to_wheel_width <= target;
        end

        %% Pivot getters

        % Takes "upper" or "lower" prefix and returns pivot bearing diameter
        function res = get_pivot_bearing_diameter(~, prefix)
            if prefix == "upper"
                res = Susp.upper_bearing_diameter;
            else
                res = Susp.lower_bearing_diameter;
            end  
        end
        
        % Takes "upper" or "lower" prefix and returns pivot housing diameter
        function res = get_pivot_housing_diameter(obj, prefix)
            res = obj.get_pivot_bearing_diameter(prefix) + 6 * obj.pivot_housing_bolt_diameter;
        end

        % Takes "upper" or "lower" prefix and returns pivot number of bolts
        function res = get_pivot_housing_num_bolts(obj, prefix)
            r_bearing = obj.get_pivot_bearing_diameter(prefix) / 2;
            D_pb = obj.pivot_housing_bolt_diameter;
            a = Susp.pivot_bolt_spacing;
            res = ceil((2 * pi * (r_bearing + 1.5 * D_pb)) / (a * D_pb));
        end
        
        %% Linkage getters

        % Returns linkage angle (deg) and extended length
        function [angle, extended_length] = get_linkage_angle_and_extended_length(~, height, width)
            angle = rad2deg(atan2(height, width));
            extended_length = sqrt(height^2 + width^2);
        end
        
        % Returns front rocker linkage angle (deg) and extended length
        function [angle, extended_length] = get_front_rocker_linkage_angle_and_extended_length(obj)
            height = (obj.ground_clearance + (0.5 * Susp.frame_height)) - ...
                     (obj.corner_wheel_asm_height + Susp.steering_asm_height + Susp.front_steering_mount_neck_height);
            width = 0.5 * (Susp.rover_length - obj.wheel_diameter);
            [angle, extended_length] = obj.get_linkage_angle_and_extended_length(height, width);
        end
        
        % Returns front rocker length, angle and offset
        function [length, angle, offset] = get_front_rocker_linkage(obj)
            [angle, extended_length] = obj.get_front_rocker_linkage_angle_and_extended_length();
            alpha_rad = deg2rad((angle + 90) / 2);
            offset = Susp.linkage_width / (2 * tan(alpha_rad));
            upper_pivot_housing_radius = obj.get_pivot_housing_diameter("upper") / 2;
            length = extended_length - (upper_pivot_housing_radius + 2 * Susp.linkage_mount_base_length + offset);
        end
       
        % Returns rear rocker length and angle
        function [length, angle] = get_rear_rocker_linkage(obj)
            height = ((obj.ground_clearance + (0.5 * Susp.frame_height)) - ...
                     (obj.corner_wheel_asm_height + Susp.steering_asm_height + ...
                      Susp.rear_steering_mount_neck_height + (Susp.linkage_width / 2)));
            width = Susp.rover_length / 4;
            [angle, extended_length] = obj.get_linkage_angle_and_extended_length(height, width);
            upper_pivot_housing_radius = obj.get_pivot_housing_diameter("upper") / 2;
            lower_pivot_housing_radius = obj.get_pivot_housing_diameter("lower") / 2;
            length = extended_length - (upper_pivot_housing_radius + lower_pivot_housing_radius + ...
                     (2 * Susp.linkage_mount_base_length));
        end
        
        % Returns middle bogie length and angle
        function [length, angle] = get_middle_bogie_linkage(obj)
            height = obj.corner_wheel_asm_height + Susp.steering_asm_height + ...
                     Susp.rear_steering_mount_neck_height + (Susp.linkage_width / 2) - ...
                     (obj.wheel_diameter / 2);
            width = Susp.rover_length / 4;
            [angle, extended_length] = obj.get_linkage_angle_and_extended_length(height, width);
            lower_pivot_housing_radius = obj.get_pivot_housing_diameter('lower') / 2;
            length = extended_length - (lower_pivot_housing_radius + (Susp.middle_wheel_shaft_diameter / 2) + ...
                     (2 * Susp.linkage_mount_base_length));
        end
        
        % Returns rear bogie length
        function length = get_rear_bogie_linkage(obj)
            width = Susp.rover_length / 4;
            lower_pivot_housing_radius = obj.get_pivot_housing_diameter('lower') / 2;
            length = width - (lower_pivot_housing_radius + (obj.wheel_diameter / 2) + ...
                     (Susp.linkage_width / 2) + (2 * Susp.linkage_mount_base_length));
        end
        
        %% Map updates
        
        % Updates upper shaft map
        function update_upper_shaft(obj)
            obj.upper_shaft_map = containers.Map({'shaft_inner_diameter'},[obj.upper_shaft_inner_diameter]);
        end

        % Updates linkages map
        % Returns angles required to dimension pivot housings
        function [a_fr, a_rr, a_mb] = update_linkages(obj)
            
            % Get linkage length and angles
            [l_fr, a_fr] = obj.get_front_rocker_linkage();
            [l_rr, a_rr] = obj.get_rear_rocker_linkage();
            [l_mb, a_mb] = obj.get_middle_bogie_linkage();
            l_rb = obj.get_rear_bogie_linkage();
            
            % Update output map
            linkages = containers.Map('KeyType', 'char', 'ValueType', 'double');
            linkages('wall_thickness') = obj.linkage_wall_thickness;
            linkages('front_rocker_length') = l_fr;
            linkages('front_rocker_angle') = a_fr;
            linkages('rear_rocker_length') = l_rr;
            linkages('rear_rocker_angle') = a_rr;
            linkages('middle_bogie_length') = l_mb;
            linkages('middle_bogie_angle') = a_mb;
            linkages('rear_bogie_length') = l_rb;
            obj.linkages_map = linkages;

        end

        % Takes "upper" or "lower" prefix, angles of housed linkages and updates pivot housing map
        function update_pivot_housing(obj, prefix, interior_angle_1, interior_angle_2)
            pivot_housing = containers.Map('KeyType', 'char', 'ValueType', 'double');
            pivot_housing('housing_diameter') = obj.get_pivot_housing_diameter(prefix);
            pivot_housing('housing_min_wall_thickness') = obj.pivot_housing_bolt_diameter;
            pivot_housing('housing_bolt_diameter') = obj.pivot_housing_bolt_diameter;
            pivot_housing('linkage_separation_angle') = 180 - (interior_angle_1 + interior_angle_2);
            pivot_housing('bolt_placement_radius') = (obj.get_pivot_bearing_diameter(prefix) / 2) + ...
                1.5 * obj.pivot_housing_bolt_diameter;
            pivot_housing('num_bolts') = obj.get_pivot_housing_num_bolts(prefix);
            pivot_housing('linkage_mount_shoulder_depth') = obj.linkage_wall_thickness;
            if prefix == "upper"
                obj.upper_pivot_housing_map = pivot_housing;
            else
                obj.lower_pivot_housing_map = pivot_housing;
            end
        end
        
        % Takes "upper" or "lower" prefix, pivot housing map and updates spacer map
        function update_spacer(obj, prefix, pivot_housing)
            spacer = containers.Map('KeyType', 'char', 'ValueType', 'double');
            spacer('outer_diameter') = pivot_housing('housing_diameter');
            spacer('bolt_diameter') = pivot_housing('housing_bolt_diameter');
            spacer('bolt_placement_radius') = pivot_housing('bolt_placement_radius');
            spacer('num_bolts') = pivot_housing('num_bolts');
            if prefix == "upper"
                obj.upper_spacer_map = spacer;
            else
                obj.lower_spacer_map = spacer;
            end
        end 
        
        % Updates differential clevis map
        function update_differential_clevis(obj)
            obj.differential_clevis_map = containers.Map(keys(obj.upper_spacer_map),...
                values(obj.upper_spacer_map));
            obj.differential_clevis_map('bolt_placement_angle') = ...
                obj.upper_pivot_housing_map('linkage_separation_angle') / 2;
        end

        % Updates front steering mount map
        function update_front_steering_mount(obj)
            
            % Calculate arm length
            [~, angle, offset] = obj.get_front_rocker_linkage();
            arm_length = Susp.linkage_mount_base_length + ...
                (2 + 3 * Susp.linkage_bolt_spacing) * Susp.linkage_mount_bolt_diameter + ...
                offset;
            
            % Update map
            mount = containers.Map('KeyType', 'char', 'ValueType', 'double');
            mount('arm_length') = arm_length;
            mount('angle') = angle;
            mount('linkage_mount_shoulder_depth') = obj.linkage_wall_thickness;
            obj.front_steering_mount_map = mount;

        end
        
        % Updates rear steering and middle wheel mount map
        function update_rear_steering_and_middle_wheel_mounts(obj)
            obj.rear_steering_and_middle_wheel_mounts_map = ...
                containers.Map({'linkage_mount_shoulder_depth'},[obj.linkage_wall_thickness]);
        end
        
        % Updates bolts and nuts map
        function update_bolts_and_nuts(obj)
            
            % Bolt and nut dimensions as factors of bolt diameter (based on standard fasteners)
            SOCKET_WIDTH_FACTOR = 0.6;
            HEAD_DIAMETER_FACTOR = 1.7;
            HEAD_THICKNESS_FACTOR = 1/3;
            NUT_WIDTH_FACTOR = 1.6;
            NUT_THICKNESS_FACTOR = 0.8;
            
            base_diameter = obj.pivot_housing_bolt_diameter;

            % Calculate bolt lengths
            base_length =  2 * Susp.linkage_thickness + NUT_THICKNESS_FACTOR * base_diameter + 2;
            upper_spacer_thickness = Susp.middle_wheel_clearance;
            lower_spacer_thickness = Susp.upper_shaft_overhang + Susp.middle_wheel_clearance + Susp.middle_wheel_shaft_overhang;
            upper_length = base_length + upper_spacer_thickness + Susp.swingarm_thickness;
            lower_length = base_length + lower_spacer_thickness;

            % Update map
            bns = containers.Map('KeyType', 'char', 'ValueType', 'double');
            bns('pivot_diameter') = base_diameter;
            bns('pivot_head_diameter') = base_diameter * HEAD_DIAMETER_FACTOR;
            bns('pivot_head_thickness') = base_diameter * HEAD_THICKNESS_FACTOR;
            bns('pivot_socket_width') = base_diameter * SOCKET_WIDTH_FACTOR;
            bns('pivot_nut_width') = base_diameter * NUT_WIDTH_FACTOR;
            bns('pivot_nut_thickness') = base_diameter * NUT_THICKNESS_FACTOR;
            bns('upper_length') = upper_length;
            bns('lower_length') = lower_length;
            obj.bolts_and_nuts_map = bns;
        
        end

        % Updates all dimension maps
        function update_dimensions(obj)
            
            % Update upper shaft
            obj.update_upper_shaft();
            
            % Update linkages and get angles
            [a_fr, a_rr, a_mb] = obj.update_linkages();

            % Update pivot housings
            obj.update_pivot_housing("upper", a_fr, a_rr);
            obj.update_pivot_housing("lower", a_mb, 0);

            % Update spacers and differential clevis
            obj.update_spacer("upper", obj.upper_pivot_housing_map);
            obj.update_spacer("lower", obj.lower_pivot_housing_map);
            obj.update_differential_clevis();

            % Update mounts
            obj.update_front_steering_mount();
            obj.update_rear_steering_and_middle_wheel_mounts();

            % Update bolts and nuts
            obj.update_bolts_and_nuts();

            % Log results
            obj.append_to_log(newline + "Geometric:")
            obj.log_dimensions(obj.upper_shaft_map, "Connecting Shaft");
            obj.log_dimensions(obj.linkages_map, "Linkages");
            obj.log_dimensions(obj.upper_pivot_housing_map, "Upper Pivot Housing");
            obj.log_dimensions(obj.lower_pivot_housing_map, "Lower Pivot Housing");
            obj.log_dimensions(obj.upper_spacer_map, "Upper Spacer");
            obj.log_dimensions(obj.lower_spacer_map, "Lower Spacer");
            obj.log_dimensions(obj.differential_clevis_map, "Differential Clevis Bolt Pattern");
            obj.log_dimensions(obj.front_steering_mount_map, "Front Suspension-Steering Mount");
            obj.log_dimensions(obj.rear_steering_and_middle_wheel_mounts_map, "Rear Suspension-Steering and Middle Wheel Mount");
            obj.log_dimensions(obj.bolts_and_nuts_map, "Bolts and Nuts");
            
        end
        
    end
    
    %%% OPTIMIZATION METHODS %%%
    % Dimensions in mm and rad
    % Forces in N, moments in N*mm
    % Stresses in MPa

    methods
        
        % Constructor
        function obj = Susp(F_h, F_v, ground_clearance_rel, corner_wheel_asm_height, wheel_diameter)
            obj.F_h = F_h;
            obj.F_v = F_v;
            obj.ground_clearance = ground_clearance_rel + wheel_diameter;
            obj.corner_wheel_asm_height = corner_wheel_asm_height;
            obj.wheel_diameter = wheel_diameter;
        end
        
        % Appends line to log string
        function append_to_log(obj, msg, end_flag)
            
            if nargin < 3
                end_flag = false;
            end
            
            if isempty(obj.log)
                obj.log = "***Suspension System***" + newline;
            end

            obj.log = obj.log + msg + newline;

            if end_flag
                obj.log = obj.log + newline;
            end

        end
        
        % Appends failure message to log string
        function log_failure(obj, src)
            msg = "Error: " + src + " optimization failed!" + newline + "Optimizer aborted.";
            obj.append_to_log(msg, true);
        end
        
        % Appends updated dimensions to log string
        function log_dimensions(obj, map, part_name)
            
            msg = "*" + part_name + "*" + newline;
            keySet = keys(map);
            numKeys = length(keySet);
            
            for i = 1 : numKeys
                
                key = keySet{i};
                value = map(key);
                unit = '';
                
                if contains(key, 'angle')
                    unit = "deg";
                elseif ~contains(key, 'num')
                    unit = "mm";
                end
                
                dim_name = strjoin(split(key, '_'), ' ');
                dim_name = regexprep(dim_name, '^(.)', '${upper($1)}');
                
                msg = msg + dim_name + ": " + num2str(value) + unit;
                
                if i ~= numKeys
                    msg = msg + newline;
                end

            end
        
            obj.append_to_log(msg);

        end

        % Returns optimized upper shaft inner diameter or -1 in case of failure
        function [D_i, SF] = optimize_upper_shaft(~, F_x, F_y, SF, D_g)
            
            % Resultant force, stress concentration factor and yield strength
            R_p = sqrt(F_x^2 + F_y^2);
            K_t = 2;
            S_y = 275; % 6061 T6 Al tensile yield strength = 275 MPa
    
            % Stress function
            function res = tau_max(D_i)
                A_min = (pi / 4) * (D_g^2 - D_i^2);
                res = K_t * 4 * R_p / A_min;
            end
    
            % SF function
            function res = achieved_SF(D_i)
                res = 0.58 * S_y / tau_max(D_i);
            end
    
            % Optimization loop
            D_i = D_g - Susp.INCREMENT;
            while achieved_SF(D_i) < SF
                if D_i <= 0
                    D_i = -1;
                    return;
                else
                    D_i = D_i - Susp.INCREMENT;
                end
            end
            
            % Update SF
            SF = achieved_SF(D_i);

        end

        % Returns moment about pivot
        function M_p = get_pivot_moment(~, F_x, F_y, theta, L_ext, D_w)
            M_p = abs(-F_x * sin(theta) * (D_w / (2 * cos(theta)) + L_ext) + F_y * cos(theta) * L_ext);
        end
        
        % Returns optimized pivot bolt diameter
        function [D_pb, SF] = optimize_upper_pivot_bolt(~, F_x, F_y, M_p, SF, theta, r_bearing)
    
            % Bolt spacing and tensile yield strength
            a = Susp.pivot_bolt_spacing;
            S_y = 205; % 316 SS tensile yield strength = 205 MPa
            
            % Stress function
            function res = tau_avg(D_pb)
                tau_avg_d = a * (F_x * cos(theta) + F_y * sin(theta)) / (pi^2 * D_pb * (r_bearing + 1.5 * D_pb));
                tau_avg_t = 2 * a * M_p / (pi^2 * D_pb * (r_bearing + 1.5 * D_pb)^2);
                res = tau_avg_d + tau_avg_t;
            end

            % SF function
            function res = achieved_SF(D_pb)
                res = (0.58 * S_y) / tau_avg(D_pb);
            end
            
            % Optimization loop
            D_pb = Susp.INCREMENT;
            while achieved_SF(D_pb) < SF
                D_pb = D_pb + Susp.INCREMENT;
            end
    
            % Round optimized diameter to nearest integer 
            D_pb = ceil(D_pb);
            
            % Update SF
            SF = achieved_SF(D_pb);

        end
        
        % Returns optimized linkage wall thickness or -1 in case of failure
        function [t_w, SF_l, SF_m] = optimize_linkage_and_mount(~, F_x, F_y, SF, theta, L, l_sm, D_w, t_l, w_l, D_lb)
    
            % Forces and moment at critical point, and tensile yield strength
            N = abs(-F_x * cos(theta) - F_y * sin(theta));
            V = abs(F_x * sin(theta) - F_y * cos(theta));
            M = abs(-F_x * sin(theta) * (D_w / (2 * cos(theta)) + l_sm + L - 1.5 * D_lb) + F_y * cos(theta) * (l_sm + L - 1.5 * D_lb));
            S_y = 275; % 6061 T6 Al tensile yield strength = 275 MPa

            % Stress functions
            function res = vM_stress(sigma, tau)
                res = sqrt(sigma^2 + 3*tau^2);
            end
            function [vM_stress_l, vM_stress_m] = max_stresses(t_w)
                % Cross-sectional properties
                I_tube = (t_l * w_l^3 - (t_l - 2 * t_w) * (w_l - 2 * t_w)^3) / 12;
                I_hole = (t_w * D_lb^3 / 12);
                I_l = I_tube - 2 * I_hole;
                I_m = ((t_l - 2 * t_w) * (w_l - 2 * t_w)^3 - (t_l - 2 * t_w) * D_lb^3) / 12;
                A_l = t_l * w_l - (t_l - 2 * t_w) * (w_l - 2 * t_w) - 2 * t_w * D_lb;
                A_m = (t_l - 2 * t_w) * (w_l - 2 * t_w) - (t_l - 2 * t_w) * D_lb;
                Q_m = ((w_l - D_lb) / 2 - t_w) * (t_l - 2 * t_w) * (1 / 2 * ((w_l - D_lb) / 2 - t_w) + D_lb / 2);
                Q_l = (1 / 2) * t_w * (t_l + w_l - D_lb - 2 * t_w) * (w_l - t_w);
    
                % Normal and shear stress
                sigma_l_max = N / A_l + (M * w_l / 2) / I_l;
                sigma_m_max = N / A_m + (M * (w_l - 2 * t_w) / 2) / I_m;
                tau_l_max = (V * Q_l) / (I_l * 2 * t_w);
                tau_m_max = (V * Q_m) / (I_m * (t_l - 2 * t_w));

                vM_stress_l = vM_stress(sigma_l_max, tau_l_max);
                vM_stress_m = vM_stress(sigma_m_max, tau_m_max);

            end
    
            % Optimization loop
            t_w = Susp.INCREMENT;
            while true
                [vM_stress_l, vM_stress_m] = max_stresses(t_w);
                if (S_y / vM_stress_l >= SF && S_y / vM_stress_m >= SF)
                    break;
                elseif (t_w >= t_l / 2)
                    t_w = -1;
                    return;
                else
                    t_w = t_w + Susp.INCREMENT;
                end
            end
            
            % Update SFs
            SF_l = S_y / vM_stress_l;
            SF_m = S_y / vM_stress_m;

        end

        % Calls all optimizers and updates dimensions
        function optimize(obj)

            %% Init. impact forces & SF
            
            m = 50;
            g = 9.81;
            SF = 2;
            
            % Horizontal (crash)
            F_hx = obj.F_h;
            F_hy = (m*g) / 6;

            % Vertical (free fall)
            F_vx = 0;
            F_vy = obj.F_v + m*g;
            
            %% Upper shaft optimization
            
            % Run optimizer for both loading cases
            D_g = Susp.upper_ret_ring_inner_diameter;
            [D_i_h, SF_us_h]= obj.optimize_upper_shaft(F_hx, F_hy, SF, D_g);
            [D_i_v, SF_us_v] = obj.optimize_upper_shaft(F_vx, F_vy, SF, D_g);
            
            % Check for failure
            if D_i_h == -1 || D_i_v == -1
                obj.log_failure("upper shaft");
                return
            end
            
            % Store worst-case result
            if D_i_h < D_i_v
                D_i = D_i_h;
                SF_us = SF_us_h;
            else
                D_i = D_i_v;
                SF_us = SF_us_v;
            end
            obj.upper_shaft_inner_diameter = D_i;

            % Log result
            msg = newline + "Structural:";
            obj.append_to_log(msg);
            msg = "Required connecting shaft inner diameter: " + num2str(D_i) + "mm" + " (SF = " + num2str(SF_us) + ")";
            obj.append_to_log(msg);
            
            %% Optimize upper pivot bolts

            % Dimension inputs
            [theta, L_ext] = obj.get_front_rocker_linkage_angle_and_extended_length();
            theta = deg2rad(theta);
            D_w = obj.wheel_diameter;
            r_bearing = obj.get_pivot_bearing_diameter("upper") / 2;
            
            % Moment inputs
            obj.M_hp = obj.get_pivot_moment(F_hx, F_hy, theta, L_ext, D_w);
            obj.M_vp = obj.get_pivot_moment(F_vx, F_vy, theta, L_ext, D_w);

            % Run optimizer for both loading cases and store worst-case result
            [D_pb_h, SF_pb_h] = obj.optimize_upper_pivot_bolt(F_hx, F_hy, obj.M_hp, SF, theta, r_bearing);
            [D_pb_v, SF_pb_v] = obj.optimize_upper_pivot_bolt(F_vx, F_vy, obj.M_vp, SF, theta, r_bearing); 
            if D_pb_h > D_pb_v
                D_pb = D_pb_h;
                SF_pb = SF_pb_h;
            else
                D_pb = D_pb_v;
                SF_pb = SF_pb_v;
            end
            obj.pivot_housing_bolt_diameter = D_pb;
            
            % Log result
            msg = "Required upper pivot bolt diameter and quantity: M" + num2str(D_pb) + ", " + ... 
                num2str(obj.get_pivot_housing_num_bolts("upper")) + " (SF = " + num2str(SF_pb) + ")";
            obj.append_to_log(msg);

            %% Optimize front rocker linkage and mount
            
            % Dimension inputs
            t_l = Susp.linkage_thickness;
            w_l = Susp.linkage_width;
            D_lb = Susp.linkage_mount_bolt_diameter;
            [L, ~, offset] = obj.get_front_rocker_linkage();
            l_sm = Susp.linkage_mount_base_length + offset;

            % Run optimizer for both loading cases
            [t_w_h, SF_l_h, SF_m_h] = obj.optimize_linkage_and_mount(F_hx, F_hy, SF, theta, L, l_sm, D_w, t_l, w_l, D_lb);
            [t_w_v, SF_l_v, SF_m_v] = obj.optimize_linkage_and_mount(F_vx, F_vy, SF, theta, L, l_sm, D_w, t_l, w_l, D_lb);
            
            % Check for failure
            if t_w_h == -1 || t_w_v == -1
                obj.log_failure("front rocker linkage and mount");
                return
            end

            % Store worst-case result
            if t_w_h > t_w_v
                t_w = t_w_h;
                SF_l = SF_l_h;
                SF_m = SF_m_h;
            else
                t_w = t_w_v;
                SF_l = SF_l_v;
                SF_m = SF_m_v;
            end
            obj.linkage_wall_thickness = t_w;

            % Log result
            msg = "Required linkage wall thickness: " + num2str(t_w) + "mm " + "(linkage SF = " + ...
                num2str(SF_l) + ", mount SF = " + num2str(SF_m) + ")";
            obj.append_to_log(msg);

            %% Dimensioning
            obj.update_dimensions();

        end

    end

end