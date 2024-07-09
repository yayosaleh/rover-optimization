function log = pipeline(wheel_diameter, ground_clearance, arm_reach, max_speed, runtime)
    
    % Returns horizontal and vertical impact forces (N)
    function [F_h, F_v] = get_impact_forces(wheel_diameter, max_speed)
        % Convert wheel diameter from mm to drop height in m
        % Convert max speed from km/h to m/s
        h = wheel_diameter / 1000;
        v = max_speed * 0.277778;

        % Constants (SI units)
        % k_ are wheel-to-pivot chain stiffnesses
        m = 50;
        g = 9.81;
        k_x = 4003.77175392;
        k_y = 3530.22082013;

        % Impact forces (N)
        F_h = v * sqrt(k_x * m);
        F_v = sqrt(2 * g * h * k_y * m);
    end
    
    % Returns required battery capacity (Wh)
    function res = get_battery_capacity(runtime)
        % Convert runtime from min to hours
        runtime = runtime / 60;
        
        % Fixed parameters
        P_steering = 29.8;   % power required per steering motor in W
        P_drivetrain = 73.5; % power required per drivetrain motor in W
        P_arm = 290.16;      % power required for entire robotic arm (4 linear actuators and 2 motors) in W
        
        % Battery capacity calculation
        % 4 steering motors running 10% of the time
        % 6 drivetrain motors running 85% of the time
        % 1 robotic arm assembly runing 5% of the time
        res = runtime*(4*0.1*P_steering + 6*0.85*P_drivetrain + 0.05*P_arm);
    end

    % Get impact forces and battery capacity
    [F_h, F_v] = get_impact_forces(wheel_diameter, max_speed);
    battery_capacity = get_battery_capacity(runtime);
    
    %% Get optimization maps

    % Drivetrain
    [drivetrain_map, drivetrain_height, length_y, drivetrain_log] = optimize_drivetrain(F_h, F_v, wheel_diameter);
    
    % Steering
    [steering_map, casing_bolt_d, flange_bolt_size, steering_log] = optimize_steering(F_h, length_y);
           
    % Suspension
    susp = Susp(F_h, F_v, ground_clearance, drivetrain_height, wheel_diameter);
    susp.optimize();

    % Differential
    max_moment = max([susp.M_hp, susp.M_vp]) / 1000; % convert to N m
    [diff_map, diff_log] = optimize_differential(max_moment);

    % Robotic Arm
    arm = optimize_robotic_arm(arm_reach);
    
    %% Construct text files

    % Drivetrain
    drivetrain_map('wheel_diameter') = wheel_diameter;
    drivetrain_map('steering_bolt_diameter') = flange_bolt_size;
    drivetrain_map = Utils.merge_maps(drivetrain_map, susp.rear_steering_and_middle_wheel_mounts_map);
    Utils.map_to_file(drivetrain_map, 'drivetrain.txt');
    
    % Steering
    susp.front_steering_mount_map('casing_bolt_diameter') = casing_bolt_d;
    susp.rear_steering_and_middle_wheel_mounts_map('casing_bolt_diameter') = casing_bolt_d;
    Utils.map_to_file(susp.front_steering_mount_map, 'steering_front_suspension_mount.txt');
    Utils.map_to_file(susp.rear_steering_and_middle_wheel_mounts_map, 'steering_rear_suspension_mount.txt');
    Utils.map_to_file(steering_map, 'Steering.txt');

    % Suspension
    Utils.map_to_file(susp.bolts_and_nuts_map, "suspension_bolts_and_nuts.txt");
    Utils.map_to_file(susp.linkages_map, "suspension_linkages.txt");
    Utils.map_to_file(susp.lower_pivot_housing_map, "suspension_lower_pivot_housing.txt");
    Utils.map_to_file(susp.lower_spacer_map, "suspension_lower_spacer.txt");
    Utils.map_to_file(susp.upper_pivot_housing_map, "suspension_upper_pivot_housing.txt");
    Utils.map_to_file(susp.upper_shaft_map, "suspension_upper_shaft.txt");
    Utils.map_to_file(susp.upper_spacer_map, "suspension_upper_spacer.txt");
    
    % Differential
    diff_map = Utils.merge_maps(diff_map, susp.differential_clevis_map);
    Utils.map_to_file(diff_map, 'Differential_Assembly.txt');

    % Robotic Arm
    Utils.map_to_file(arm.map, 'Arm Length.txt');
    
    %% Construct log file
    
    log = "***General Rover Parameters***" + newline + newline + ...
        "Rover length: 1000 mm" + newline + ...
        "Rover width: 800 mm" + newline + ...
        "Rover height: " + num2str(wheel_diameter + ground_clearance + 200) + " mm" + newline + ...
        "Wheel diameter: " + num2str(wheel_diameter) + " mm" + newline + ...
        "Ground clearance (absolute): " + num2str(wheel_diameter + ground_clearance) + " mm" + newline + ...
        "Arm reach: " + num2str(arm_reach) + " mm" + newline + ...
        "Top speed: " + num2str(max_speed) + " km/h" + newline + ...
        "Runtime: " + num2str(runtime) + " min" + newline + ...
        "Horizontal and vertical impact forces (N): " + num2str(F_h) + ", " + num2str(F_v) + newline + ...
        "Required battery capacity: " + num2str(battery_capacity) + "Wh" + newline + ...
        "Rover chassis mass range: 31.102-34.873 kg" + newline + ...
        "Battery mass (200 Wh/kg specific energy): " + num2str(battery_capacity/200) + " kg" + newline + newline;

    log = log + drivetrain_log + newline + steering_log + newline + susp.log + newline + diff_log + newline + arm.log;
    
end