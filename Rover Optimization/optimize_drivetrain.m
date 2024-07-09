function [drivetrain_map, drivetrain_height, length_y, log] = optimize_drivetrain(F_impact_x, F_impact_y, wheel_diameter)
    
    %Geometric Equations:
    function [drivetrain_height, length_y, upright_height, geometric_log] = geometric_drivetrain_heights(wheel_diameter)

    drivetrain_height = wheel_diameter + 10 + 40; %mm
    length_y = (wheel_diameter/2) + 10 + 40; %mm
    upright_height = (wheel_diameter/2) + 30; %mm

    geometric_log = sprintf('Upright height = %.3fmm \n', upright_height);

    end
    %----------------Drivetrain Optimzation----------------%
    
    %Fixed Gloval Variables
    stress_sy = 160.08; % MPa, Aluminium Yeild Strength of 276Mpa * 0.58
    
    function [motor_housing_thickness, motor_housing_log] = optimize_motor_housing(F_impact_x)
    %----------------Motor Housing----------------%
    %Local Parameters
    motor_housing_od = 60.3; % in mm
    
    % Define the range of ID values to iterate through
    initial_ID = 60.2; % in mm (minimum of 0.1mm thickness)
    step_size = 0.01; % in mm
    final_ID = 0; % End condition will be when ID <= final_ID
    
    % Initialize an empty array to store safety factors
    safety_factors = [];
    
    % Start the for loop
    for drivetrain_motor_housing_id = initial_ID:-step_size:final_ID
        A = (pi/4) * ((motor_housing_od^2) - (drivetrain_motor_housing_id^2)); % Calculate cross-sectional area
        stress_y_allow = F_impact_x / A; % Calculate allowable stress
        SF_motor_housing = stress_sy / stress_y_allow; % Calculate safety factor
        if SF_motor_housing >= 4
            %motor_housing_log = sprintf('Safety factor of %.2f achieved at ID = %.2f mm\n',SF_motor_housing, drivetrain_motor_housing_id);
            break; % Exit the loop once safety factor of 2 is achieved
        end
    end
    
    %solve for motor housing thickness 
        motor_housing_thickness = motor_housing_od-drivetrain_motor_housing_id;
     motor_housing_log = sprintf('Motor housing wall thickness = %.3fmm (SF = %.2f)\n', motor_housing_thickness, SF_motor_housing);
    end
   
    %----------------Motor Housing Optimzation----------------%
 
    function [drivetrain_wheel_bolt_dia, wheel_bolt_log] = optimize_wheel_bolt(F_impact_x)
    %-----------------Wheel Bolt Diameter----------------%
    
    % Fixed parameters
    S_p_wheel = 310; % MPa , SAE Class 4.8 Bolts
    S_y_wheel = 240; % MPa , SAE Class 4.8 Bolts
    mu = 0.61; % Coefficient of friction assumed for steel to aluminum contact
    
    % Define values for A_t_casing and corresponding wheel_bolt_dia
    % Each row represents a pair of A_t_wheel and wheel_bolt_dia
    parameters = [
        % A_t_wheel, drivetrain_wheel_bolt_dia
        3.23, 2.5;
        5.03, 3;
        6.78, 3.5
    ];
    
    % Initialize variable to store the resulting wheel_bolt_d
    drivetrain_wheel_bolt_dia = [];
    
    % Iterate over each pair of A_t_wheel and wheel_bolt_d
    for i = 1:size(parameters, 1)
        % Extract A_t_wheel and wheel_bolt_d from parameters
        A_t_wheel = parameters(i, 1);
        wheel_bolt_dia = parameters(i, 2);
        
        % Calculating full proof load
        F_i_wheel = 4 * (3/4) * A_t_wheel * S_p_wheel;
    
        % Calculating friction force
        F_f = mu * F_i_wheel;
    
        % Calculating area at shear plane
        Shear_area = (1/4) * pi * wheel_bolt_dia^2;
    
        % Calculating force required to double shear 1 bolt
        F_shearbolt = 2*0.58 * S_y_wheel * Shear_area;
    
        % Calculating force required to overcome friction and shear 4 bolts
        F_required = F_f + (4 * F_shearbolt);
    
        % Calculating safety factor for wheel bolts
        SF_wheel_bolt = F_f / F_impact_x;
        
        % Check if the safety factor is greater than 2
        if SF_wheel_bolt > 2
            % Store the insert_bolt_d value
            drivetrain_wheel_bolt_dia = wheel_bolt_dia;
            wheel_bolt_log = sprintf('Wheel bolt size of M%.2f (SF = %.2f)\n', drivetrain_wheel_bolt_dia,SF_wheel_bolt);
            break; % Exit the loop since we found the optimal value
        end
    end
     end
    
    %----------------Wheel Bolt Optimzation----------------%

    function [drivetrain_wheelplate_thickness, wheel_plate_log] = optimize_wheel_plate(F_impact_x)
    %----------------Wheel Plate Thickness----------------%
    %Initialize thickness
    plate_thickness = 1; % mm
    
    while true %continuously incrases thickness until saftey factor is achieved
        stress_applied = F_impact_x / (drivetrain_wheel_bolt_dia * plate_thickness);
        SF_wheel_plate = stress_sy / stress_applied;
        
        if SF_wheel_plate >= 2 %end loop when safety factor is achieved
     wheel_plate_log = sprintf('Wheel plate thickness %.2f mm (SF = %.2f)\n', plate_thickness, SF_wheel_plate);
            drivetrain_wheelplate_thickness = plate_thickness;
            break;
        end
        
        plate_thickness = plate_thickness + 0.1; % Increase thickness by 0.1 mm
    end
    end
    
    %----------------Wheel Plate Optimzation----------------%

    function [weld_leg, weld_leg_log] = optimize_weld_leg(F_impact_y)
    %----------------Weld Leg Height----------------%
    % Local Parameters
    upright_length = 95.98;  % mm not the same as upright height
    stress_sy_filler = 75.9978;  % MPa filler rod yeild strength of 131.04times 0.58)
    
    % Initial thickness and increment
    throat_area_weld = 0.05;   % mm
    t_increment_weld = 0.001;  % mm
    
    % Loop until safety factor of 4 is achieved
    while true
        % Calculate Ix
        I_x_weld = 42666.57 * throat_area_weld;  % mm^4
        
        % Calculate sigma
        M_c_weld = F_impact_y * upright_length; % N*mm
        sigma_weld = (M_c_weld / I_x_weld) ; % Convert to MPa
        
        % Calculate A and tau
        Area_weld = 160 * throat_area_weld;        % mm^2
        tau_weld = F_impact_y / Area_weld;        % MPa
        
        % Calculate stress resultant
        stress_resultant_weld = sqrt(sigma_weld^2 + tau_weld^2);
        
        % Calculate safety factor
        SF_throat_area = stress_sy_filler / stress_resultant_weld;
        
        % Check if safety factor is greater than or equal to 4
        if SF_throat_area >= 4
            break;
        end
        
        % Increment thickness
        throat_area_weld = throat_area_weld + t_increment_weld;
    end
    
    
        %Calculate weld leg size
        weld_leg = throat_area_weld/0.707;
    
    %fprintf('Throat area required to achieve a safety factor of 4: %.3f mm\n', throat_area_weld);
    weld_leg_log = sprintf('Weld leg of: %.3fmm (SF = %.2f)\n', weld_leg, SF_throat_area);
    end
    
    %----------------Wheel Insert Optimzation----------------%

    function [drivetrain_wheel_insert_thickness, wheel_insert_log] = optimize_wheel_insert(F_impact_x)
    %----------------Wheel Insert Thickness----------------%
    % Local Parameters
    drivetrain_insert_bolt_dia = 2.5; % mm 
    % Initial thickness
    t_wheel_insert = 1; % mm
    
    while true %generate argument that solves safety factor
        stress_applied = F_impact_x / (drivetrain_insert_bolt_dia * t_wheel_insert);
        SF_wheel_insert = stress_sy / stress_applied;
        
        if SF_wheel_insert >= 2
      wheel_insert_log = sprintf('Wheel insert thickness %.2fmm (SF = %.2f)\n', t_wheel_insert,SF_wheel_insert);
            drivetrain_wheel_insert_thickness = t_wheel_insert;
            break;
        end
        
        t_wheel_insert = t_wheel_insert + 0.1; % Increase thickness by 0.1 mm
    end
    end
    
    %Call Optimization Function
    [drivetrain_height, length_y, upright_height, geometric_log] = geometric_drivetrain_heights(wheel_diameter);
    [motor_housing_thickness, motor_housing_log] = optimize_motor_housing(F_impact_x);
    [drivetrain_wheel_bolt_dia, wheel_bolt_log] = optimize_wheel_bolt(F_impact_x);
    [drivetrain_wheelplate_thickness, wheel_plate_log] = optimize_wheel_plate(F_impact_x);
    [weld_leg, weld_leg_log] = optimize_weld_leg(F_impact_y);
    [drivetrain_wheel_insert_thickness, wheel_insert_log] = optimize_wheel_insert(F_impact_x);


    % Populate outputs (maps, etc.)
    drivetrain_map= containers.Map({'drivetrain_height','length_y', 'upright_height', 'drivetrain_wheel_insert_thickness','weld_leg','drivetrain_wheelplate_thickness','drivetrain_wheel_bolt_dia','motor_housing_thickness',},[drivetrain_height, length_y, upright_height, drivetrain_wheel_insert_thickness,weld_leg,drivetrain_wheelplate_thickness,drivetrain_wheel_bolt_dia,motor_housing_thickness]);
    
    % Construct log string
        log = "***Drivetrain System***" +newline + newline + "Structural:" +newline + motor_housing_log + wheel_bolt_log + wheel_plate_log + weld_leg_log + wheel_insert_log + newline + "Geometric:" + newline + geometric_log;

end