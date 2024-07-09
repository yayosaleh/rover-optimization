% Input is the reach of the robotic arm from GUI
% Outputs are the length of the second member and the material thickness
% for the arm members

function arm = optimize_robotic_arm(arm_reach)

    % The following set of values are constants needed to complete
    % calculations pertaining to the optimization of the robotic arm

    g = 9.81; % in m/s^2. Gravitational constant

    l_member1 = 500; % in mm. Length of first member

    m_claw = 0.18777; % in kg. Mass of claw and its hardware
    m_member2 = 0.23738; % in kg. Mass of member 2 and its hardware
    m_member1 = 0.26896; % in kg. Mass of member 1 and its hardware
    m_lin_act1 = 0.70295; % in kg. Mass of PA-11 from Progressive Automations and its hardware
    m_lin_act2 = 0.79365; % in kg. Mass of PA-14 from Progressive Automations and its hardware
    m_lin_act3 = 2 * m_lin_act2; % in kg. Mass of 2 PA-14s from Progressive Automations and its hardware
    m_payload = 5; % in kg. Mass of payload being carried by robotic arm

    r_lin_act3 = 120; % in mm. Perpendicular distance between COM of linear actuator 3 and arm pivot point
    rf_lin_act3 = 60; % in mm. Perpendicular distance between Force of linear actuator 3 and arm pivot point

    t_table = [0.455, 0.511, 0.574, 0.643, 0.724, 0.831, 0.912, 1.024, 1.151, 1.29, 1.45, 1.628, 1.829]; % in mm. Table of standard aluminum sheet metal sizes from 25 to 13 gauge material
    
    SF = 2; % Safety factor for bearing load and bending analysis

    b_outer = 26; % in mm. Outer base dimension of member 2 cross-section
    h_outer = 40; % in mm. Outer height dimension of member 2 cross-section

    Yield_stress = 110; % in MPa. Yield stress of aluminum 6061-T4

    Angle1 = asind((l_member1*sind(122)/arm_reach)); % in degrees. Angle between member2 and horizontal plane when payload is furthest from the pivot point in the horizontal direction
    Angle2 = 58 - Angle1; % in degrees. Angle between member1 and horizontal plane when payload is furthest from the pivot point in the horizontal direction

    l_member2 = (arm_reach*sind(Angle2))/sind(122); % in mm. Calculated length of member2 based on client input

    % Calculating distances of component's COMs from the main arm
    % pivot point
        
    r_lin_act2 = l_member2*cosd(Angle1) - 140; % in mm. Distance between COM of linear actuator 2 and arm pivot
    r_member2 = (l_member2/2)*cosd(Angle1); % in mm. Distance between COM of member 2 and arm pivot
    r_member1 = l_member2*cosd(Angle1) + l_member1*cosd(Angle2); % in mm. Distance between COM of member 1 and arm pivot
    r_claw = arm_reach; % in mm. Distance between claw and payload COM and arm pivot

    % Calculating weight forces of all components composing robotic arm
    % assembly

    W_claw = m_claw * g; % in N. Weight force caused by claw
    W_member2 = m_member2 * g; % in N. Weight force caused by member 2
    W_member1 = m_member1 * g; % in N. Weight force caused by member 1
    W_lin_act1 = m_lin_act1 * g; % in N. Weight force caused by PA-11 linear actuator
    W_lin_act2 = m_lin_act2 * g; % in N. Weight force caused by PA-14 linear actuator
    W_lin_act3 = m_lin_act3 * g; % in N. Weight force caused by PA-14 linear actuator
    W_payload = m_payload * g; % in N. Weight force caused by payload

    % Calculating moments caused by weight of components

    M_claw = (W_claw + W_payload)*r_claw; % in N*m. Moment caused by weight of claw and payload
    M_member1 = (W_lin_act1 + W_member1)*r_member1; % in N*m. Moment caused by weight of member 1
    M_lin_act2 = W_lin_act2*r_lin_act2; % in N*m. Moment caused by weight of linear actuator 2
    M_member2 = W_member2*r_member2; % in N*m. Moment caused by weight of member 2
    M_lin_act3 = W_lin_act3*r_lin_act3; % in N*m. Moment caused by weight of linear actuator 3

    % This function calculates the force required by linear actuator 3 to
    % lift the payload of 5kg and the weight of the robotic arm itself

    function [F_lin_act3, length_member2] = Force_of_Actuator3(l_member1, arm_reach, rf_lin_act3, M_claw, M_member1, M_lin_act2, M_member2, M_lin_act3)
     
        % Calculating Force requried by linear actuator 3 and its safety
        % factor based on the capacity of PA-14 linear actuator

        F_lin_act3 = (M_claw + M_member1 + M_lin_act2 + M_member2 + M_lin_act3)/rf_lin_act3; % in N. Force required by the linear actuator 3 to lift payload

        theta1 = asind((l_member1*sind(122)/arm_reach)); % in degrees. Angle between horizon and member 2
        theta2 = 58 - theta1; % in degrees. Angle between horizon and member 1

        length_member2 = (arm_reach*sind(theta2))/sind(122); % in mm. Length of member 2
        
    end
    
% This function finds the minimum material thickness for member 2 in order
% to not yield under bending

    function Thickness_member2 = Optimize_Member_Thickness(SF, t_table, b_outer, h_outer, M_claw, M_member1, M_lin_act2, M_member2, M_lin_act3, length_member2, Yield_stress, r_lin_act3, r_lin_act2, r_member2, W_lin_act2, W_member2, W_lin_act3, l_member1, arm_reach)

        Thickness_member2 = t_table(1); % in mm. Material thickness of member 2
        b_inner = b_outer - 2*Thickness_member2; % in mm. inner base dimension of member 2 cross-section
        h_inner = h_outer - 2*Thickness_member2; % in mm. inner height dimension of member 2 cross-section
        Cross_Sec_Area = (b_outer*h_outer)-(b_inner*h_inner); % in mm^2. Cross-sectional area of material of member 2
        theta1 = asind((l_member1*sind(122)/arm_reach));

        I = (b_outer*(h_outer^3)-b_inner*(h_inner^3))/12; % in mm^4. Moment of inertia of member 2

        M_equivalent = M_claw + M_member1; % in N*mm. Equivalent moment caused by the claw, payload, member 1 and linear actuator 1

        Bend_stress = ((M_equivalent + M_lin_act2 + M_member2 + M_lin_act3)*(h_outer/2))/I; % in MPa. Stress seen by member 2 due to bending

        Axial_force = ((M_equivalent/length_member2) + (M_lin_act2/r_lin_act2) + (M_member2/r_member2) + (M_lin_act3/r_lin_act3))*tand(theta1); % in N. Force acting on member 2 in the axial direction

        Shear_force = M_equivalent/length_member2 + W_lin_act2 + W_member2 + W_lin_act3; % in N. Force acting in Shear on member 2

        Axial_stress = Axial_force/Cross_Sec_Area; % in MPa. Stress seen by member 2 in the axial direction

        Shear_stress = Shear_force/Cross_Sec_Area; % in MPa. Shear stress acting on member 2

        Von_Mises = sqrt((Axial_stress + Bend_stress)^2 + 3*(Shear_stress^2)); % in MPa. Von MIses stress based on the axial stress, shear stress and bending stress

        SF_Von_Mises = Yield_stress/Von_Mises; % Safety factor based on Von Mises stress and yield stress 

        i = 0;

        while SF_Von_Mises < SF
    
            i = i + 1;

            Thickness_member2 = t_table(i);
            b_inner = b_outer - 2*Thickness_member2;
            h_inner = h_outer - 2*Thickness_member2;
            Cross_Sec_Area = (b_outer*h_outer)-(b_inner*h_inner);

            I = (b_outer*(h_outer^3)-b_inner*(h_inner^3))/12;

            M_equivalent = M_claw + M_member1;

            Bend_stress = ((M_equivalent + M_lin_act2 + M_member2 + M_lin_act3)*(h_outer/2))/I;

            Axial_force = ((M_equivalent/length_member2) + (M_lin_act2/r_lin_act2) + (M_member2/r_member2) + (M_lin_act3/r_lin_act3))*tand(theta1);
            Axial_stress = Axial_force/Cross_Sec_Area;

            Shear_force = M_equivalent/length_member2 + W_lin_act2 + W_member2 + W_lin_act3;
            Shear_stress = Shear_force/Cross_Sec_Area;

            Von_Mises = sqrt((Axial_stress + Bend_stress)^2 + 3*(Shear_stress^2));

            SF_Von_Mises = Yield_stress/Von_Mises;

        end
    end

% This function finds the minimum material thickness for member 2 in order
% to not yield when linear actuator pushes on actuator mount

    function [Thickness_Material, SF_Bearing] = Bearing_Load(t_table, Thickness_member2, F_lin_act3, Yield_stress, SF)

        Bolt_Diameter = 9.525; % in mm. bolt diameter connecting linear actuator to mount
        t_mount = t_table(1); % in mm. material thickness of member 2
        Bearing_Area = Bolt_Diameter*t_mount; % in mm^2. Stressed area loaded by bolt

        Bearing_Stress = F_lin_act3/Bearing_Area; % in MPa. Stress seen by member 2

        SF_Bearing = Yield_stress/Bearing_Stress; % Safety factor based on the bearing stress seen by member 2 and the yield stress 

        i = 1;

        while SF_Bearing < SF
    
            i = i + 1;

            t_mount = t_table(i);

            Bearing_Area = Bolt_Diameter*t_mount;

            Bearing_Stress = F_lin_act3/Bearing_Area;

            SF_Bearing = Yield_stress/Bearing_Stress;
    
        end

        if t_mount > Thickness_member2

            Thickness_Material = t_mount;

        else 

            Thickness_Material = Thickness_member2;

        end
    end

    % Calling functions to calculate factors important to the optimization
    % of the robotic arm assembly

    [F_lin_act3, length_member2] = Force_of_Actuator3(l_member1, arm_reach, rf_lin_act3, M_claw, M_member1, M_lin_act2, M_member2, M_lin_act3);
    Thickness_member2 = Optimize_Member_Thickness(SF, t_table, b_outer, h_outer, M_claw, M_member1, M_lin_act2, M_member2, M_lin_act3, length_member2, Yield_stress, r_lin_act3, r_lin_act2, r_member2, W_lin_act2, W_member2, W_lin_act3, l_member1, arm_reach);
    [Thickness_Material, SF_Bearing] = Bearing_Load(t_table, Thickness_member2, F_lin_act3, Yield_stress, SF);

    log = sprintf("Structural:\nThickness of material: %.3gmm\nSafety Factor: %.3g\n\nGeometric:\nLength of member 2: %.3gmm\n", Thickness_Material, SF_Bearing, length_member2);
    
    % Populate outputs (maps, etc.)
    
    arm.map = containers.Map({'length_member2', 'Thickness_Material'},[length_member2, Thickness_Material]);
    arm.important_value = containers.Map({'Arm Reach'},arm_reach);
    
    % Construct log string

    arm.log = "***Robotic Arm System***" + newline + newline + log + newline + newline;

end