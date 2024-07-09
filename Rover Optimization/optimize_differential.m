function [diff_map, diff_log] = optimize_differential(vertical_moment)
    
    % Optimization functions

    function [bearing_inner_diameter, bearing_outer_diameter, bearing_thickness, hex_thickness, hex_width, linkage_stud_diameter, linkage_outer_diameter, linkage_ball_width, linkage_pitch, linkage_stud_length, linkage_shank_length, linkage_hole_depth, ring_diff_thickness, ring_diff_outer_diameter, ring_diff_inner_diameter, ring_diff_groove_diameter, ring_diff_groove_thickness, ring_frame_thickness, ring_frame_outer_diameter, ring_frame_inner_diameter, ring_frame_groove_diameter, ring_frame_groove_thickness, rod_length, differential_rod_width, plate_height, log_contribution, log_structural] = optimize_diff(T)
        % Initial constants & impact force calculation
        SF = 2;
        plate_height = 99; % Assumed clevis plate height at worst case scenerio (mm)
        F = T/(plate_height/1000); %Impact force (N)

        % ------------------ Ball joint linkage ------------------ %
        % Optimization loop to determine ball joint linkage size and to verify the
        % recalculated impact force with the new associated plate height dimension
        n = false; %Index to check if recalculated force fits in matched range
        ball_joint_linkage_SF = 2;
        while ~n
            Load12 = F*SF/9.81*2.2; % Force acting on ball joint linkage
            % Compare to McMasterCARR Catalogue
            if (Load12 < 260)
                plate_height = 109.75;
                n = true;
                
                %Dimensions for CAD (Ball Joint Linkage Size: M5 x 0.8 (6275K52))
                linkageID = '6275K52';
                linkage_stud_diameter = 5;
                linkage_outer_diameter = 17;
                linkage_ball_width = 32.5;
                linkage_pitch = 0.8;
                linkage_stud_length = 10;
                linkage_shank_length = 30;
                linkage_hole_depth = 16;
            
                rod_length = 132;
                differential_rod_width = 170.25;
        
                hex_thickness = 4;
                hex_width = 8;
        
            elseif (Load12 >= 260) && (Load12 < 370)
                plate_height = 109.75;
        
                % Verify if recalculated force with correct height works in range
                if (Load12 >= 260) && (Load12 < 370)
                    n = true;
                    F = T/(plate_height/1000);
                else
                    n = false;
                    F = T/(plate_height/1000);
                end
                
                %Dimensions for CAD (Ball Joint Linkage Size: M6 x 1.0 (6275K53))
                linkageID = '6275K53';
                linkage_stud_diameter = 6;
                linkage_outer_diameter = 19;
                linkage_ball_width = 32.5;
                linkage_pitch = 1;
                linkage_stud_length = 10;
                linkage_shank_length = 30;
                linkage_hole_depth = 16;
            
                rod_length = 132;
                differential_rod_width = 170.25;
        
                hex_thickness = 5;
                hex_width = 10;
        
            elseif (Load12 >= 370) && (Load12 < 980)
                plate_height = 106.5;
        
                % Verify if recalculated force with correct height works in range
                if (Load12 >= 370) && (Load12 < 980)
                    n = true;
                    F = T/(plate_height/1000);
                else
                    n = false;
                    F = T/(plate_height/1000);
                end
        
                % Dimensions for CAD (Ball Joint Linkage Size: M8 x 1.25 (6275K54))
                linkageID = '6275K54';
                linkage_stud_diameter = 8;
                linkage_outer_diameter = 24;
                linkage_ball_width = 41.5;
                linkage_pitch = 1.25;
                linkage_stud_length = 12.5;
                linkage_shank_length = 36;
                linkage_hole_depth = 19;
            
                rod_length = 126;
                differential_rod_width = 173.5;
        
                hex_thickness = 6.5;
                hex_width = 13;
        
            elseif (Load12 >= 980) && (Load12 < 1650)
                plate_height = 111;
        
                % Verify if recalculated force with correct height works in range
                if (Load12 >= 980) && (Load12 < 1650)
                    n = true;
                    F = T/(plate_height/1000);
                else
                    n = false;
                    F = T/(plate_height/1000);
                end
        
                % Dimensions for CAD (Ball Joint Linkage Size: M10 x 1.50 (6275K57))
                linkageID = '6275K57';
                linkage_stud_diameter = 10;
                linkage_outer_diameter = 28;
                linkage_ball_width = 49;
                linkage_pitch = 1.5;
                linkage_stud_length = 17;
                linkage_shank_length = 43;
                linkage_hole_depth = 23;
            
                rod_length = 120;
                differential_rod_width = 175;
        
                hex_thickness = 8;
                hex_width = 17;
        
            elseif (Load12 >= 1650) && (Load12 < 2200)
                plate_height = 99;
        
                % Verify if recalculated force with correct height works in range
                if (Load12 >= 1650) && (Load12 < 2200)
                    n = true;
                    F = T/(plate_height/1000);
                else
                    n = false;
                    F = T/(plate_height/1000);
                end
        
                % Dimensions for CAD (Ball Joint Linkage Size: M12 x 1.75 (6275K58))
                linkageID = '6275K58';
                linkage_stud_diameter = 12;
                linkage_outer_diameter = 34;
                linkage_ball_width = 64;
                linkage_pitch = 1.75;
                linkage_stud_length = 20;
                linkage_shank_length = 50;
                linkage_hole_depth = 27;
            
                rod_length = 114;
                differential_rod_width = 181;
        
                hex_thickness = 10;
                hex_width = 19;

            elseif (Load12 >= 2200)
                plate_height = 99;
                n = true;

                % Calculate new safety factor
                ball_joint_linkage_SF = 2200*9.81/2.2/Load12;

                % Dimensions for CAD (Ball Joint Linkage Size: M12 x 1.75 (6275K58))
                linkageID = '6275K58';
                linkage_stud_diameter = 12;
                linkage_outer_diameter = 34;
                linkage_ball_width = 64;
                linkage_pitch = 1.75;
                linkage_stud_length = 20;
                linkage_shank_length = 50;
                linkage_hole_depth = 27;
            
                rod_length = 114;
                differential_rod_width = 181;
        
                hex_thickness = 10;
                hex_width = 19;

            else
                fprintf('Ball Joint Linkage: Error\n')
            end
        end

        % ------------------ Ball Bearing ------------------ %
        LoadBy = F*2*SF/9.81*2.2; % Radial force acting on ball bearing
        ball_bearing_SF = 2;
        % Compare to McMasterCARR Catalogue
        if (LoadBy < 1750)
            % Dimensions for CAD (Ball Bearing Size: 20mm ID (5972K351))
            bearingID = '5972K351';
            bearing_inner_diameter = 20;
            bearing_outer_diameter = 52;
            bearing_thickness = 15;
        elseif (LoadBy >= 1750) && (LoadBy < 2600)
            % Dimensions for CAD (Ball Bearing Size: 25mm ID (5972K352))
            bearingID = '5972K352';
            bearing_inner_diameter = 25;
            bearing_outer_diameter = 62;
            bearing_thickness = 17;
        elseif (LoadBy >= 2600) && (LoadBy < 3600)
            % Dimensions for CAD (Ball Bearing Size: 30mm ID (5972K353))
            bearingID = '5972K353';
            bearing_inner_diameter = 30;
            bearing_outer_diameter = 72;
            bearing_thickness = 19;
        elseif (LoadBy >= 3600)
            % Calculate new safety factor
            ball_bearing_SF = 3600*9.81/2.2/Load12/2;
            
            % Dimensions for CAD (Ball Bearing Size: 30mm ID (5972K353))
            bearingID = '5972K353';
            bearing_inner_diameter = 30;
            bearing_outer_diameter = 72;
            bearing_thickness = 19;
        else
            fprintf('Ball Bearing: Error\n')
        end
        
        % ------------------ External retaining rings ------------------ %
        % Compare to McMasterCARR Catalogue based off chosen ball bearing
        if (bearing_inner_diameter == 20)
            % Dimensions for CAD
            %fprintf('Retaining Ring (Diff): 20mm OD (98541A123)\n')
            ring_diffID = '98541A123';
            ring_diff_thickness = 1.2;
            ring_diff_outer_diameter = 20;
            ring_diff_inner_diameter = 18.5;
            ring_diff_groove_diameter = 19;
            ring_diff_groove_thickness = 1.3;
        
            %fprintf('Retaining Ring (Frame): 25mm OD (98541A440)\n')
            ring_frameID = '98541A440';
            ring_frame_thickness = 1.2;
            ring_frame_outer_diameter = 25;
            ring_frame_inner_diameter = 23.2;
            ring_frame_groove_diameter = 23.9;
            ring_frame_groove_thickness = 1.3;
        elseif (bearing_inner_diameter == 25)
            % Dimensions for CAD
            %fprintf('Retaining Ring (Diff): 25mm OD (98541A440)\n')
            ring_diffID = '98541A440';
            ring_diff_thickness = 1.2;
            ring_diff_outer_diameter = 25;
            ring_diff_inner_diameter = 23.2;
            ring_diff_groove_diameter = 23.9;
            ring_diff_groove_thickness = 1.3;
        
            %fprintf('Retaining Ring (Frame): 30mm OD (98541A134)\n')
            ring_frameID = '98541A134';
            ring_frame_thickness = 1.5;
            ring_frame_outer_diameter = 30;
            ring_frame_inner_diameter = 27.9;
            ring_frame_groove_diameter = 28.6;
            ring_frame_groove_thickness = 1.6;
        elseif (bearing_inner_diameter == 30)
            % Dimensions for CAD
            %fprintf('Retaining Ring (Diff): 30mm OD (98541A134)\n')
            ring_diffID = '98541A134';
            ring_diff_thickness = 1.5;
            ring_diff_outer_diameter = 30;
            ring_diff_inner_diameter = 27.9;
            ring_diff_groove_diameter = 28.6;
            ring_diff_groove_thickness = 1.6;
           
            %fprintf('Retaining Ring (Frame): 35mm OD (98541A146)\n')
            ring_frameID = '98541A146';
            ring_frame_thickness = 1.5;
            ring_frame_outer_diameter = 35;
            ring_frame_inner_diameter = 32.2;
            ring_frame_groove_diameter = 33;
            ring_frame_groove_thickness = 1.6;
        else
            fprintf('Retaining Ring: Error\n')
        end
        
        % ------------------ Differential Rod Pin ------------------ %
        % Verify if the pin will fail based off the ball bearing chosen
        % Requires SYMBOLIC MATH TOOLBOX
        dP = sym ('dP');
        S = 276 * 10^6; %Tensile yield strength for Aluminum 6061 T6
        FBy = F*SF; % Radial force acting on pin
        eqn = ((S^2)*((3.14)^2)*(dP^6))+(-12*(FBy^2)*(SF^2)*(dP^2))+(-16*(FBy^2)*(((bearing_thickness*10^-3)+(ring_diff_thickness*10^-3)+(((ring_frame_thickness)+1.5)*10^-3))^2)*(SF^2));
        solution = solve(eqn,dP);
        min_diameter = vpa(solution*10^3);
        min_diameter=abs(min_diameter(3:3,1:1));
        % Check that the ball bearing chosen is greater than the minimum size
        if (bearing_inner_diameter >= min_diameter)
            pin_material = 'Aluminum 6061 T6';
        else
            fprintf('Differential Rod Pin: Error\n')
        end
        
        % ------------------ Connecting Rod ------------------ %
        E = 70*10^9; % Elastic modulus for Aluminum 6061 T6
        Ry12 = F*SF; % Axial force acting on rod to fail by buckling
        Pcr = (((3.14)^3)*E*((linkage_stud_diameter*10^-3)^4))/(64*(rod_length*10^-3)^2);
        % Check that the connecting rod will not buckle
        if (Pcr >= Ry12)
            connecting_rod_material = 'Aluminum 6061 T6';
        else
            % Switch material to stainless steel
            fprintf('Connecting Rod: Use stainless steel\n')
            E = 193*10^9; % Elastic modulus for Stainless steel
            Pcr = (((3.14)^3)*E*((linkage_stud_diameter*10^-3)^4))/(64*(rod_length*10^-3)^2);
            % Check that the connecting rod will not buckle
            if (Pcr >= Ry12)
                connecting_rod_material = 'Stainless steel';
            else
                fprintf('Connecting Rod: Error\n')
            end
        end

        % ------------------ Log ------------------ %
        % Displays different log message depending if SF for ball bearing or ball joint linkage is compromised
        if (ball_bearing_SF < SF) && (ball_joint_linkage_SF < SF)
            log_structural = sprintf(" Ball bearing safety factor: %.3g\n Ball joint linkage safety factor: %.3g\n Ball bearing McMASTERCARR ID: %s(ID: %.3gmm)\n Ball joint linkage McMASTERCARR ID: %s(Size: M%.3g x %.3g mm)", ball_bearing_SF, ball_joint_linkage_SF, bearingID ,bearing_inner_diameter, linkageID ,linkage_stud_diameter, linkage_pitch);
            log_contribution = sprintf(" External retaining ring (diff) McMASTERCARR ID: %s\n External retaining ring (frame) McMASTERCARR ID: %s\n Differential rod pin will not fail based on chosen ball bearing size (Design diameter: %.3gmm > Minimum diameter: %.3gmm)\n Connecting rod will not fail based on chosen ball joint linkage size (Material: %s)", ring_diffID, ring_frameID, bearing_inner_diameter, min_diameter, connecting_rod_material);
        elseif (ball_joint_linkage_SF < SF)
            log_structural = sprintf(" Ball joint linkage safety factor: %.3g\n Ball bearing McMASTERCARR ID: %s(ID: %.3gmm)\n Ball joint linkage McMASTERCARR ID: %s(Size: M%.3g x %.3g mm)", ball_joint_linkage_SF, bearingID ,bearing_inner_diameter, linkageID ,linkage_stud_diameter, linkage_pitch);
            log_contribution = sprintf(" External retaining ring (diff) McMASTERCARR ID: %s\n External retaining ring (frame) McMASTERCARR ID: %s\n Differential rod pin will not fail based on chosen ball bearing size (Design diameter: %.3gmm > Minimum diameter: %.3gmm)\n Connecting rod will not fail based on chosen ball joint linkage size (Material: %s)", ring_diffID, ring_frameID, bearing_inner_diameter, min_diameter, connecting_rod_material);
        elseif (ball_bearing_SF < SF)
            log_structural = sprintf(" Ball bearing safety factor: %.3g\n Ball bearing McMASTERCARR ID: %s(ID: %.3gmm)\n Ball joint linkage McMASTERCARR ID: %s(Size: M%.3g x %.3g mm)", ball_bearing_SF, bearingID ,bearing_inner_diameter, linkageID ,linkage_stud_diameter, linkage_pitch);
            log_contribution = sprintf(" External retaining ring (diff) McMASTERCARR ID: %s\n External retaining ring (frame) McMASTERCARR ID: %s\n Differential rod pin will not fail based on chosen ball bearing size (Design diameter: %.3gmm > Minimum diameter: %.3gmm)\n Connecting rod will not fail based on chosen ball joint linkage size (Material: %s)", ring_diffID, ring_frameID, bearing_inner_diameter, min_diameter, connecting_rod_material);
        else
            log_structural = sprintf(" Ball bearing McMASTERCARR ID: %s(ID: %.3gmm)\n Ball joint linkage McMASTERCARR ID: %s(Size: M%.3g x %.3g mm)", bearingID ,bearing_inner_diameter, linkageID ,linkage_stud_diameter, linkage_pitch);
            log_contribution = sprintf(" External retaining ring (diff) McMASTERCARR ID: %s\n External retaining ring (frame) McMASTERCARR ID: %s\n Differential rod pin will not fail based on chosen ball bearing size (Design diameter: %.3gmm > Minimum diameter: %.3gmm)\n Connecting rod will not fail based on chosen ball joint linkage size (Material: %s)", ring_diffID, ring_frameID, bearing_inner_diameter, min_diameter, connecting_rod_material);
        end

    end

    % Process inputs (if needed)
    
    T = vertical_moment;

    % Call optimizers

    [bearing_inner_diameter, bearing_outer_diameter, bearing_thickness, hex_thickness, hex_width, linkage_stud_diameter, linkage_outer_diameter, linkage_ball_width, linkage_pitch, linkage_stud_length, linkage_shank_length, linkage_hole_depth, ring_diff_thickness, ring_diff_outer_diameter, ring_diff_inner_diameter, ring_diff_groove_diameter, ring_diff_groove_thickness, ring_frame_thickness, ring_frame_outer_diameter, ring_frame_inner_diameter, ring_frame_groove_diameter, ring_frame_groove_thickness, rod_length, differential_rod_width, plate_height, log_contribution, log_structural] = optimize_diff(T);
    
    % Populate outputs (maps, etc.)
    
    diff_map = containers.Map({'bearing_inner_diameter', 'bearing_outer_diameter', 'bearing_thickness', 'hex_thickness', 'hex_width', 'linkage_stud_diameter', 'linkage_outer_diameter', 'linkage_ball_width', 'linkage_pitch', 'linkage_stud_length', 'linkage_shank_length', 'linkage_hole_depth', 'ring_diff_thickness', 'ring_diff_outer_diameter', 'ring_diff_inner_diameter', 'ring_diff_groove_diameter', 'ring_diff_groove_thickness', 'ring_frame_thickness', 'ring_frame_outer_diameter', 'ring_frame_inner_diameter', 'ring_frame_groove_diameter', 'ring_frame_groove_thickness', 'rod_length', 'differential_rod_width', 'plate_height'},[bearing_inner_diameter, bearing_outer_diameter, bearing_thickness, hex_thickness, hex_width, linkage_stud_diameter, linkage_outer_diameter, linkage_ball_width, linkage_pitch, linkage_stud_length, linkage_shank_length, linkage_hole_depth, ring_diff_thickness, ring_diff_outer_diameter, ring_diff_inner_diameter, ring_diff_groove_diameter, ring_diff_groove_thickness, ring_frame_thickness, ring_frame_outer_diameter, ring_frame_inner_diameter, ring_frame_groove_diameter, ring_frame_groove_thickness, rod_length, differential_rod_width, plate_height]);
    
    % Construct log string

    diff_log = "***Differential System***" + newline + newline + "Structural:" + newline + log_structural + newline + newline + "Geometric:" + newline + log_contribution + newline;

end