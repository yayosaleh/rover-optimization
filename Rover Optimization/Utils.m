classdef Utils

    methods(Static)

        % Returns target file path
        function path = get_path(filename)
            if contains(lower(filename), 'log')
                path = fullfile('..', 'Log', filename);
                return
            end
            path = fullfile('..', 'Solidworks','Equations', filename);
        end
        
        % Converts map to a string
        function str = map_to_string(map)
            
            str = '';
            keySet = keys(map);
            numKeys = length(keySet);
            
            for i = 1 : numKeys
                
                key = keySet{i};
                value = map(key);
                unit = '';
                
                if ~contains(key, 'angle') && ~contains(key, 'num')
                    unit = "mm";
                end
                
                str = str + """" + key + """" + "=" + num2str(value, '%.17g') + unit;
                
                if i ~= numKeys
                    str = str + newline;
                end

            end

        end
    
        % Merges unique entries of supplementary map into base map
        % Returns result
        function map = merge_maps(base, supp)
            
            % Copy base
            map = containers.Map(keys(base), values(base));
            
            % Iterate through supp
            % If supp key is unique, add entry to (base) map
            suppKeySet = keys(supp);
            for i = 1 : length(supp)
                suppKey = suppKeySet{i};
                if ~isKey(map, suppKey)
                    map(suppKey) = supp(suppKey);
                end
            end

        end
        
        % Writes string to text file
        function string_to_file(str, filename)
            filename = Utils.get_path(filename);
            fileId = fopen(filename, 'w');
            if fileId == -1
                error('Error opening file for writing.');
            end
            fprintf(fileId, '%s', str);
            fclose(fileId);
        end
        
        % Reads text file and converts it to a map
        function map = file_to_map(filename)
            filename = Utils.get_path(filename);
            % Check if the file exists
            if ~exist(filename, 'file')
                disp(['File ' filename ' does not exist.']);
                map = containers.Map('KeyType', 'char', 'ValueType', 'double');
                return;
            end
            
            % Initialize an empty map
            map = containers.Map('KeyType', 'char', 'ValueType', 'double');
            
            % Open the file for reading
            fileId = fopen(filename, 'r');
            
            % Read and process each line of the file
            while ~feof(fileId)
                line = fgetl(fileId);
                if ~isempty(line) && ~isspace(line(1))  % Check for non-empty lines
                    % Extract key and value from the line
                    tokens = regexp(line, '"([^"]+)"\s*=\s*([\d.]+)', 'tokens');
                    if ~isempty(tokens)
                        key = tokens{1}{1};
                        value = str2double(tokens{1}{2});
                        map(key) = value;
                    end
                end
            end
            
            % Close the file
            fclose(fileId);
        end
        
        % Write map to text file
        function map_to_file(map, filename)
            new = map;
            old = Utils.file_to_map(filename);
            merged = Utils.merge_maps(new, old);
            Utils.string_to_file(Utils.map_to_string(merged), filename);
        end

    end

end