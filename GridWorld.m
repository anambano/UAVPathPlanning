classdef GridWorld < matlab.System
% GridWorld represents a 12x12 deterministic grid world with 3 robots.

% Copyright 2020-2022 The MathWorks, Inc.
    
    % Public, tunable properties
    properties
        % Initial position of robots
        InitialStates (3,2) double = [2 2; 11 4; 3 12]
    end

    % Public, non-tunable properties
    properties(Nontunable)
        % Obstacle matrix
        Obstacles double = -1
        % Max step count
        MaxStepCount (1,1) double = 500
    end

    properties(DiscreteState)
        % Discretized XY space with cells containing 0.5 or 1.0
        % 0:   unexplored
        % 0.25: explored by robot A
        % 0.50: explored by robot B
        % 0.75: explored by robot B
        % 1.0: obstacle
        Grid
        Visits
        % States of robot A,B: [rowA colA; rowB colB]
        States
        % Step count
        StepCount
        % Individual cell exploration count
        NumExploredCells
    end

    % Pre-computed constants
    properties(Access = private)
        % Handle to figure
        Figure
        % Grid size [numrows numcols]
        Size (1,2) double = [12 12]
        Destination (1, 2) double = [12 12]
        FarUnits = 0
        PrevFarUnits = 0
        CurrentReward = 0
    end

    methods
        % Constructor
        function this = GridWorld(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(this,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        %% Common functions
        function setupImpl(obj) %#ok<MANU>
            % Perform one-time calculations, such as computing constants
        end

        function [observations,rewards,isdone] = stepImpl(obj,actions)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            numRobots = 3;
            
            % Rewards are:
            % Agent moves to unexplored cell: +20
            % Agent moves to explored cell: 0
            % Agent tries to move out of grid: -10
            % Agent collides with another agent: -10
            % Agent collides with obstacle: -10
            % Movement penalty: -1
            % Lazy penalty: -2
            % On full coverage: +4000 * coverage contribution
            
            % move robots to their next state
            %next_states = zeros(numRobots,2);
            rewards = zeros(numRobots,1);
            isdone = 0;
            next_states = obj.States;
            for idx = 1:numRobots
                state = obj.States(idx,:);
                action = actions(idx);
                switch(action)
                    case 0
                        % Wait
                        next_states(idx,:) = state;
                        rewards(idx) = rewards(idx) - 10;  % lazy penalty
                    case 1
                        % Move up
                        if state(1) < obj.Size(1) && ~checkCollision(obj,state+[1 0],next_states(idx~=1:numRobots,:))
                            proposedstate = state + [1 0];
                            [ps, reward] = proximityReward(obj, idx, proposedstate);
                            if ps(1, 1) ~= -1
                                next_states(idx,:) = ps;
                            end
                            rewards(idx) = rewards(idx) + reward;
                        else
                            % add a negative reward for trying to go up
                            next_states(idx,:) = state;
                            rewards(idx) = rewards(idx) - 20;
                        end
                    case 2
                        % Move down
                        if state(1) > 1 && ~checkCollision(obj,state+[-1 0],next_states(idx~=1:numRobots,:))
                            proposedstate = state + [-1 0];
                            [ps, reward] = proximityReward(obj, idx, proposedstate);
                            if ps(1, 1) ~= -1
                                next_states(idx,:) = ps;
                            end
                            rewards(idx) = rewards(idx) + reward;
                        else
                            % add a negative reward for trying to go down
                            next_states(idx,:) = state;
                            rewards(idx) = rewards(idx) - 20;
                        end
                    case 3
                        % Move left
                        if state(2) > 1 && ~checkCollision(obj,state+[0 -1],next_states(idx~=1:numRobots,:))
                            proposedstate = state + [0 -1];
                            [ps, reward] = proximityReward(obj, idx, proposedstate);
                            if ps(1, 1) ~= -1
                                next_states(idx,:) = ps;
                            end
                            rewards(idx) = rewards(idx) + reward;
                        else
                            % add a negative reward for trying to go left
                            next_states(idx,:) = state;
                            rewards(idx) = rewards(idx) - 20;
                        end
                    case 4
                        % Move right
                        if state(2) < obj.Size(2) && ~checkCollision(obj,state+[0 1],next_states(idx~=1:numRobots,:))
                            proposedstate = state + [0 1];
                            [ps, reward] = proximityReward(obj, idx, proposedstate);
                            if ps(1, 1) ~= -1
                                next_states(idx,:) = ps;
                            end
                            rewards(idx) = rewards(idx) + reward;
                        else
                            % add a negative reward for trying to go right
                            next_states(idx,:) = state;
                            rewards(idx) = rewards(idx) - 20;
                        end
                end
                if idx == 1
                    obj.PrevFarUnits = obj.FarUnits;
                end
            end
            
            % update grid and reward agents for new exploration
            for idx = 1:numRobots
                r = next_states(idx,1);
                c = next_states(idx,2);
                if obj.Grid(r,c) == 0.0
                    % robot explores an unexplored cell
                    rewards(idx) = rewards(idx) + 10;
                    % update grid values
                    switch(idx)
                        case 1
                            obj.Grid(r,c) = 0.25;  % explored by A
                            obj.Visits(r, c) = 0;
                            obj.NumExploredCells = obj.NumExploredCells + [1 0 0];
                        case 2
                            obj.Grid(r,c) = 0.5;   % explored by B
                            obj.NumExploredCells = obj.NumExploredCells + [0 1 0];
                        case 3
                            obj.Grid(r,c) = 0.75;   % explored by C
                            obj.NumExploredCells = obj.NumExploredCells + [0 0 1];
                    end
                else
                    obj.Visits(r, c) = obj.Visits(r, c) + 1;
                    rewards(idx) = rewards(idx) - (obj.Visits(r, c)/2);
                end
            end
            
            % coverage metrics
            %totalExploredCells = sum(obj.NumExploredCells);
            %totalCells = obj.Size(1) * obj.Size(2) - sum(obj.Grid==1,'all');
            obj.CurrentReward = rewards(1);
            if obj.States(1,:) == obj.Destination
                %rewards = rewards + 2000 - obj.StepCount;
                rewards(1) = rewards(1) + 2000 - obj.StepCount;
                obj.CurrentReward = rewards(1);
                isdone = 1;
            end
            
            % Observation for each agent is a 12x12 4-channel image. The
            % channels are:
            % 1. Obstacle channel - cells with obstacles are 1, rest 0
            % 2. Self channel - cell with the agent's state is 1, rest 0
            % 3. Friend channel - cell with other agent's state is 1, rest 0
            % 4. Coverage channel - cells that are unexplored are 1, rest 0
            observations = zeros(obj.Size(1),obj.Size(2),4,numRobots);
            for idx = 1:numRobots
                obstacleChannel = 1.0 * (obj.Grid == 1.0);
                
                selfIdx = idx;
                selfRow = next_states(selfIdx,1);
                selfCol = next_states(selfIdx,2);
                selfChannel = zeros(obj.Size);
                selfChannel(selfRow,selfCol) = 1.0;
                
                friendIdxs = idx ~= (1:numRobots);
                friendRows = next_states(friendIdxs,1);
                friendCols = next_states(friendIdxs,2);
                friendChannel = zeros(obj.Size);
                friendChannel(friendRows(1),friendCols(1)) = 1.0;
                friendChannel(friendRows(2),friendCols(2)) = 1.0;
                
                coverageChannel = 1.0;
                
                observations(:,:,1,idx) = obstacleChannel;
                observations(:,:,2,idx) = selfChannel;
                observations(:,:,3,idx) = friendChannel;
                observations(:,:,4,idx) = coverageChannel;
            end
            
            % Scale down rewards
            rewards = rewards./20;
            
            % DEBUG
            if all(next_states(1,:)==next_states(2,:)) || ...
                    all(next_states(2,:)==next_states(3,:)) || ...
                    all(next_states(1,:)==next_states(3,:))
                fprintf('Assertion: Invalid state.\n');
            end
            
            % Update states
            obj.States = next_states;
            obj.StepCount = obj.StepCount + 1;
            
            % plot the environment
            if obj.StepCount <= obj.MaxStepCount
                plot(obj);
            end
            
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            
            % set unexplored cells
            obj.Grid = zeros(obj.Size);
            obj.Visits = zeros(obj.Size);
            % set obstacle cells
            if ~isequal(obj.Obstacles,-1)
                for idx = 1:size(obj.Obstacles,1)
                    r = obj.Obstacles(idx,1);
                    c = obj.Obstacles(idx,2);
                    obj.Grid(r,c) = 1.0;
                end
            end
            
            % set step count
            obj.StepCount = 0;
            
            % explored cells
            obj.NumExploredCells = ones(1,3);
            
            % set robot cells
            sA = obj.InitialStates(1,:);
            sB = obj.InitialStates(2,:);
            sC = obj.InitialStates(3,:);
            obj.Grid(sA(1),sA(2)) = 0.25;
            obj.Grid(sB(1),sB(2)) = 0.50;
            obj.Grid(sC(1),sC(2)) = 0.75;
            obj.States = obj.InitialStates;
        end

        %% Backup/restore functions
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            %s.myproperty = obj.myproperty;
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            % obj.myproperty = s.myproperty; 

            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        %% Simulink functions
        function ds = getDiscreteStateImpl(obj)
            % Return structure of properties with DiscreteState attribute
            ds.Grid = obj.Grid;
            ds.States = obj.States;
            ds.StepCount = obj.StepCount;
            ds.NumExploredCells = obj.NumExploredCells;
        end

        function flag = isInputSizeMutableImpl(obj,index) %#ok<INUSD>
            % Return false if input size cannot change
            % between calls to the System object
            flag = false;
        end

        function [ps, r] = proximityReward(obj, idx, proposedstate)
            Gap = obj.InitialStates(idx, :) - proposedstate;
            GapD = obj.Destination - proposedstate;
            dist = abs(Gap(1,1)) + abs(Gap(1,2));
            distD = abs(GapD(1,1)) + abs(GapD(1,2));
            destMaxVal = obj.Destination(1, 1) + obj.Destination(1, 2);
            obj.FarUnits = destMaxVal - distD;
            if idx == 1
                ps = proposedstate;
                cr = coverageReward(obj, proposedstate);
                if obj.FarUnits > obj.PrevFarUnits
                    r = obj.FarUnits + cr;
                else
                    r = (-1 * obj.FarUnits) + cr;
                end
            else
                if dist <= 2
                    ps = proposedstate;
                    r = -1;
                else
                    ps = [-1 -1];
                    r = 0;
                end
            end
        end

        function r = coverageReward(obj, proposedstate)
            GapGreen = obj.States(2, :) - proposedstate;
            GapBlue = obj.States(3, :) - proposedstate;
            distGreen = abs(GapGreen(1,1)) + abs(GapGreen(1,2));
            distBlue = abs(GapBlue(1,1)) + abs(GapBlue(1,2));
            if distGreen > 5
                distGreen = 12;
            end
            if distBlue > 5
                distBlue = 12;
            end
            dist = distGreen + distBlue;
            destMaxVal = obj.Destination(1, 1) + obj.Destination(1, 2);
            destVal = destMaxVal - dist;
            r = destVal/10;
        end

        function [out1, out2, out3] = getOutputSizeImpl(obj) %#ok<MANU>
            % Return size for each output port
            out1 = [12 12 4 3];  % observation
            out2 = [3 1];   % reward
            out3 = [1 1];   % isdone
        end

        function [out1,out2,out3] = getOutputDataTypeImpl(obj) %#ok<MANU>
            % Return data type for each output port
            out1 = "double";
            out2 = "double";
            out3 = "double";
        end

        function [out1,out2,out3] = isOutputComplexImpl(obj) %#ok<MANU>
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
            out3 = false;
        end

        function [out1,out2,out3] = isOutputFixedSizeImpl(obj) %#ok<MANU>
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;
            out3 = true;
        end

        function [sz,dt,cp] = getDiscreteStateSpecificationImpl(obj,name)
            % Return size, data type, and complexity of discrete-state
            % specified in name
            if strcmpi(name,'Grid')
                sz = obj.Size;
                dt = "double";
                cp = false;
            elseif strcmpi(name,'States')
                sz = [3 2];
                dt = "double";
                cp = false;
            elseif strcmpi(name,'StepCount')
                sz = [1 1];
                dt = "double";
                cp = false;
            elseif strcmpi(name,'NumExploredCells')
                sz = [1 3];
                dt = "double";
                cp = false;
            else
                error(['Error: Incorrect State Name: ', name.']);
            end
        end

        function icon = getIconImpl(obj) %#ok<MANU>
            % Define icon for System block
            icon = mfilename("class"); % Use class name
        end
    end

    methods(Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename("class"));
        end

        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            group = matlab.system.display.Section(mfilename("class"));
        end

    end
    
    methods(Access=private)
        function collision = checkCollision(obj,new_state,other_states)
            % Create array of occupied states
            if isequal(obj.Obstacles,-1)
                occupiedStates = other_states;
            else
                occupiedStates = [other_states; obj.Obstacles];
            end
            % check collision
            collision = any(all(new_state==occupiedStates,2));
        end

        function toofar = checkToofar(dist, idx)
            % check collision
            toofar = (dist > 2) && (idx > 1);
        end
        
        function plot(obj)
            persistent ax cells robots

            if isempty(ax) || ~isvalid(ax)
                % build figure
                f = figure;
                f.Position = [195 120 400 300];
                %f.Visible = 'on';  % force external figure
                ax = gca(f);

                hold(ax,'on');
                if ~isequal(obj.Obstacles,-1)
                    cmap = [255 255 255; ...    % white  (unexplored)
                            255 140 105; ...    % light red (explored by A)
                            152 251 152; ...    % light green (explored by B)
                            176 226 255; ...    % light blue (explored by C)
                            0 0 0]./255;        % black (obstacles)
                else
                    cmap = [255 255 255;...     % white  (unexplored)
                            255 140 105; ...    % light red (explored by A)
                            152 251 152; ...    % light green (explored by B)
                            176 226 255; ...    % light blue (explored by C)
                            ]./255;        % black (obstacles)
                end
                colormap(ax,cmap);
                
                % plot cells
                cdata = obj.Grid;
                cells = imagesc(ax,[0.5 obj.Size(1)-0.5],[0.5 obj.Size(2)-0.5],cdata);
                
                % plot grid lines
                x = 0:1:obj.Size(2);
                y = 0:1:obj.Size(1);
                [X,Y] = meshgrid(x,y);
                plot(ax,X,Y,'Color',[0.94 0.94 0.94]);
                plot(ax,Y,X,'Color',[0.94 0.94 0.94]);

                % plot robots
                sA = obj.InitialStates(1,:);
                sB = obj.InitialStates(2,:);
                sC = obj.InitialStates(3,:);
                robots    = gobjects(1,2);
                robots(1) = rectangle(ax,'Position',[sA(2)-1 sA(1)-1 1 1],'FaceColor','r','Curvature',1);
                robots(2) = rectangle(ax,'Position',[sB(2)-1 sB(1)-1 1 1],'FaceColor','g','Curvature',0.2);
                robots(3) = rectangle(ax,'Position',[sC(2)-1 sC(1)-1 1 1],'FaceColor','b','Curvature',0.2);
                rectangle(ax,'Position',[(obj.Destination(1, 2) - 1) (obj.Destination(1,1) - 1) 1 1],'FaceColor', 'y','Curvature',0.0);

                % time text
                totalExploredCells = sum(obj.NumExploredCells);
                totalCells = obj.Size(1) * obj.Size(2) - sum(obj.Grid==1,'all');
                coverage = totalExploredCells / totalCells * 100;
                title(ax,sprintf('Steps = %d, DD Reward = %d',obj.StepCount,obj.CurrentReward));
                ax.XTick = 0:1:obj.Size(1);
                ax.YTick = 0:1:obj.Size(2);
                ax.XTickLabel = {};
                ax.YTickLabel = {};
                axis(ax,'equal');
                ax.XLim = [0 obj.Size(1)];
                ax.YLim = [0 obj.Size(2)];
                ax.Box = 'on';
            end
            
            % update color map
            cmap = [255 255 255; ...    % white  (unexplored)
                    255 140 105; ...    % light red (explored by A)
                    152 251 152; ...    % light green (explored by B)
                    176 226 255; ...    % light blue (explored by C)
                    0 0 0]./255;        % black (obstacles)
            if all(obj.Grid~=0,'all')
                cmap = cmap(2:end,:);   % no unexplored cells
            end
            if isequal(obj.Obstacles,-1)
                cmap = cmap(1:end-1,:); % no obstacles
            end

            colormap(ax,cmap);
            
            % update cell colors
            cdata = obj.Grid;
            cells.CData = cdata;
            
            % update robot positions
            for idx = 1:3
                s = obj.States(idx,:);
                robots(idx).Position = [s(2)-1 s(1)-1 1 1];
            end
            
            % update info text
            totalExploredCells = sum(obj.NumExploredCells);
            totalCells = obj.Size(1) * obj.Size(2) - sum(obj.Grid==1,'all');
            coverage = totalExploredCells / totalCells * 100;
            ax.Title.String = sprintf('Steps = %d, DD Reward = %d',obj.StepCount,obj.CurrentReward);
            ax.XLim = [0 obj.Size(1)];
            ax.YLim = [0 obj.Size(2)];
            grid(ax,'on');
            
            drawnow();
        end
    end
end
