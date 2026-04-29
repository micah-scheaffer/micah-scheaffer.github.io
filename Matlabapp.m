classdef Matlabapp < matlab.apps.AppBase

    properties (Access = public)
        UIFigure            matlab.ui.Figure
        LeftPanel           matlab.ui.container.Panel
        LinkLengthsLabel    matlab.ui.control.Label
        L1Label             matlab.ui.control.Label
        L1Field             matlab.ui.control.NumericEditField
        L2Label             matlab.ui.control.Label
        L2Field             matlab.ui.control.NumericEditField
        L3Label             matlab.ui.control.Label
        L3Field             matlab.ui.control.NumericEditField
        JointAnglesLabel    matlab.ui.control.Label
        Th1Label            matlab.ui.control.Label
        Th1Field            matlab.ui.control.NumericEditField
        Th1Slider           matlab.ui.control.Slider
        Th2Label            matlab.ui.control.Label
        Th2Field            matlab.ui.control.NumericEditField
        Th2Slider           matlab.ui.control.Slider
        Th3Label            matlab.ui.control.Label
        Th3Field            matlab.ui.control.NumericEditField
        Th3Slider           matlab.ui.control.Slider
        MotionLabel         matlab.ui.control.Label
        TotalTimeLabel      matlab.ui.control.Label
        TotalTimeField      matlab.ui.control.NumericEditField
        TimeStepLabel       matlab.ui.control.Label
        TimeStepField       matlab.ui.control.NumericEditField
        ComputeButton       matlab.ui.control.Button
        AnimateButton       matlab.ui.control.Button
        StatusLabel         matlab.ui.control.Label
        RightPanel          matlab.ui.container.Panel
        TrajectoryAxes      matlab.ui.control.UIAxes
        PathAxes            matlab.ui.control.UIAxes
        AnimPanel           matlab.ui.container.Panel
    end

    properties (Access = private)
        Q_traj
        t_vec
        xyz
        robot_obj
        cached_L
        AnimAxes        % regular axes (not uiaxes) inside AnimPanel
    end

    methods (Access = private)

        function rb = buildRobot(app, L1, L2, L3)
            lnk(1) = Link([0, 0,  0,  0   ], 'modified');
            lnk(2) = Link([0, 0, L1, pi/2 ], 'modified');
            lnk(3) = Link([0, 0, L2,  0   ], 'modified');
            rb = SerialLink(lnk, 'name', '3-DOF Arm', 'tool', transl(L3, 0, 0));
            app.cached_L = [L1 L2 L3];
        end

        function setStatus(app, msg, clr)
            if nargin < 3, clr = [0.15 0.15 0.15]; end
            app.StatusLabel.Text      = msg;
            app.StatusLabel.FontColor = clr;
            drawnow;
        end

        function ComputeButtonPushed(app, ~)
            app.setStatus('Computing trajectory...');
            try
                L1    = app.L1Field.Value;
                L2    = app.L2Field.Value;
                L3    = app.L3Field.Value;
                q_end = deg2rad([app.Th1Field.Value, app.Th2Field.Value, app.Th3Field.Value]);
                T_tot = app.TotalTimeField.Value;
                dt    = app.TimeStepField.Value;

                assert(all([L1 L2 L3] > 0), 'Link lengths must be positive.');
                assert(T_tot > 0,            'Total time must be positive.');
                assert(dt > 0 && dt < T_tot, 'Time step must be in (0, total time).');
                assert(T_tot/dt <= 5000,     'Too many steps - increase dt or reduce T.');

                if isempty(app.robot_obj) || ~isequal([L1 L2 L3], app.cached_L)
                    app.robot_obj = app.buildRobot(L1, L2, L3);
                end

                app.t_vec  = (0:dt:T_tot)';
                app.Q_traj = jtraj([0 0 0], q_end, app.t_vec);

                T_all = app.robot_obj.fkine(app.Q_traj);
                n = size(app.Q_traj, 1);
                app.xyz = zeros(n, 3);
                for k = 1:n
                    if isa(T_all, 'SE3')
                        app.xyz(k,:) = T_all(k).t';
                    else
                        app.xyz(k,:) = T_all(1:3, 4, k)';
                    end
                end

                cla(app.TrajectoryAxes); hold(app.TrajectoryAxes, 'on');
                plot(app.TrajectoryAxes, app.t_vec, app.xyz(:,1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'X');
                plot(app.TrajectoryAxes, app.t_vec, app.xyz(:,2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y');
                plot(app.TrajectoryAxes, app.t_vec, app.xyz(:,3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z');
                hold(app.TrajectoryAxes, 'off');
                legend(app.TrajectoryAxes, 'X','Y','Z', 'Location','best');
                xlabel(app.TrajectoryAxes, 'Time (s)'); ylabel(app.TrajectoryAxes, 'Position (m)');
                title(app.TrajectoryAxes, 'End-Effector Position vs Time');
                xlim(app.TrajectoryAxes, [0 T_tot]);
                grid(app.TrajectoryAxes, 'on');

                cla(app.PathAxes); hold(app.PathAxes, 'on');
                plot3(app.PathAxes, app.xyz(:,1), app.xyz(:,2), app.xyz(:,3), 'm', 'LineWidth', 1.5);
                plot3(app.PathAxes, app.xyz(1,1),   app.xyz(1,2),   app.xyz(1,3),   'g.', 'MarkerSize', 15, 'DisplayName', 'Start');
                plot3(app.PathAxes, app.xyz(end,1), app.xyz(end,2), app.xyz(end,3), 'r.', 'MarkerSize', 15, 'DisplayName', 'End');
                hold(app.PathAxes, 'off');
                legend(app.PathAxes, 'Path','Start','End', 'Location','best');
                xlabel(app.PathAxes, 'X (m)'); ylabel(app.PathAxes, 'Y (m)'); zlabel(app.PathAxes, 'Z (m)');
                title(app.PathAxes, '3-D End-Effector Path');
                grid(app.PathAxes, 'on'); view(app.PathAxes, -45, 30);

                app.setStatus(sprintf('Done. %d steps, dt=%.3fs | End Effector final: (%.3f, %.3f, %.3f) m', ...
                    n, dt, app.xyz(end,1), app.xyz(end,2), app.xyz(end,3)), [1 1 1]);
            catch ME
                app.setStatus(['Error: ' ME.message], [0.8 0 0]);
            end
        end

        function AnimateButtonPushed(app, ~)
            if isempty(app.Q_traj)
                uialert(app.UIFigure, 'Run Compute Trajectory first.', 'No Data');
                return;
            end
            app.setStatus('Animating...');
            try
                r  = sum([app.L1Field.Value, app.L2Field.Value, app.L3Field.Value]) * 1.2;
                ws = [-r r -r r -r r];

                delete(findobj(app.AnimPanel, 'type', 'axes'));
                app.AnimAxes = axes('Parent', app.AnimPanel, ...
                    'Units', 'normalized', 'Position', [0.05 0.05 0.90 0.90]);
                axes(app.AnimAxes);
                try
                    app.robot_obj.plot(app.Q_traj, 'workspace', ws, 'trail', 'm-', ...
                        'delay', app.t_vec(2) - app.t_vec(1));
                catch
                    app.robot_obj.plot(app.Q_traj, 'workspace', ws);
                end
                app.setStatus('Animation complete.', [1 1 1]);
            catch ME
                app.setStatus(['Animation error: ' ME.message], [0.8 0 0]);
            end
        end

        function Th1SliderValueChanged(app, ~); app.Th1Field.Value = app.Th1Slider.Value; end
        function Th2SliderValueChanged(app, ~); app.Th2Field.Value = app.Th2Slider.Value; end
        function Th3SliderValueChanged(app, ~); app.Th3Field.Value = app.Th3Slider.Value; end
        function Th1FieldValueChanged(app, ~); app.Th1Slider.Value = max(-180, min(180, app.Th1Field.Value)); end
        function Th2FieldValueChanged(app, ~); app.Th2Slider.Value = max(-180, min(180, app.Th2Field.Value)); end
        function Th3FieldValueChanged(app, ~); app.Th3Slider.Value = max(-180, min(180, app.Th3Field.Value)); end

        function createComponents(app)
            app.UIFigure                  = uifigure('Visible','off');
            app.UIFigure.Position         = [60 60 1424 738];
            app.UIFigure.Name             = '3-DOF Robot Arm Simulator';
            app.UIFigure.HandleVisibility = 'on';

            app.LeftPanel          = uipanel(app.UIFigure);
            app.LeftPanel.Title    = 'Parameters';
            app.LeftPanel.Position = [10 10 280 718];

            app.LinkLengthsLabel            = uilabel(app.LeftPanel);
            app.LinkLengthsLabel.Position   = [10 673 200 22];
            app.LinkLengthsLabel.Text       = 'Link Lengths (m)';
            app.LinkLengthsLabel.FontWeight = 'bold';

            app.L1Label = uilabel(app.LeftPanel, 'Position', [10 645 40 22], 'Text', 'L1');
            app.L1Field = uieditfield(app.LeftPanel, 'numeric', 'Position', [55 645 100 22], 'Value', 1.0);
            app.L2Label = uilabel(app.LeftPanel, 'Position', [10 615 40 22], 'Text', 'L2');
            app.L2Field = uieditfield(app.LeftPanel, 'numeric', 'Position', [55 615 100 22], 'Value', 1.0);
            app.L3Label = uilabel(app.LeftPanel, 'Position', [10 585 40 22], 'Text', 'L3');
            app.L3Field = uieditfield(app.LeftPanel, 'numeric', 'Position', [55 585 100 22], 'Value', 1.0);

            app.JointAnglesLabel            = uilabel(app.LeftPanel);
            app.JointAnglesLabel.Position   = [10 552 220 22];
            app.JointAnglesLabel.Text       = 'Target Joint Angles (deg)';
            app.JointAnglesLabel.FontWeight = 'bold';

            app.Th1Label  = uilabel(app.LeftPanel, 'Position', [10 525 30 22], 'Text', 'θ1');
            app.Th1Field  = uieditfield(app.LeftPanel, 'numeric', 'Position', [45 525 80 22], 'Value', 30, ...
                            'ValueChangedFcn', createCallbackFcn(app, @Th1FieldValueChanged, true));
            app.Th1Slider = uislider(app.LeftPanel, 'Position', [10 508 250 3], 'Limits', [-180 180], 'Value', 30, ...
                            'ValueChangedFcn', createCallbackFcn(app, @Th1SliderValueChanged, true));

            app.Th2Label  = uilabel(app.LeftPanel, 'Position', [10 460 30 22], 'Text', 'θ2');
            app.Th2Field  = uieditfield(app.LeftPanel, 'numeric', 'Position', [45 460 80 22], 'Value', 45, ...
                            'ValueChangedFcn', createCallbackFcn(app, @Th2FieldValueChanged, true));
            app.Th2Slider = uislider(app.LeftPanel, 'Position', [10 443 250 3], 'Limits', [-180 180], 'Value', 45, ...
                            'ValueChangedFcn', createCallbackFcn(app, @Th2SliderValueChanged, true));

            app.Th3Label  = uilabel(app.LeftPanel, 'Position', [10 395 30 22], 'Text', 'θ3');
            app.Th3Field  = uieditfield(app.LeftPanel, 'numeric', 'Position', [45 395 80 22], 'Value', 60, ...
                            'ValueChangedFcn', createCallbackFcn(app, @Th3FieldValueChanged, true));
            app.Th3Slider = uislider(app.LeftPanel, 'Position', [10 378 250 3], 'Limits', [-180 180], 'Value', 60, ...
                            'ValueChangedFcn', createCallbackFcn(app, @Th3SliderValueChanged, true));

            app.MotionLabel            = uilabel(app.LeftPanel);
            app.MotionLabel.Position   = [10 335 200 22];
            app.MotionLabel.Text       = 'Motion Parameters';
            app.MotionLabel.FontWeight = 'bold';

            app.TotalTimeLabel = uilabel(app.LeftPanel, 'Position', [10 308 110 22], 'Text', 'Total Time (s)');
            app.TotalTimeField = uieditfield(app.LeftPanel, 'numeric', 'Position', [130 308 80 22], 'Value', 1.0);
            app.TimeStepLabel  = uilabel(app.LeftPanel, 'Position', [10 278 110 22], 'Text', 'Time Step (s)');
            app.TimeStepField  = uieditfield(app.LeftPanel, 'numeric', 'Position', [130 278 80 22], 'Value', 0.01);

            app.ComputeButton                 = uibutton(app.LeftPanel, 'push');
            app.ComputeButton.Position        = [15 235 250 35];
            app.ComputeButton.Text            = 'Compute Trajectory';
            app.ComputeButton.ButtonPushedFcn = createCallbackFcn(app, @ComputeButtonPushed, true);

            app.AnimateButton                 = uibutton(app.LeftPanel, 'push');
            app.AnimateButton.Position        = [15 190 250 35];
            app.AnimateButton.Text            = 'Animate Robot (Peter Corke)';
            app.AnimateButton.ButtonPushedFcn = createCallbackFcn(app, @AnimateButtonPushed, true);

            app.StatusLabel                   = uilabel(app.LeftPanel);
            app.StatusLabel.Position          = [10 10 258 170];
            app.StatusLabel.Text              = 'Set parameters and click Compute Trajectory.';
            app.StatusLabel.WordWrap          = 'on';
            app.StatusLabel.VerticalAlignment = 'top';
            app.StatusLabel.FontAngle         = 'italic';

            app.RightPanel          = uipanel(app.UIFigure);
            app.RightPanel.Title    = 'Results';
            app.RightPanel.Position = [300 10 500 718];

            app.TrajectoryAxes = uiaxes(app.RightPanel, 'Position', [15 375 470 320]);
            title(app.TrajectoryAxes, 'End-Effector Position vs Time');
            xlabel(app.TrajectoryAxes, 'Time (s)'); ylabel(app.TrajectoryAxes, 'Position (m)');
            grid(app.TrajectoryAxes, 'on');

            app.PathAxes = uiaxes(app.RightPanel, 'Position', [15 15 470 345]);
            title(app.PathAxes, '3-D End-Effector Path');
            xlabel(app.PathAxes, 'X (m)'); ylabel(app.PathAxes, 'Y (m)'); zlabel(app.PathAxes, 'Z (m)');
            grid(app.PathAxes, 'on'); view(app.PathAxes, -45, 30);

            app.AnimPanel          = uipanel(app.UIFigure);
            app.AnimPanel.Title    = 'Robot Animation';
            app.AnimPanel.Position = [810 10 604 718];

            app.UIFigure.Visible = 'on';
        end

    end

    methods (Access = public)
        function app = Matlab2app
            app.cached_L = [-1 -1 -1];
            createComponents(app);
            registerApp(app, app.UIFigure);
            if nargout == 0; clear app; end
        end

        function delete(app)
            delete(app.UIFigure);
        end
    end

end