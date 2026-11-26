classdef CustomGravityTorqueBlock < matlab.System
% This class is for internal use only. It may be removed in the future.

%CustomGravityTorqueBlock Gravity Torque block system object
%   Inputs:
%       Config  - Joint configuration vector
%       Gravity - 3-element gravity vector, expressed in the base frame.
%                 It is applied at every step, so it may vary over time.

% Copyright 2018-2024 The MathWorks, Inc.

%#codegen

    properties(Nontunable)

        %TreeStruct - Struct from which wrapper SysObj builds the TreeInternal
        TreeStruct = 0;

        %TreeInternal - A robotics.manip.internal.RigidBodyTree instantiated from TreeStruct
        TreeInternal
    end

    methods
        function obj = CustomGravityTorqueBlock(varargin)
        %CustomGravityTorqueBlock Constructor for Gravity Torque block system object

        % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
        %setupImpl Perform one-time calculations, such as computing constants

        % Assign the robot internal property by instantiating from the structure
            obj.TreeInternal = robotics.manip.internal.RigidBodyTree(obj.TreeStruct.MaxNumBodies, obj.TreeStruct);
        end

        function jointTorq = stepImpl(obj,q,gravity)
        %stepImpl Call RBT object to generate a solution

            velnum = obj.TreeStruct.VelocityNumber;
            jointTorq = cast(zeros(velnum, 1), like=q);
            fExt = zeros(6, obj.TreeStruct.NumBodies);

            % Update gravity for this step (expressed in the base frame)
            obj.TreeInternal.Gravity = reshape(double(gravity), 1, 3);

            jointTorq(:) = cast(robotics.manip.internal.RigidBodyTreeDynamics.inverseDynamics(...
                obj.TreeInternal, double(q), zeros(velnum, 1),...
                zeros(velnum, 1), fExt),like=q);
        end

        function resetImpl(~)
        %resetImpl Initialize / reset discrete-state properties
        end

        function validateInputsImpl(~,q,gravity)
        %validateInputsImpl Validate inputs to the step method at initialization
            validateattributes(q,{'single','double'},{'vector'},'CustomGravityTorqueBlock','Config');
            validateattributes(gravity,{'single','double'},{'vector','numel',3},'CustomGravityTorqueBlock','Gravity');
        end

        function flag = isInputSizeMutableImpl(~,~)
        %isInputSizeMutableImpl Specify input size mutability
        %   Return false if input size cannot change
        %   between calls to the System object
            flag = false;
        end

        function flag = isInputDataTypeMutableImpl(~,~)
        %isInputDataTypeMutableImpl Specify input type mutability
        %   Return false if input data type cannot change
        %   between calls to the System object
            flag = false;
        end

        function num = getNumInputsImpl(~)
        %getNumInputsImpl Define total number of inputs
            num = 2;
        end

        function [name1,name2] = getInputNamesImpl(~)
        %getInputNamesImpl Label the input ports on the block
            name1 = 'Config';
            name2 = 'Gravity';
        end

        function num = getNumOutputsImpl(~)
        %getNumOutputsImpl Define total number of outputs
            num = 1;
        end

        function out = getOutputSizeImpl(obj)
        %getOutputSizeImpl Return size for each output port
            out = [obj.TreeStruct.VelocityNumber 1];
        end

        function out = getOutputDataTypeImpl(obj)
        %getOutputDataTypeImpl Return data type for each output port
            out = propagatedInputDataType(obj,1);
        end

        function out = isOutputComplexImpl(~)
        %isOutputComplexImpl Return true for each output port with complex data
            out = false;
        end

        function out = isOutputFixedSizeImpl(~)
        %isOutputFixedSizeImpl Return true for each output port with fixed size
            out = true;
        end
    end

    methods(Access = protected, Static)
        function header = getHeaderImpl
        %getHeaderImpl Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename("class"));
        end

        function group = getPropertyGroupsImpl
        %getPropertyGroupsImpl Define property section(s) for System block dialog
            mainGroup = matlab.system.display.SectionGroup(...
                'Title','Initial conditions', ...
                'PropertyList',{'TreeStruct'});

            group = mainGroup;
        end

        function flag = showSimulateUsingImpl
        %showSimulateUsingImpl Return false if simulation mode hidden in System block dialog
            flag = true;
        end
    end
end