classdef RobotData < handle
    properties (SetAccess = public)
        robotPose(1,3) double                    %this is where the robot is
        nextRobotPose(1,3) double               %this is where the robot will be at
                                                %the end of current control loop
        robotNode double                    % Current robot node
        nextMoveTarget                      %this is the node at the root-end of 
                                            %the edge contains next robotPose
        distanceFromNextRobotPoseToNextMoveTarget(1,1) double %this holds the distance from nextRobotPose to  nextMoveTarget along the trajectory the robot                                    
                                            % will be following at that time
        robotMaxVelocity double             % the maximum robot speed Robot can reach
        robotMinVelocity double             % The mimumum robot speed that robot can move on an edge
        moving                        %set to true when the robot starts moving
        currentMoveInvalid            %this gets set to true if nextMoveTarget
                                            %has become invalid due to dynamic obstacles        
        robotMovePath                       % this holds the path the robot has followed
                                            % from the start of movement up through robotPose
        numRobotMovePoints double           % the number of points in robotMovePath
        robotLocalPath                      %this holds the path between robotPose and
                                            % nextRobotPose (not including the former)
        numLocalMovePoints  double          % the number of points in robotLocalPath
        robotEdge                           % this is the edge that contians the trajectory that the robot is currently following
                                    
        robotEdgeUsed                       %true if the robotEdge is populated
        distAlongRobotEdge(1,1) double      %the current distance that the robot "will be" along robotEdge (i.e., next time slice 
        timeAlongRobotEdge (1,1) double     % the current rime that the robot "will be" along robotEdge (i.e., next time slice
        %the following things are only used to help with visualization
        
        robotEdgeForPlottingUsed  
        robotEdgeForPlotting
        distAlongRobotEdgeForPlotting double
        timeAlongRobotEdgeForPlotting double
    end
    methods (Access = public)
        function R = RobotData(rPos, nMoveTarget, maxPathNodes, maxVelocity, minVelocity)
        R.robotPose = rPos;
        R.nextRobotPose = rPos;
        R.nextMoveTarget= nMoveTarget;
        R.distanceFromNextRobotPoseToNextMoveTarget=0.0;
        R.robotNode=1;
        
        R.moving = false;
        R.currentMoveInvalid = false;
        R.robotMaxVelocity= maxVelocity;
        R.robotMinVelocity=minVelocity;
        
        R.robotMovePath = zeros(maxPathNodes, length(rPos));
        R.robotMovePath(1,:)= rPos;
        R.numRobotMovePoints = 1;
        
        R.robotLocalPath = zeros(maxPathNodes, length(rPos));
        R.numLocalMovePoints=0;
        
        R.robotEdgeUsed = false;
        R.distAlongRobotEdge = 0.0;
        R.timeAlongRobotEdge = 0.0;

        R.robotEdgeForPlottingUsed = false;
        R.distAlongRobotEdgeForPlotting = 0.0;
        R.timeAlongRobotEdgeForPlotting = 0.0;
        
        end
        
    end
end