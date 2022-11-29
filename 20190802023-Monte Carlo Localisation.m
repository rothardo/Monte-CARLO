ipaddress = '192.168.2.150';
rosinit(ipaddress,11311);


load officemap.mat
show(map)

odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = map;

tftree = rostf;
waitForTransform(tftree,'/base_link','/base_scan');
sensorTransform = getTransform(tftree,'/base_link', '/base_scan');

laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

rangeFinderModel.SensorPose = ...
    [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];

laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');

[velPub,velMsg] = ...
    rospublisher('/cmd_vel','geometry_msgs/Twist');

amcl = monteCarloLocalization;
amcl.UseLidarScan = true;

amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;

amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;

amcl.ParticleLimits = [500 5000];
amcl.GlobalLocalization = false;
amcl.InitialPose = ExampleHelperAMCLGazeboTruePose;
amcl.InitialCovariance = eye(3)*0.5;

visualizationHelper = ExampleHelperAMCLVisualization(map);

wanderHelper = ...
    ExampleHelperAMCLWanderer(laserSub, sensorTransform, velPub, velMsg);

numUpdates = 60;
i = 0;
while i < numUpdates
    scanMsg = receive(laserSub);
    odompose = odomSub.LatestMessage;
    scan = lidarScan(scanMsg);
    
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];
    
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scan);
    
    wander(wanderHelper);
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amcl, estimatedPose, scan, i)
    end
    
end

stop(wanderHelper);
rosshutdown
