function robot = buildTiledRobot(n, opts)

arguments
    n (1,1) {mustBeInteger, mustBePositive}
    opts.SegmentLength (1,1) double = 0.1016      % [m] full section length
    opts.Twist         (1,1) double = pi/2      % [rad] rotation about Z per section
    opts.JointLimit    (1,1) double = pi/2      % [rad] +/- range of motion
    opts.Radius        (1,1) double = 0.0381      % [m]
    opts.Mass          (1,1) double = 0.3       % [kg] per full section
end

L = opts.SegmentLength;
r = opts.Radius;
m = opts.Mass;

robot = rigidBodyTree('DataFormat','row','MaxNumBodies',n+2);
robot.Gravity = [0 0 -9.81];


mount = rigidBody('mount');
jm = rigidBodyJoint('mountFixed','fixed');
setFixedTransform(jm, se3(90*pi/180, "roty"));
mount.Joint = jm;
addVisual(mount,'Cylinder',[r L/2], trvec2tform([0 0 L/4]));
addBody(robot, mount, 'base');

parent = 'mount';
for k = 1:n
    Lk = L;
    mk = m * Lk / L;

    body = rigidBody(sprintf('seg%d',k));
    jnt  = rigidBodyJoint(sprintf('joint%d',k),'revolute');
    jnt.JointAxis      = [1 0 0];
    jnt.PositionLimits = [-opts.JointLimit opts.JointLimit];

    if k == 1
        Tfix = trvec2tform([0 0 L/2]);
    else
        Tfix = trvec2tform([0 0 L]) * axang2tform([0 0 1 opts.Twist]);
    end
    setFixedTransform(jnt, Tfix);
    body.Joint = jnt;

    Ixx = mk*(3*r^2 + Lk^2)/12 + mk*(Lk/2)^2;
    body.Mass         = mk;
    body.CenterOfMass = [0 0 Lk/2];
    body.Inertia      = [Ixx Ixx mk*r^2/2 0 0 0];

    addVisual(body,'Cylinder',[r Lk],        trvec2tform([0 0 Lk/2]));
    addVisual(body,'Cylinder',[r*1.3 2.2*r], axang2tform([0 1 0 pi/2]));

    addBody(robot, body, parent);
    parent = body.Name;
end

tip = rigidBody('tip');
jt  = rigidBodyJoint('tipFixed','fixed');
setFixedTransform(jt, trvec2tform([0 0 L/2]));
tip.Joint        = jt;
tip.Mass         = 0;
tip.CenterOfMass = [0 0 0];
tip.Inertia      = zeros(1,6);
addBody(robot, tip, parent);
end