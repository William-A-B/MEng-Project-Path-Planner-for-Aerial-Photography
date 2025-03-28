dubConnObj = dubinsConnection(MinTurningRadius=2);

startPose = [0 0 0];
goalPose = [4 4 pi];

[pathSegObj, pathCosts] = connect(dubConnObj,startPose,goalPose);

show(pathSegObj{1})
