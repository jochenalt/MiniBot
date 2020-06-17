/**
 * @author Jochen Alt
 */
var RobotPanel = RobotPanel || {};


/** 
 * @constructor
 * @param options 
 *   * 
 */

RobotPanel.Init = function(options) {
  options = options || {};
  var ros = options.ros;
  // interface to other panels
  var kinematicsPanel;
  var programmePanel;
  var poseStoragePanel;
  var botUrdfView;

  var setKinematicsPanel = function(panel) {
    kinematicsPanel = panel;
  }

  var setPoseStorePanel = function(panel) {
    poseStorePanel = panel;
  }

  var setProgrammePanel = function(panel) {
    programmePanel = panel;
  }


  var initialize = function(backgroundColor) {
    // Setup a client to listen to TFs.
    var tfClient = new ROSLIB.TFClient({
      ros: ros,
      angularThres: 0.0001, // [rad] angular threshold for rotations before th UI reacts
      transThres: 0.0001, // [m] translation threshold, 0.1 mm
      fixedFrame: 'base_link', // all coordinates are relative to world
      rate: 17.0 // update frequency [Hz]
    });

    var w = document.getElementById("RobotView").offsetWidth;
    var h = document.getElementById("RobotView").offsetHeight;
    if (botUrdfView)
      botUrdfView.stop();
    botUrdfView = new ROS3D.Viewer({
      divID: 'urdf',
      width: w,
      height: h,
      antialias: true,
      background: backgroundColor,
      cameraPose: {
        x: 0.6,
        y: 0.6,
        z: 0.8
      },
      displayPanAndZoomFrame: false, // no coord frame in the origin
      lineTypePanAndZoomFrame: false,
      antialias: true
    });

    // Add a grid.
    botUrdfView.addObject(new ROS3D.Grid({
      num_cells: 5,
      color: '#343A30',
      lineWidth: 1,
      cellSize: 0.1
    }));

    // Setup the direct kinematics panel
    var urdfClient = new ROS3D.UrdfClient({
      ros: ros,
      tfClient: tfClient,
      path: 'http://' + serverName + ':8085/',
      rootObject: botUrdfView.scene,
      loader: ROS3D.COLLADA_LOADER
    });
    // Setup the marker client.
    var startMarkerClient = new ROS3D.InteractiveMarkerClient({
      ros: ros,
      tfClient: tfClient,
      // hidden : true,
      topic: '/markers',
      camera: botUrdfView.camera,
      rootObject: botUrdfView.selectableObjects
    });
  }

  // exposed inner functions
  return {
    initialize: initialize
  };
};