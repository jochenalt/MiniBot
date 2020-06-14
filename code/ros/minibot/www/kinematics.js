/**
 * @author Jochen Alt
 */


var Kinematics = Kinematics || {
};


Kinematics.Init = function(options) {
  options = options || {};
  var ros = options.ros;
  var jointNames = [];
  var linkNames = [];
  var kinematicGroup = 'minibot_arm';
  var kinematicGroupLinks = [];
  var kinematicGroupJoints = [];
  var zeroPose;

  var initializeURDF = function(param) {
    var parser = new DOMParser();
    var xmlDoc = parser.parseFromString(param, 'text/xml');
    var XPATH_FIRST_ORDERED_NODE_TYPE = 9;
    var robotXml = xmlDoc.evaluate('//robot', xmlDoc, null, XPATH_FIRST_ORDERED_NODE_TYPE, null).singleNodeValue;
    for (var nodes = robotXml.childNodes, i = 0; i < nodes.length; i++) {
      var node = nodes[i];
      if(node.tagName == 'joint'){
        if(node.getAttribute('type')!=='fixed'){
          var mimicsJoint = node.getElementsByTagName('mimic')[0];
          if(node.getAttribute('type')=='revolute'){
            if (mimicsJoint == null) {
              jointNames[jointNames.length] = node.getAttribute('name');
              linkNames[linkNames.length] = node.getAttribute('name').replace('_joint', '_link');
            }
          }
        }
      };
    };
  };

  var initializeSRDF = function(param) {
    var parser = new DOMParser();
    var xmlDoc = parser.parseFromString(param, 'text/xml');
    var XPATH_FIRST_ORDERED_NODE_TYPE = 9;
    var robotXml = xmlDoc.evaluate('//robot', xmlDoc, null, XPATH_FIRST_ORDERED_NODE_TYPE, null).singleNodeValue;
    for (var nodes = robotXml.childNodes, i = 0; i < nodes.length; i++) {
      var node = nodes[i];
      if ((node.tagName == 'group') && node.getAttribute('name') == kinematicGroup) {
        for (var j = 0;j<node.childNodes.length;j++) {
            var link = node.childNodes[j];
            if (link.tagName == 'link') {
              kinematicGroupLinks[kinematicGroupLinks.length] = link.getAttribute('name');
              kinematicGroupJoints[kinematicGroupJoints.length] = link.getAttribute('name').replace('_link','_joint');
            }
        }
      }
    }
  }

  function computeFK(jointState, success, failure) {
    var jointNames = jointState.name.slice(0,6);
    var jointStates = new ROSLIB.Message({
      name: jointNames,
      position: jointState.position.slice(0,6)
    });

    var request = new ROSLIB.ServiceRequest({
      header: {
        seq: 0,
        stamp: 0,
        frame_id: 'base_link'
     },
     fk_link_names: kinematicGroupLinks,
     robot_state: {
        joint_state: jointStates
     }
    });

    var getPositionFK = new ROSLIB.Service({
        ros : ros,
        name : '/compute_fk',
        serviceType : 'moveit_msgs/GetPositionFK'
    });

    getPositionFK.callService(request, 
      function(result) {
        if (result.error_code.val == ErrorCode.MOVEIT.SUCCESS) {
          pose = result.pose_stamped[7].pose;
          success(pose);
        }
        else {
          failure(result.error_code.val);
        }
      }, 
      function (result) {
        failure(result);
      });
  };

  function computeAllFK(jointState, success, failure) {
    var jointNames = jointState.name.slice(0,6);
    var jointStates = new ROSLIB.Message({
      name: jointNames,
      position: jointState.position.slice(0,6)
    });

    var request = new ROSLIB.ServiceRequest({
      header: {
        seq: 0,
        stamp: 0,
        frame_id: 'base_link'
     },
     fk_link_names: kinematicGroupLinks,
     robot_state: {
        joint_state: jointStates
     }
    });

    var getPositionAllFK = new ROSLIB.Service({
        ros : ros,
        name : '/compute_all_fk',
        serviceType : 'minibot/GetPositionAllFK'
    });

    getPositionAllFK.callService(request, 
      function(result) {
        if (result.error_code.val == ErrorCode.MOVEIT.SUCCESS) {
          success(result);
        }
        else {
          failure(result.error_code.val);
        }
      }, 
      function (result) {
        failure(result);
      });
  };

  var computeIK = function(jointStates, tcpPose, success, failure) {
    var linkNames = [];
    var jointNames = [];
    var jointAngleValues = [];

    var jointNames = kinematicGroupJoints.slice(0,6);
    var linkNames = kinematicGroupLinks.slice(0,6);
   
    var request = new ROSLIB.ServiceRequest({
      ik_request: {
        group_name: 'minibot_arm',  // as referred to in kinematics.yaml and defined in minibot.xacro
        robot_state: {
            joint_state: jointStates
        },
        pose_stamped: {
          header: {
            seq: 0,
            stamp: 0,
            frame_id: 'base_link'
          },
          pose : {
            position: { 
              x: tcpPose.position.x,
              y: tcpPose.position.y, 
              z: tcpPose.position.z },
            orientation: { 
              x: tcpPose.orientation.x, 
              y: tcpPose.orientation.y, 
              z: tcpPose.orientation.z, 
              w: tcpPose.orientation.w }
          }
        }  
      }
    });

    var getPositionIK  = new ROSLIB.Service({
      ros : ros,
      name : '/compute_ik',
      serviceType : 'moveit_msgs/GetPositionIK'
    });

    getPositionIK.callService(request, function(result) {
      if (result.error_code.val == ErrorCode.MOVEIT.SUCCESS) {
          success(result.solution.joint_state);
        }
        else
          failure(result.error_code.val);
      },
      function (result) {
        failure (result);
      }
    );
  }


  var computeAllIK = function(jointStates, tcpPose, success, failure) {
    var linkNames = [];
    var jointNames = [];
    var jointAngleValues = [];

    var jointNames = kinematicGroupJoints.slice(0,6);
    var linkNames = kinematicGroupLinks.slice(0,6);
   
    var request = new ROSLIB.ServiceRequest({
      ik_request: {
        group_name: 'minibot_arm',  // as referred to in kinematics.yaml and defined in minibot.xacro
        robot_state: {
            joint_state: {
              name: jointStates.name.slice(0,6),
              position: jointStates.position.slice(0,6),
            }
        },
        pose_stamped: {
          pose : {
            position: { 
              x: tcpPose.position.x,
              y: tcpPose.position.y, 
              z: tcpPose.position.z },
            orientation: { 
              x: tcpPose.orientation.x, 
              y: tcpPose.orientation.y, 
              z: tcpPose.orientation.z, 
              w: tcpPose.orientation.w }
          }
        }  
      }
    });

    var getPositionIK  = new ROSLIB.Service({
      ros : ros,
      name : '/compute_all_ik',
      serviceType : 'moveit_msgs/GetPositionIK'
    });

    getPositionIK.callService(request, function(result) {
      if (result.error_code.val == ErrorCode.MOVEIT.SUCCESS) {
          success(result.solution);
        }
        else
          failure(result.error_code.val);
      },
      function (result) {
        failure (result);
      }
    );
  }

  
  function getZeroPose(successCallback, failureCallback) {
    computeFK(getZeroJointState(), successCallback, failureCallback);
  };

  function getZeroJointState() {
    var jointNames = kinematicGroupJoints.slice(0,6);
    var jointState = new ROSLIB.Message({
      name: jointNames,
      position: [0,0,0,0,0,0]
    });
    return jointState;
  }

    // initialize URDF 
  var robotDescription = new ROSLIB.Param({
    ros : ros,
    name : 'robot_description'
  });

  robotDescription.get(initializeURDF);
  // initialize SRDF
  var robotDescriptionSemantic = new ROSLIB.Param({
    ros : ros,
    name : 'robot_description_semantic'
  });

  robotDescriptionSemantic.get(initializeSRDF);


  // exposed inner functions
  return {
        computeFK: computeFK,
        computeAllIK: computeAllIK,
        computeAllFK: computeAllFK,
        getZeroPose : getZeroPose,
        getZeroJointState : getZeroJointState
  };
};
