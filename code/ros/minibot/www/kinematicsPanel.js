/**
 * @author Jochen Alt
 */

var KinematicsPanel = KinematicsPanel || {
};

KinematicsPanel.Init = function(options) {
  options = options || {};
  var ros = options.ros;
  var kinematicsUtils = options.kinematics;
  var paramName                   = 'robot_description';
  var readTopicName               = '/joint_states';                              // 20Hz topic that gives the current state of the joints, publish constantly
  var jointInputTopicName         = '/move_group/fake_controller_joint_states'    // topic that gives changes of the joint values, published only when a change occurs
  var receiveTcpTopicName         = '/tcp/update'                                 // topic to receive the tcp out joint states (coming from core.py)
  var tcpInputTopicName           = '/tcp/input/update'                           // tcp sliders publish the change here
  
  var sliders = [];
  var inputs = [];
  var coordInputs = [];
  var coordSlider = [];
  var orientationInputs = [];
  var orientationSliders = [];
  var solutionsIK = [];
  var currentIkSolutionIdx = 0;

  var minVal  = [];
  var maxVal  = [];

  var param = new ROSLIB.Param({
    ros : ros,
    name : paramName
  });

  var loadJointModel = function(param) {
    var parser = new DOMParser();
    var xmlDoc = parser.parseFromString(param, 'text/xml');
    var XPATH_FIRST_ORDERED_NODE_TYPE = 9;
    var robotXml = xmlDoc.evaluate('//robot', xmlDoc, null, XPATH_FIRST_ORDERED_NODE_TYPE, null).singleNodeValue;
    for (var nodes = robotXml.childNodes, i = 0; i < nodes.length; i++) {
        var node = nodes[i];
        if(node.tagName==='joint'){
            if(node.getAttribute('type')!=='fixed'){
                var min, max, val, mimicsJoint;
                mimicsJoint = node.getElementsByTagName('mimic')[0];
                if(node.getAttribute('type')==='continuous'){
                    min = -Math.PI;
                    max = Math.PI;
                }else{
                    var limit = node.getElementsByTagName('limit')[0];
                    min = parseFloat(limit.getAttribute('lower'));
                    max = parseFloat(limit.getAttribute('upper'));
                }           

                // set value in the middle (just in case, later on it will be set as published by  jointStatePublisher
                val = (max + min) / 2;

                // display sliders only for non mimicing joints 
                if (mimicsJoint == null) {
                  var name = node.getAttribute('name');
                
                  // fetch input element, set callbacks and common properties
                  var textInput = document.getElementById(name + '_text');
                  textInput.setAttribute('name', name + '_text');
                  textInput.setAttribute('value', Utils.rad2View(val));
                  textInput.setAttribute('step', Utils.angleSteps());

                  textInput.onblur = callBackJointInput;
                  textInput.onChange = callBackJointInput;
                  textInput.onkeyup  = callBackJointInput;
                  inputs[ inputs.length ] = textInput;

                  // fetch slider element, set callbacks and common properties
                  var sliderInput = document.getElementById(name + '_slider');
                  sliderInput.setAttribute('name', name + '_slider');
                  sliderInput.setAttribute('type', 'range');
                  sliderInput.setAttribute('min', Utils.rad2View(min));
                  sliderInput.setAttribute('max', Utils.rad2View(max));
                  sliderInput.setAttribute('step', Utils.angleSteps());
                  sliderInput.setAttribute('value', Utils.rad2View(val));

                  sliderInput.oninput  = callBackJointInput;
                  sliders[ sliders.length ] = sliderInput;

                  // store the limits
                  minVal[minVal.length] = min;
                  maxVal[maxVal.length] = max;
                }
            }
        }

        coordInputs = [         document.getElementById("coord_x_text"), 
                                document.getElementById("coord_y_text"),
                                document.getElementById("coord_z_text") ];
        coordSliders = [        document.getElementById("coord_x_slider"), 
                                document.getElementById("coord_y_slider"),
                                document.getElementById("coord_z_slider") ];

        orientationInputs = [   document.getElementById("orientation_x_text"), 
                                document.getElementById("orientation_y_text"),
                                document.getElementById("orientation_z_text") ];
        orientationSliders = [  document.getElementById("orientation_x_slider"), 
                                document.getElementById("orientation_y_slider"),
                                document.getElementById("orientation_z_slider") ];

        for (var index = 0;index < 3; index++) {
          coordInputs[index].setAttribute('name', coordInputs[index].id);
          coordInputs[index].setAttribute('value', 0);
          coordInputs[index].setAttribute('step', Utils.distanceSteps());
          coordInputs[index].onblur   = callbackCartesicInput;
          coordInputs[index].onchange = callbackCartesicInput;
          coordInputs[index].onkeyup  = callbackCartesicInput;

          coordSliders[index].setAttribute('name', coordSliders[index].id);
          coordSliders[index].setAttribute('value', 0);
          coordSliders[index].setAttribute('step', Utils.distanceSteps());
          coordSliders[index].oninput  = callbackCartesicInput;

          orientationInputs[index].setAttribute('name', orientationInputs[index].id);
          orientationInputs[index].setAttribute('value', 0);
          orientationInputs[index].setAttribute('step', Utils.angleSteps());
          orientationInputs[index].onblur   = callbackCartesicInput;
          orientationInputs[index].onchange = callbackCartesicInput;
          orientationInputs[index].onkeyup  = callbackCartesicInput;

          orientationSliders[index].setAttribute('name', orientationSliders[index].id);
          orientationSliders[index].setAttribute('value', 0);
          orientationSliders[index].setAttribute('step', Utils.angleSteps());
          orientationSliders[index].setAttribute('min',Utils.rad2View(-Math.PI));
          orientationSliders[index].setAttribute('max',Utils.rad2View(Math.PI));
          orientationSliders[index].oninput = callbackCartesicInput;
        };
        coordSliders[0].setAttribute('max',500);
        coordSliders[0].setAttribute('min',-100);
        coordSliders[1].setAttribute('max',400);
        coordSliders[1].setAttribute('min',-400);

        coordSliders[2].setAttribute('max',600);
        coordSliders[2].setAttribute('min',0);
    }  

    var listenToTCP  = new ROSLIB.Topic({
      ros : ros,
      name : receiveTcpTopicName,
      messageType : 'geometry_msgs/PoseStamped'
    });

    listenToTCP.subscribe(function(tcpPose) {
        // Utils.queueAtMutex("blockTcp",setTcpView, tcpPose);
        setTcpView(tcpPose);
    });

    var listenToJointState  = new ROSLIB.Topic({
      ros : ros,
      name : jointInputTopicName,
      messageType : 'sensor_msgs/JointState'
    });

    listenToJointState.subscribe(function(jointState) {
        // Utils.queueAtMutex("blockJoint",setJointView, jointState);
        setJointView (jointState);
    });

    initErrorMessages()
    refresh()
  };

  var initErrorMessages = function() {
   // listen to error messages
    var listenToErrors  = new ROSLIB.Topic({
      ros : ros,
      name : '/messages/err',
      messageType : 'std_msgs/String'
    });

    listenToErrors.subscribe(function(msg) {
        displayErr(msg.data)
    });

    var listenToInfo  = new ROSLIB.Topic({
      ros : ros,
      name : '/messages/info',
      messageType : 'std_msgs/String'
    });

    listenToInfo.subscribe(function(msg) {
        displayInfo(msg.data)
    });

    var listenToWarn   = new ROSLIB.Topic({
      ros : ros,
      name : '/messages/warn',
      messageType : 'std_msgs/String'
    });

    listenToWarn.subscribe(function(msg) {
        displayWarn(msg.data)
    });
  } 

  function switchAngleUnit(toGrad) {
    Utils.switchAngleUnit(toGrad);

    var min = -Math.PI/2;
    var max = Math.PI/2;

    for(var idx = 0;idx<3;idx++) {
      orientationInputs[idx].setAttribute('min', Utils.rad2View(min));
      orientationInputs[idx].setAttribute('max', Utils.rad2View(max));
      orientationSliders[idx].setAttribute('min', Utils.rad2View(min));
      orientationSliders[idx].setAttribute('max', Utils.rad2View(max));
    }
    for (var idx = 0;idx < sliders.length;idx++) {
      sliders[idx].setAttribute('min', Utils.rad2View(minVal[idx]));
      sliders[idx].setAttribute('max', Utils.rad2View(maxVal[idx]));

      inputs[idx].setAttribute('min', Utils.rad2View(minVal[idx]));
      inputs[idx].setAttribute('max', Utils.rad2View(maxVal[idx]));
    }

    if (toGrad == 1) {
      document.getElementById("orientXUnitText").innerHTML = "x [GRAD]" 
      document.getElementById("orientYUnitText").innerHTML = "y [GRAD]" 
      document.getElementById("orientZUnitText").innerHTML = "z [GRAD]" 
    }
    else {
      document.getElementById("orientXUnitText").innerHTML = "x [RAD]" 
      document.getElementById("orientYUnitText").innerHTML = "y [RAD]" 
      document.getElementById("orientZUnitText").innerHTML = "z [RAD]" 
    }
    // update the content of the input fields
    refresh();
  }

  var setTcpView = function (tcpPose) {

      // convert the world frame into view frame 
      var pos = [ tcpPose.pose.position.x, tcpPose.pose.position.y, tcpPose.pose.position.z ];
      coordInputs[0].value = Utils.distance2View(pos[0]);
      coordInputs[1].value = Utils.distance2View(pos[1]);
      coordInputs[2].value = Utils.distance2View(pos[2]);
      coordSliders[0].value =  coordInputs[0].value;
      coordSliders[1].value =  coordInputs[1].value;
      coordSliders[2].value =  coordInputs[2].value;

      // convert the world frame into view frame 
      var orientWorld = Utils.quaternion2Euler(tcpPose.pose.orientation);

      orientationInputs[0].value = Utils.rad2View(orientWorld.x);
      orientationInputs[1].value = Utils.rad2View(orientWorld.y);
      orientationInputs[2].value = Utils.rad2View(orientWorld.z);
      orientationSliders[0].value =  orientationInputs[0].value;
      orientationSliders[1].value =  orientationInputs[1].value;
      orientationSliders[2].value =  orientationInputs[2].value;
  }

  var setJointView = function(msgJointState) {
    var jointIdx = 0;

    for (var index = 0;(index < msgJointState.name.length) && (jointIdx < sliders.length);index++) {
      var msg_joint_name = msgJointState.name[index];
      var joint_name = sliders[jointIdx].name;
      joint_name = joint_name.substring(0,msg_joint_name.length);
      if (msg_joint_name == joint_name) {
        sliders[jointIdx].value = Utils.rad2View(msgJointState.position[index]);
        inputs[jointIdx].value = Utils.rad2View(msgJointState.position[index]);            
        jointIdx++;            
      } 
    }
  }

  // callback when a slider/input changes
  var callBackJointInput = function(event) {
    var name = event.target.name;
    var target;
    if( name.indexOf('_text') >= 0) {
        target = name.replace('_text', '_slider');
    }else{
        target = name.replace('_slider', '_text');
    }

    // set the same value in sibbling widget (slider vs input)
    document.getElementById(target).value = Utils.angleRound(parseFloat(event.target.value)); 

    // retrieve current joint
    var newJointState = getCurrentJointState();

    // and publish joint state, but throttled  
    Utils.callThrottler("newJoint", Constants.Kinematics.MAX_KINEMATICS_RATE, function(params) {
      jointInputTopic.publish(params);      
    }, newJointState);

    // block joint input from kinematics while we turn the sliders (and 0.5 seconds afterwards)
    // Utils.stopMutex("blockJoint");
    // Utils.callDelay("blockJoint", Constants.Kinematics.BLOCK_UI_INPUT_TIME, function() {
    //  Utils.releaseMutex("blockJoint", setJointView );
    //});
  }

  var jointInputTopic  = new ROSLIB.Topic({
      ros : ros,
      name : jointInputTopicName,
      messageType : 'sensor_msgs/JointState'
    });
  

  // returns a ros object JointState with the names of the joints and their positions
  var getCurrentJointState = function() {
    var names = [];
    var values = [];
    for(var index = 0; index < sliders.length; index++) {
      var slider = sliders[index];
      var joint_name = slider.name;
      joint_name = joint_name.substring(0,joint_name.indexOf('_slider'));
      names[ names.length ] = joint_name;
      values[ values.length ] = Utils.view2Rad(parseFloat(slider.value));
    }

    var jointState = new ROSLIB.Message({
      name: names, position: values
    });
    return jointState;
  }

  var setIKSolutions = function (solutions) {
    solutionsIK = solutions;
    currentIkSolutionIdx = 0;
    if (solutions.length > 0) {

      if (solutions.length <= 1) {
          document.getElementById("changeConfigurationButton").innerHTML = "Cannot change configuration";
          document.getElementById("changeConfigurationButton").disabled  = true;
      }
      else {
          document.getElementById("changeConfigurationButton").disabled = false;
          document.getElementById("changeConfigurationButton").innerHTML = "Change configuration(" + (currentIkSolutionIdx +1) + "/" + solutionsIK.length + ")";
      }
    }
    else {
       document.getElementById("changeConfigurationButton").innerHTML = "Cannot change configuration";
       document.getElementById("changeConfigurationButton").disabled = true;
       currentIkSolutionIdx = -1;
    }
  }

  var changeConfiguration = function() {
    currentIkSolutionIdx = (currentIkSolutionIdx + 1) % solutionsIK.length;
    document.getElementById("changeConfigurationButton").innerHTML = "Change configuration(" + (currentIkSolutionIdx + 1) + "/" + solutionsIK.length + ")";
    jointInputTopic.publish(solutionsIK[currentIkSolutionIdx].joint_state);      
  }

  // call back when a cartesic sliders/input changes, 
  var callbackCartesicInput = function(event) {
    var name = event.target.name;
    var target;
    if( name.indexOf('_text') >= 0) {
        target = name.replace('_text', '_slider');
    }else{
        target = name.replace('_slider', '_text');
    }

    // set the same value to sibbling widget (slider-text input)
    if (name.startsWith("orientation"))
      document.getElementById(target).value = Utils.angleRound(parseFloat(event.target.value)); 
    else
      document.getElementById(target).value = Utils.distanceRound(parseFloat(event.target.value)); 

    var eulerWorld = {  x:Utils.view2Rad(orientation_x_slider.value), 
                        y:Utils.view2Rad(orientation_y_slider.value), 
                        z:Utils.view2Rad(orientation_z_slider.value)};
    var quat = Utils.euler2Quaternion({ x:eulerWorld.x, y:eulerWorld.y, z:eulerWorld.z })

    var tcpPose = getCurrentPose();

    // call IK
    kinematicsUtils.computeAllIK (getCurrentJointState(),tcpPose.pose, function(solutions) {
      setIKSolutions(solutions);
      if (solutions.length > 0)
        jointInputTopic.publish(solutions[currentIkSolutionIdx].joint_state);      
    }, function(err) {
      displayErr("no IK solution found (" + err + ")");
    }
    )

    // publish the new pose and trigger inverse kinematics
    //Utils.callThrottler("publishTCP", Constants.Kinematics.MAX_KINEMATICS_RATE, function(params) {
    //    tcpInputTopic.publish(params);              
    //}, tcpPose);

    // block tcp input from kinematics while we turn the sliders
    // Utils.stopMutex("blockTcp");
    // Utils.callDelay("blockTcp", Constants.Kinematics.BLOCK_UI_INPUT_TIME, function() {
    //  Utils.releaseMutex("blockTcp", setTcpView );
    //});
  }

  var jointInputTopic  = new ROSLIB.Topic({
    ros : ros,
    name : jointInputTopicName,
    messageType : 'sensor_msgs/JointState'
  });


  var tcpInputTopic  = new ROSLIB.Topic({
    ros : ros,
    name : tcpInputTopicName,
    messageType : 'geometry_msgs/PoseStamped'
  });


  // refresh joints and cartesic coordinates from joint state
  var refresh = function() {
    var listener  = new ROSLIB.Topic({
      ros : ros,
      name : readTopicName,
      messageType : 'sensor_msgs/JointState'
    });

    listener.subscribe(function(msgJointState) {
      jointInputTopic.publish(msgJointState)
      listener.unsubscribe();
    });
  };

  function displayAlert(text, headlinewidget, widget) {
    widget.style.display = 'block';
    widget.innerHTML = text;
    headlinewidget.style.display = 'none';

    setTimeout(function() {
      widget.style.display = 'none';
      headlinewidget.style.display = 'block';
    }, text.length * 50);
  }
  
  function displayInfo(t) {
    displayAlert(t,document.getElementById('kinematicsAlertHeadline'),document.getElementById('kinematicsAlertSuccess'));
  }
  function displayErr(t) {
    displayAlert(t,document.getElementById('kinematicsAlertHeadline'),document.getElementById('kinematicsAlertError'));
  }
  function displayWarn(t) {
    displayAlert(t,document.getElementById('kinematicsAlertHeadline'),document.getElementById('kinematicsAlertWarning'));
  }


  // reset the pose to the factory setting
  function setZeroPosition() {
    var names = [];
    var values = [];
    for(var index = 0; index < sliders.length; index++) {
      var slider = sliders[index];
      var joint_name = slider.name;
      joint_name = joint_name.substring(0,joint_name.indexOf('_slider'));
      names[ names.length ] = joint_name;
      values[ values.length ] = Utils.view2Rad(parseFloat(0));
    }

    // post new joint state to compute forward kinematics
    var newJointState = new ROSLIB.Message({
      name: names, position: values
    });
    jointInputTopic.publish(newJointState);
  }

  function getCurrentPose() {
    var eulerWorld = {  x:Utils.view2Rad(orientation_x_slider.value), 
                        y:Utils.view2Rad(orientation_y_slider.value), 
                        z:Utils.view2Rad(orientation_z_slider.value)};
    var quat = Utils.euler2Quaternion({ x:eulerWorld.x, y:eulerWorld.y, z:eulerWorld.z })

    var pose = new ROSLIB.Message( {
      header: {
        seq: 0,
        stamp: 0,
        frame_id: 'base_link'
      },
      pose : {
        position: { 
          x: Utils.view2Distance(coord_x_slider.value), 
          y: Utils.view2Distance(coord_y_slider.value), 
          z: Utils.view2Distance(coord_z_slider.value) },
        orientation: { 
          x: quat.x, 
          y: quat.y, 
          z: quat.z, 
          w: quat.w
        }
      }     
    });  
    return pose;
  }

  function setPose(pose) {
    tcpInputTopic.publish(pose);
  }

  function setJointState(jointState) {
    jointInputTopic.publish(jointState);
  }

  // call init in contructor
  param.get(loadJointModel);

  // exposed inner functions
  return {
        switchAngleUnit: switchAngleUnit,
        refresh: refresh,
        setZeroPosition: setZeroPosition,
        getCurrentPose: getCurrentPose,
        getCurrentJointState: getCurrentJointState,
        setPose: setPose,
        setJointState: setJointState,
        changeConfiguration: changeConfiguration
  };
};
