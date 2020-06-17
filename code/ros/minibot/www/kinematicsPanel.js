/**
 * @author Jochen Alt
 */

var KinematicsPanel = KinematicsPanel || {
};

KinematicsPanel.Init = function(options) {
  options = options || {};
  var ros = options.ros;
  var kinematicsUtils = options.kinematics;
  var paramName                     = 'robot_description';
  var readTopicName                 = '/joint_states';                              // 20Hz topic that gives the current state of the joints, publish constantly
  var receiveJointStateTopicName    = '/joint_states/update'                        // joint state are received here
  var jointInputTopicName           = '/joint_states/input/update'                  // UI joint state changes are published here

  var receiveTcpTopicName           = '/pose/update'                                // receive changes in tcp not coming from UI
  var tcpInputTopicName             = '/pose/input/update'                          // UI tcp changes are published here 
  var receiveConfigurationTopicName = '/joint_states/configuration'                 // listen to all confgurations

  var pubConfigurationTopicName     = '/joint_configuration/input/update'          // listen to all confgurations

  var msgTopicName                  = '/msg'                                        // listen to messages from server

  var sliders = [];
  var inputs = [];
  var coordInputs = [];
  var coordSlider = [];
  var toolDistanceSlider = null;
  var toolDistanceInput = null;
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
                  textInput.onkeyup  = callBackJointInputKeyUp;
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

        toolDistanceSlider = document.getElementById("tool_distance_slider");
        toolDistanceInput = document.getElementById("tool_distance_text");

        for (var index = 0;index < 3; index++) {
          coordInputs[index].setAttribute('name', coordInputs[index].id);
          coordInputs[index].setAttribute('value', 0);
          coordInputs[index].setAttribute('step', Utils.distanceSteps());
          coordInputs[index].onblur   = callbackCartesicInput;
          coordInputs[index].onchange = callbackCartesicInput;
          coordInputs[index].onkeyup  = callbackCartesicInputKeyUp;

          coordSliders[index].setAttribute('name', coordSliders[index].id);
          coordSliders[index].setAttribute('value', 0);
          coordSliders[index].setAttribute('step', Utils.distanceSteps());
          coordSliders[index].oninput  = callbackCartesicInput;

          orientationInputs[index].setAttribute('name', orientationInputs[index].id);
          orientationInputs[index].setAttribute('value', 0);
          orientationInputs[index].setAttribute('step', Utils.angleSteps());
          orientationInputs[index].onblur   = callbackCartesicInput;
          orientationInputs[index].onchange = callbackCartesicInput;
          orientationInputs[index].onkeyup  = callbackCartesicInputKeyUp;

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

        toolDistanceSlider.setAttribute('name', toolDistanceSlider.id);
        toolDistanceSlider.setAttribute('value', 0);
        toolDistanceSlider.setAttribute('step',Utils.distanceSteps());
        toolDistanceSlider.setAttribute('min',0,);
        toolDistanceSlider.setAttribute('max',150);
        toolDistanceSlider.oninput = callbackToolDistanceInput;

        toolDistanceInput.setAttribute('name', toolDistanceInput.id);
        toolDistanceInput.setAttribute('value', 0);
        toolDistanceInput.setAttribute('min',0,);
        toolDistanceInput.setAttribute('max',150);
        toolDistanceInput.setAttribute('step',Utils.distanceSteps());
        toolDistanceInput.onblur   = callbackToolDistanceInput;
        toolDistanceInput.onchange = callbackToolDistanceInput;
        toolDistanceInput.onkeyup  = callbackToolDistanceInputKeyUp;
    }  

    var listenToTCP  = new ROSLIB.Topic({
      ros : ros,
      name : receiveTcpTopicName,
      messageType : 'minibot/MinibotPose'
    });

    listenToTCP.subscribe(function(minibotPose) {
        // Utils.queueAtMutex("blockTcp",setTcpView, tcpPose);
        setTcpView(minibotPose.pose, minibotPose.tool_length);
    });

    var listenToJointState  = new ROSLIB.Topic({
      ros : ros,
      name : receiveJointStateTopicName,
      messageType : 'sensor_msgs/JointState'
    });

    listenToJointState.subscribe(function(jointState) {
        // Utils.queueAtMutex("blockJoint",setJointView, jointState);
        setJointView (jointState);
    });

    var listenToConfigurations = new ROSLIB.Topic({
      ros : ros,
      name : receiveConfigurationTopicName,
      messageType : 'minibot/JointStateConfiguration'
    })

    listenToConfigurations.subscribe(function(jointStateConfiguration) {
      setIKSolutions (jointStateConfiguration.configuration);
    })

    refresh()
  };

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

  var setTcpView = function (tcpPose, toolDistance) {

      // convert the world frame into view frame 
      var pos = [ tcpPose.position.x, tcpPose.position.y, tcpPose.position.z ];
      coordInputs[0].value = Utils.distance2View(pos[0]);
      coordInputs[1].value = Utils.distance2View(pos[1]);
      coordInputs[2].value = Utils.distance2View(pos[2]);
      coordSliders[0].value = coordInputs[0].value;
      coordSliders[1].value = coordInputs[1].value;
      coordSliders[2].value = coordInputs[2].value;

      // dont forget the tool distance slider
      toolDistanceSlider.value = Utils.distance2View(toolDistance);
      toolDistanceInput.value = Utils.distance2View(toolDistance);
      
      // convert the world frame into view frame 
      var orientWorld = Utils.quaternion2Euler(tcpPose.orientation);
      orientationInputs[0].value = Utils.rad2View(orientWorld.x);
      orientationInputs[1].value = Utils.rad2View(orientWorld.y);
      orientationInputs[2].value = Utils.rad2View(orientWorld.z);
      orientationSliders[0].value = orientationInputs[0].value;
      orientationSliders[1].value = orientationInputs[1].value;
      orientationSliders[2].value = orientationInputs[2].value;
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

  var callBackJointInputKeyUp = function(event) {
    var name = event.target.name;
    if (event.key == 'Enter') {
      callBackJointInput(event);
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

    // retrieve current state
    var minibotState = getCurrentMinibotState();

    // and publish joint state, but throttled  
    Utils.callThrottler("newJoint", Constants.Kinematics.MAX_KINEMATICS_RATE, function(params) {
      pubMinibotStateJoints.publish(params);
    },minibotState);

    // block joint input from kinematics while we turn the sliders (and 0.5 seconds afterwards)
    // Utils.stopMutex("blockJoint");
    // Utils.callDelay("blockJoint", Constants.Kinematics.BLOCK_UI_INPUT_TIME, function() {
    //  Utils.releaseMutex("blockJoint", setJointView );
    //});
  }

  var pubMinibotStateJoints  = new ROSLIB.Topic({
      ros : ros,
      name : jointInputTopicName,
      messageType : 'minibot/MinibotState'
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

  // take the solutions coming from IK, store thim globally in solutionsIK
  // and change the change-configuration-button accordingly 
  var setIKSolutions = function (solutions) {
    solutionsIK = solutions;
    currentIkSolutionIdx = 0;
    var button = document.getElementById("changeConfigurationButton");
    if (solutions.length > 0) {

      if (solutions.length <= 1) {
          button.innerHTML = "Cannot change configuration";
          button.disabled  = true;
      }
      else {
          button.disabled = false;
          button.innerHTML = "Change configuration (" + (currentIkSolutionIdx +1) + "/" + solutionsIK.length + ")";
      }
    }
    else {
       button.innerHTML = "One configuration only";
       button.disabled = true;
       currentIkSolutionIdx = -1;
    }
  }

  var changeConfiguration = function() {
    currentIkSolutionIdx = (currentIkSolutionIdx + 1) % solutionsIK.length;
    document.getElementById("changeConfigurationButton").innerHTML = "Change configuration (" + (currentIkSolutionIdx + 1) + "/" + solutionsIK.length + ")";

    // retrieve current joint
    var minibotState = getCurrentMinibotState();
    minibotState.joint_state = solutionsIK[currentIkSolutionIdx];

    // and publish joint state, but throttled  
    var pubConfigurationTopic = new ROSLIB.Topic({
      ros : ros,
      name : pubConfigurationTopicName,
      messageType : 'minibot/MinibotState'
    });

    Utils.callThrottler("newJoint", Constants.Kinematics.MAX_KINEMATICS_RATE, function(params) {
      pubConfigurationTopic.publish(params);
    },minibotState);


/*
    var publisherJointState  = new ROSLIB.Topic({
      ros : ros,
      name : receiveJointStateTopicName,
      messageType : 'sensor_msgs/JointState'
    });

    publisherJointState.publish(solutionsIK[currentIkSolutionIdx]);
    */
  }

  var callbackCartesicInputKeyUp = function(event) {
    var name = event.target.name;
    if (event.key == 'Enter') {
      callbackCartesicInput(event);
    }
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

    var minibotState = getCurrentMinibotState();

    Utils.callThrottler("publishTCP", Constants.Kinematics.MAX_KINEMATICS_RATE, function(params) {
        tcpInputTopic.publish(params);              
    }, minibotState);
  }

  var callbackToolDistanceInputKeyUp = function(event) {
    var name = event.target.name;
    if (event.key == 'Enter') {
      callbackToolDistanceInput(event);
    }
  }

  // call back when a cartesic sliders/input changes, 
  var callbackToolDistanceInput = function(event) {
    var name = event.target.name;
    var target;
    if( name.indexOf('_text') >= 0) {
        target = name.replace('_text', '_slider');
    }else{
        target = name.replace('_slider', '_text');
    }

    // set the same value to sibbling widget (slider-text input)
    document.getElementById(target).value = Utils.distanceRound(parseFloat(event.target.value)); 

    var minibotState = getCurrentMinibotState();

    Utils.callThrottler("publishToolDistance", Constants.Kinematics.MAX_KINEMATICS_RATE, function(params) {
        tcpInputTopic.publish(params);              
    }, minibotState);
  }


  var tcpInputTopic  = new ROSLIB.Topic({
    ros : ros,
    name : tcpInputTopicName,
    messageType : 'minibot/MinibotState'
  });

  var msgTopic  = new ROSLIB.Topic({
    ros : ros,
    name : msgTopicName,
    messageType : 'std_msgs/String'
  });

  msgTopic.subscribe(function(msg) {
    if (msg.data.startsWith('KINEMATICS:ERR:'))
      displayErr(msg.data.substring("KINEMATICS:ERR:".length))
    if (msg.data.startsWith('KINEMATICS:WARN:'))
      displayWarn(msg.data.substring("KINEMATICS:WARN:".length))
    if (msg.data.startsWith('KINEMATICS:INFO:'))
      displayInfo(msg.data.substring("KINEMATICS:INFO:".length))
  });

  // refresh joints and cartesic coordinates from joint state
  var refresh = function() {
    var listener  = new ROSLIB.Topic({
      ros : ros,
      name : readTopicName,
      messageType : 'sensor_msgs/JointState'
    });

    listener.subscribe(function(msgJointState) {
      setJointView(msgJointState);
      var minibotState = getCurrentMinibotState();
      pubMinibotStateJoints.publish(minibotState)
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

    var minibotState = getCurrentMinibotState();
    minibotState.joint_state.name = names;
    minibotState.joint_state.position = values;
    minibotState.pose.tool_length = 0;

    // post new joint state to compute forward kinematics
    jointInputTopic.publish(minibotState);
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

  function getCurrentMinibotState() {
    var eulerWorld = {  x:Utils.view2Rad(orientation_x_slider.value), 
                        y:Utils.view2Rad(orientation_y_slider.value), 
                        z:Utils.view2Rad(orientation_z_slider.value)};
    var quat = Utils.euler2Quaternion({ x:eulerWorld.x, y:eulerWorld.y, z:eulerWorld.z })

    var names = [];
    var values = [];
    for(var index = 0; index < sliders.length; index++) {
      var slider = sliders[index];
      var joint_name = slider.name;
      joint_name = joint_name.substring(0,joint_name.indexOf('_slider'));
      names[ names.length ] = joint_name;
      values[ values.length ] = Utils.view2Rad(parseFloat(slider.value));
    }

    var state = new ROSLIB.Message( {
      pose: {
        pose: {
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
        },
        tool_length : parseFloat(Utils.view2Distance(tool_distance_slider.value))
      },  
      joint_state: {
        name: names,
        position: values
      },
      configuration: solutionsIK
    });  
    return state;
  }

  function setCurrentMinibotState(minibotState) {
    solutionsIK = minibotState.configuration;
    setJointView (minibotState.joint_state)
    pubMinibotStateJoints.publish(minibotState);
  }

  // call init in contructor
  param.get(loadJointModel);

  // exposed inner functions
  return {
        switchAngleUnit: switchAngleUnit,
        refresh: refresh,
        setZeroPosition: setZeroPosition,
        getCurrentPose: getCurrentPose,
        getCurrentMinibotState: getCurrentMinibotState,
        setCurrentMinibotState: setCurrentMinibotState,
        changeConfiguration: changeConfiguration
  };
};
