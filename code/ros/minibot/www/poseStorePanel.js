/**
 * @author Jochen Alt
 */
var PoseStorePanel = PoseStorePanel || {};


/** 
 * @constructor
 * @param options 
 *   * 
 */

PoseStorePanel.Init = function(options) {
  options = options || {};
  var ros = options.ros;

  // interface to other panels
  var kinematicsPanel = options.kinematicsPanel;
  var kinematicsUtils = options.kinematics;
  var programmePanel = null;

  // each item has an id which is the positon in the list
  // and a uid that stays constant
  var poseListWidget; // UL group
  var poseItems = []; // array of items {uid, name, pose, jointState, liwidget}


  var setProgrammePanel = function(panel) {
    programmePanel = panel;
  }

  // initialize reference data, currently only zero position
  var initialize = function() {
    poseListWidget = document.getElementById("poseStoreList");

    // read the poses from the database
    var request = new ROSLIB.ServiceRequest({
       type: Constants.Database.READ_POSES
    });
   var databaseAction = new ROSLIB.Service({
        ros : ros,
        name : '/database',
        serviceType : 'minibot/Database'
    });

    databaseAction.callService(request, 
      function(result) {
        if (result.error_code.val == ErrorCode.MOVEIT.SUCCESS) {
          poseItems = [];
          for (var idx = 0;idx < result.pose_store.states.length;idx ++) {
            // create a pose with the DOM entries, but set all the rest by the database minibot state
            var poseItem = createPoseItem();
            poseItem.minibot_state = result.pose_store.states[idx];
          }

          // update DOM 
          updateWidgets();

          if (idx == 0) {
            // if the stored list is empty, create the zero position
            var zeroPose = kinematicsUtils.getZeroPose(
              function(pose) {
                createPoseElement('zero position', pose, kinematicsUtils.getZeroJointState());
              },
              function(error) {
                displayErr("creating zero positon failed " + result.error_code.val);
              }
            );
          }
        }
        else {
          displayErr("reading poses from database failed "+ result.error_code.val);
        }
      }, 
      function (result) {
          displayErr("reading poses from database failed " + result);
    });
  };

  var storeInDatabase = function() {

    var minibotStates = [];
    for (var idx = 0;idx < poseItems.length;idx++) {
      minibotStates.push(poseItems[idx].minibot_state);
    }

    // read the poses from the database
    var request = new ROSLIB.ServiceRequest({
      type: Constants.Database.WRITE_POSES,
      pose_store: { 
        states: minibotStates
      }
    });
    var databaseAction = new ROSLIB.Service({
        ros : ros,
        name : '/database',
        serviceType : 'minibot/Database'
    });

    databaseAction.callService(request, 
      function(result) {
        if (result.error_code.val == ErrorCode.MOVEIT.SUCCESS) {
             displayInfo("saved");
        }
        else {
          displayErr("storing in database failed "+ result.error_code.val);
        }
      }, 
      function (result) {
        displayErr("storing in database failed " + result.toString());
    });
  }


  var updateWidgets = function() {
    for (var idx = 0; idx < getPoseItemLength(); idx++) {
      var poseItem = poseItems[idx];
      var id = idx;
      poseItem.widget.childNodes[0].innerHTML = (id + 1).toString() + '<br/>' + poseItem.uid.toString();
      poseItem.widget.childNodes[1].textContent = poseItem.minibot_state.name;
      poseItem.widget.childNodes[2].innerHTML = getPoseString(poseItem.minibot_state);

     // set an id for proper identification of the widget in callbacks
      poseItem.widget.id = idx;
      poseItem.widget.childNodes[0].id = idx;
      poseItem.widget.childNodes[1].id = idx;
      poseItem.widget.childNodes[2].id = idx;
    }

  }

  var getPoseItemIDByUID = function(uid) {
    for (var idx = 0; idx < getPoseItemLength(); idx++) {
      if (poseItems[idx].uid == uid)
        return idx;
    }
    return -1;
  }

  // creates a MinibotPose and the according DOM element that looks like this:
  // <li class="list-group-item list-group-item-action active" id = "0">
  //    <span class="badge badge-primary float-left mr-2">1</span>    
  //    text
  //    <a href='#' class="badge badge-info badge-pill float-right">14/232/182</a>
  // </li>

  var createPoseItem = function() {
    var poseItem = new Object();

    // compute a new UID
    var max = poseItems.length;
    if (max == 0)
      max = 1; // start with 1 to distinguish an unset uid from a valid one 
    for (var idx = 0; idx < poseItems.length; idx++) {
      var currUID = poseItems[idx].uid;
      if (currUID >= max)
        max = currUID + 1;
    }

    poseItem.minibot_state = null;
    poseItem.uid = max;

    var li = document.createElement('LI');
    li.setAttribute('class', 'list-group-item py-1 list-group-item-action justify-content-center align-self-center p-1');
    li.ondblclick = renamePoseCallback;

    var leftSpan = document.createElement("SPAN");
    leftSpan.setAttribute('class', 'badge badge-light float-left mr-1 ml-0 justify-content-center align-self-center');

    var rightSpan = document.createElement("A");
    rightSpan.setAttribute('href', '#');
    rightSpan.setAttribute('class', 'badge badge-success badge-pill mt-1 mr-0 float-right justify-content-center align-self-center');
    rightSpan.onclick = activatePose;
    rightSpan.innerHTML = '';
    var text = document.createTextNode('');

    li.appendChild(leftSpan);
    li.appendChild(text);
    li.appendChild(rightSpan);
    poseListWidget.appendChild(li);
    poseItem.widget = li;
    li.onclick = callbackClick;

    // add the item to the end of the list
    poseItems[poseItems.length] = poseItem;

    return poseItem;
  }

  var updatePoseItem = function(uid, minibotState) {
    var id = getPoseItemIDByUID(uid);
    if (id >= 0) {
      var poseItem = poseItems[id];
      poseItem.minibot_state = minibotState;

      // update the dom accordingly 
      updateWidgets();

      // save immediatly 
      storeInDatabase()

      return poseItems[id];
    }
    return null;
  }

  var movePoseItem = function(uid, newId) {
    var id = getPoseItemIDByUID(uid);
    if (id >= 0) {
      var savePoseItem = poseItems[newId];
      poseItems[newId] = poseItems[id];
      poseItems[id] = savePoseItem;
    }
  }

  var getPoseItem = function(idx) {
    return poseItems[idx];
  }

  var getPoseItemLength = function(idx) {
    return poseItems.length;
  }


  // return a short string out of a pose that is used in the badges 
  var getPoseString = function(minibotState) {
    var s = Math.round(minibotState.pose.pose.position.x * 1000).toString() + '/' + 
            Math.round(minibotState.pose.pose.position.y * 1000).toString() + '/' + 
            Math.round(minibotState.pose.pose.position.z * 1000).toString();
    return s;
  }


  var createPoseElement = function(minibotState) {
    // create and update the new pose
    var newPoseItem = createPoseItem();
    updatePoseItem(newPoseItem.uid, minibotState);
    
    // update DOM
    updateWidgets();

    return newPoseItem;
  }

  var activatePose = function(event) {
    var id = parseInt(event.target.parentNode.getAttribute('id'));
    var poseItem = getPoseItem(id);
    kinematicsPanel.setCurrentMinibotState(poseItem.minibot_state);
  }

  // called from a double click in a pose
  var renamePoseCallback = function(event) {
    var id = parseInt(event.target.parentNode.getAttribute('id'));
    activate(id);

    renamePose();
  }

  var callbackKeyDown = function(event) {
    var li = event.target.parentNode;
    var id = parseInt(li.getAttribute('id'));
    if (event.key == 'Escape') {
      cancelEditMode();
    }
    if (event.key == 'Enter') {
      var poseItem = getPoseItem(id);
      poseItem.minibot_state.name = event.target.value
      cancelEditMode();
      updateWidgets();

      storeInDatabase();

    // the name is mentioned in the programme panel, so update that as well
    if (programmePanel != null)
      programmePanel.refresh();
    }
  }

  // in case an item is in edit mode, turn it off
  var callbackFocusOut = function(event) {
    cancelEditMode();
  }

  var callbackClick = function(event) {
    var id = event.target.id;
    activate(id);
  }

  var activateByUID = function(uid) {
    var id = getPoseItemIDByUID(uid);
    activate(id);
  }

  // activate the passed list item
  var activate = function(id) {
    if (id >= 0) {
      for (var idx = 0; idx < getPoseItemLength(); idx++) {
        if (idx == id)
          poseItems[idx].widget.classList.add('active');
        else
          poseItems[idx].widget.classList.remove('active');
      }
    }
  }

  // return id of active list item
  var getActiveId = function(event) {
    for (var idx = 0; idx < getPoseItemLength(); idx++) {
      if (poseItems[idx].widget.getAttribute('class').includes('active'))
        return idx;
    }
    return null;
  }

  // in case a list item is in edit mode, turn back to normal
  var cancelEditMode = function() {
    inputWidget = document.getElementById('poseStorePanelInputField');
    if (inputWidget != null) {
      var id = inputWidget.parentNode.id;
      var poseItem = getPoseItem(id);
      var li = getPoseItem(id).widget;
      var text = document.createTextNode(poseItem.minibot_state.name);
      li.removeChild(inputWidget);
      li.insertBefore(text, li.childNodes[1]); // insert between the two spans
    }
  }

  // rename the currently selected pose
  function renamePose() {
    var id = getActiveId();
    if (id != null) {
      var poseItem = getPoseItem(id);
      var text = poseItem.widget.childNodes[1];

      // if editmode is one, stop it
      cancelEditMode();

      // change the label to an text input to allow changing the name
      var inputWidget = document.createElement('input');
      inputWidget.id = 'poseStorePanelInputField'
      inputWidget.onkeydown = callbackKeyDown;
      inputWidget.onfocusout = callbackFocusOut;
      inputWidget.value = poseItem.minibot_state.name;
      inputWidget.type = 'text';
      // inputWidget.setAttribute('class', 'form-control');
      poseItem.widget.insertBefore(inputWidget, text);
      poseItem.widget.removeChild(text);
      inputWidget.focus();
    } else
      displayErr('select a pose first')
  }

  function deletePose() {
    var id = getActiveId();
    if (id != null) {
      var li = getPoseItem(id).widget;
      // check if this pose is not used in a programme
      var statementID = programmePanel.getStatementIDByPoseUID(poseItems[id].uid);
      if (statementID != null && statementID>=0) {
        programmePanel.activateStatement(statementID);
        displayErr("pose is used, delete the statement first");
        return;
      }


      // if editmode is on, stop it
      cancelEditMode();

      // delete dom node 
      poseListWidget.removeChild(li);
      for (var idx = id; idx < poseItems.length - 1; idx++)
        poseItems[idx] = poseItems[idx + 1];
      delete poseItems.pop();

      // deleting an item changes the IDs
      updateWidgets();

      // save immediatly 
      storeInDatabase()

      // activate the next item 
      if (id < poseItems.length)
        activate(id);
      else
      if (id > 0)
        activate(id - 1);
    } else
      displayErr('select a pose first')
  }

  function newPose() {
    var minibotState = kinematicsPanel.getCurrentMinibotState();

    // if editmode is on, stop it
    cancelEditMode();

    // create new element without a name yet
    var poseItem = createPoseElement(minibotState);
    var id = getPoseItemIDByUID(poseItem.uid);
    // scroll to new element and activate it
    poseItem.widget.scrollIntoView();
    activate(id);

    // store in database
    storeInDatabase();
  }

  function storePose() {
    var id = getActiveId();

    if (id == null) {
      newPose();
      displayWarn('no pose selected, new pose created');
    } else {
      var poseItem = getPoseItem(id);
      poseItem.minibot_state = kinematicsPanel.getCurrentMinibotState();

      // we need to change the badges, store everyhting and  inform the proramme panel 
      updateWidgets();
      storeInDatabase();
      programmePanel.modifyPose();
    }
  }

  function up() {
    var id = getActiveId();

    if (id > 0) {
      var uid = poseItems[id].uid;
      var poseItem = poseItems[id];
      var prevPoseItem = poseItems[id - 1];

      poseListWidget.removeChild(poseItem.widget);
      movePoseItem(uid, id - 1);

      poseListWidget.insertBefore(poseItem.widget, prevPoseItem.widget);
      updateWidgets();
      storeInDatabase();

    } else
      displayErr('selected pose is first already')
  }

  function down() {
    var id = getActiveId();

    if (id != null && id < poseItems.length - 1) {
      var uid = poseItems[id].uid;
      var poseItem = poseItems[id];
      poseListWidget.removeChild(poseItem.widget);
      movePoseItem(uid, id + 1);
      if (id == poseItems.length - 2)
        poseListWidget.append(poseItem.widget);
      else
        poseListWidget.insertBefore(poseItem.widget, poseItems[id + 2].widget);

      updateWidgets();
      storeInDatabase();

    } else
      displayErr('selected pose is last already')
  }

  // return the UID of the active pose
  function getCurrentPoseUID() {
    var id = getActiveId();
    if (id != null) {
      return poseItems[id].uid;
    }
    return null;
  }

  // set the current kinematics to the pose passed identifid by the passed uid
  function setPoseByUID(uid) {
    var id =  getPoseItemIDByUID(uid);
    if (id != null && id >= 0 && id<poseItems.length) {      
      kinematicsPanel.setCurrentMinibotState(poseItems[id].minibot_state);
    }
    else
      return null;
  }

  function getMinibotStateByUID(uid) {
    var id = getPoseItemIDByUID(uid);
    if (id != null && id >= 0 && id<poseItems.length)
      return poseItems[id].minibot_state;
    else
      return null;
  }


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
    displayAlert(t, document.getElementById('poseStorageAlertHeadline'), document.getElementById('poseStorageAlertSuccess'));
  }

  function displayErr(t) {
    displayAlert(t, document.getElementById('poseStorageAlertHeadline'), document.getElementById('poseStorageAlertError'));
  }

  function displayWarn(t) {
    displayAlert(t, document.getElementById('poseStorageAlertHeadline'), document.getElementById('poseStorageAlertWarning'));
  }

  // call init in contructor
  // initialize();

  // exposed inner functions
  return {
    deletePose: deletePose,
    storePose: storePose,
    newPose: newPose,
    renamePose: renamePose,

    up: up,
    down: down,

    getCurrentPoseUID: getCurrentPoseUID,
    setPoseByUID: setPoseByUID,
    activateByUID : activateByUID, 
    initialize : initialize,

    getMinibotStateByUID: getMinibotStateByUID,
    setProgrammePanel: setProgrammePanel    // the programme panel is using poses and needs to be updated from time to time

  };
};