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
  var kinematics = options.kinematics;
  var programmePanel = null;

  var listWidgetName = options.listName;

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
    var zeroPose = kinematics.getZeroPose(
      function(pose) {
        createPoseElement('zero position', pose, kinematics.getZeroJointState());
      },
      function(error) {
        displayErr('kinematics failure when getting zero position ${error}');
      });

  };


  var updateWidgets = function() {
    for (var idx = 0; idx < getPoseItemLength(); idx++) {
      var poseItem = poseItems[idx];
      var id = idx;
      poseItem.widget.childNodes[0].innerHTML = (id + 1).toString() + '<br/>' + poseItem.uid.toString();
      poseItem.widget.childNodes[1].textContent = poseItem.name;
      poseItem.widget.childNodes[2].innerHTML = getPoseString(poseItem.pose, poseItem.jointState);

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

  // create an element that looks like this:
  // <li class="list-group-item list-group-item-action active" id = "0">
  //    <span class="badge badge-primary float-left mr-2">1</span>    
  //    text
  //    <a href='#' class="badge badge-info badge-pill float-right">14/232/182</a>
  // </li>

  var createPoseItem = function() {
    var poseItem = new Object();

    // compute a new UID
    var max = poseItems.length;
    for (var idx = 0; idx < poseItems.length; idx++) {
      if (poseItems[idx].uid >= max)
        max = poseItems[idx] + 1;
    }

    poseItem.uid = max;
    poseItem.jointStates = null;
    poseItem.name = null;
    poseItem.pose = null;

    var li = document.createElement('LI');
    li.setAttribute('class', 'list-group-item py-1 list-group-item-action justify-content-center align-self-center');

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

  var updatePoseItem = function(uid, name, pose, jointState) {
    var id = getPoseItemIDByUID(uid);
    if (id >= 0) {
      poseItems[id].name = name;
      poseItems[id].pose = pose;
      poseItems[id].jointState = jointState;

      // update the dom accordingly 
      updateWidgets();

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
  var getPoseString = function(pose, jointState) {
    var s = Math.round(pose.pose.position.x * 1000).toString() + '/' + Math.round(pose.pose.position.y * 1000).toString() + '/' + Math.round(pose.pose.position.z * 1000).toString();
    /* s += '<br/>';
    for (var idx = 0;idx <6;idx++) {
      if (idx > 0)
        s += '/';
      s += Math.round(jointState.position[idx]*180/Math.PI).toString();
    }
    */
    return s;
  }


  var createPoseElement = function(name, pose, jointState) {
    var newPoseItem = createPoseItem();
    updatePoseItem(newPoseItem.uid, name, pose, jointState);
    updateWidgets();
    return newPoseItem;
  }

  var activatePose = function(event) {
    var li = event.target.parentNode;
    var id = parseInt(li.getAttribute('id'));
    var poseItem = getPoseItem(id);
    var jointState = poseItem.jointState;
    if (jointState != null)
      kinematicsPanel.setJointState(jointState);
    else {
      var pose = poseItem.pose;
      kinematicsPanel.setPose(pose);
    }

  }

  var callbackKeyDown = function(event) {
    var li = event.target.parentNode;
    var id = parseInt(li.getAttribute('id'));
    if (event.key == 'Escape') {
      cancelEditMode();
    }
    if (event.key == 'Enter') {
      var poseItem = getPoseItem(id);
      poseItem.name = event.target.value
      cancelEditMode();
      updateWidgets;

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
      var text = document.createTextNode(poseItem.name);
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
      inputWidget.value = poseItem.name;
      inputWidget.type = 'text';
      inputWidget.setAttribute('class', 'form-control');
      poseItem.widget.appendChild(inputWidget);
      poseItem.widget.removeChild(text);
      inputWidget.focus();


    } else
      displayErr('select a pose first')
  }

  function deletePose() {
    var id = getActiveId();
    if (id != null) {
      var li = getPoseItem(id).widget;

      // if editmode is on, stop it
      cancelEditMode();

      // delete dom node 
      poseListWidget.removeChild(li);
      for (var idx = id; idx < poseItems.length - 1; idx++)
        poseItems[idx] = poseItems[idx + 1];
      delete poseItems.pop();

      // deleting an item changes the IDs
      updateWidgets();

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
    var pose = kinematicsPanel.getCurrentPose();
    var jointState = kinematicsPanel.getCurrentJointState();

    // if editmode is on, stop it
    cancelEditMode();

    // create new element without a name yet
    var poseItem = createPoseElement('', pose, jointState);
    var id = getPoseItemIDByUID(poseItem.uid);
    // scroll to new element and activate it
    poseItem.widget.scrollIntoView();
    activate(id);
  }

  function storePose() {
    var id = getActiveId();

    if (id == null) {
      newPose();
      displayWarn('no pose selected, new pose created');
    } else {
      var pose = kinematicsPanel.getCurrentPose();
      var jointState = kinematicsPanel.getCurrentJointState();
      var poseItem = getPoseItem(id);
      poseItem.pose = pose;
      poseItem.jointState = jointState;

      // we need to change the badges
      updateWidgets();
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

  function getPoseByUID(uid) {
    var id = getPoseItemIDByUID(uid);
    if (id != null)
      return poseItems[id].pose;
    else
      return null;
  }

  function getNameByUID(uid) {
    var id = getPoseItemIDByUID(uid);
    if (id != null)
      return poseItems[id].name;
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
  initialize();

  // exposed inner functions
  return {
    deletePose: deletePose,
    storePose: storePose,
    newPose: newPose,
    up: up,
    down: down,
    renamePose: renamePose,
    getCurrentPoseUID: getCurrentPoseUID,
    getPoseByUID: getPoseByUID,
    getNameByUID: getNameByUID,
    setProgrammePanel: setProgrammePanel
  };
};