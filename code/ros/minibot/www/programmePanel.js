/**
 * @author Jochen Alt
 */
var ProgrammePanel = ProgrammePanel || {};


/** 
 * @constructor
 * @param options 
 *   * 
 */

ProgrammePanel.Init = function(options) {
  options = options || {};
  var ros = options.ros;

  // interface to other panels
  var kinematicsPanel = options.kinematicsPanel;
  var kinematics = options.kinematics;
  var poseStorePanel = options.poseStorePanel;
  var settingsPanel  = options.settingsPanel;



  // each item has an id which is the positon in the list
  // and a uid that stays constant
  var programmeListWidget; // UL group
  var programmeItems = []; // array of items {uid, name, liwidget, type, waitType, poseuid,  (used for waypoint), comment (used for comment or wait), wait [seconds]}

  // initialize reference data, currently only zero position
  var initialize = function() {
    programmeListWidget = document.getElementById('programmeList');

    // read the poses from the database
    var request = new ROSLIB.ServiceRequest({
      type: Constants.Database.READ_PROGRAMME
    });
    var databaseAction = new ROSLIB.Service({
      ros: ros,
      name: '/database',
      serviceType: 'minibot/Database'
    });

    databaseAction.callService(request,
      function(result) {
        if (result.error_code.val == Constants.ErrorCodes.SUCCESS) {
          programmeItems = [];
          for (var idx = 0; idx < result.programme_store.statements.length; idx++) {
            var statementDB = result.programme_store.statements[idx];
            var stmt = newStatement();
            stmt.statement = result.programme_store.statements[idx];
          }

          // update DOM 
          updateWidgets();

          // and select the first statement if it is there
          if (programmeItems.length > 0) {
            activateStatement(0);
          }
        } else {
          displayErr("reading programme from database failed " + result.error_code.val);
        }
      },
      function(result) {
        displayErr("reading programme from database failed " + result);
      })
  };


  var storeInDatabase = function(force) {
    if (force)
      Utils.callDelay("storedatabase", 0, function() { 
        displayInfo("store and plan")
        rawStoreInDatabase() 
      }); 
    else
      Utils.callDelay("storedatabase", settingsPanel.getSaveAfterSeconds(), function() { 
        displayInfo("store and plan")
        rawStoreInDatabase() 
      }); 

  }

  var rawStoreInDatabase = function() {

    var statements = [];
    for (var idx = 0; idx < programmeItems.length; idx++)
      statements.push(programmeItems[idx].statement);

    // write statements to database
    var request = new ROSLIB.ServiceRequest({
      type: Constants.Database.WRITE_PROGRAMME,
      programme_store: {
        statements: statements
      }
    });
    var databaseAction = new ROSLIB.Service({
      ros: ros,
      name: '/database',
      serviceType: 'minibot/Database'
    });

    databaseAction.callService(request,
      function(result) {
        if (result.error_code.val == Constants.ErrorCodes.SUCCESS) {
          displayInfo("saved");
        } else {
          displayErr("database error " + result.error_code.val);
        }
      },
      function(result) {
        displayErr("database error " + result.toString());
      });
  }


  var updateWidget = function(idx) {
    var prgStmt  = programmeItems[idx];
    var id = idx;

    var idbadge = prgStmt.widget.childNodes[0];
    var text = prgStmt.widget.childNodes[1];
    var badge = prgStmt.widget.childNodes[2];
    idbadge.innerHTML = (id + 1).toString() + '<br/>' + prgStmt.statement.uid.toString();
    text.textContent = prgStmt.statement.name;
    badge.innerHTML = getBadgeString(prgStmt.statement);

    // set an id for proper identification of the widget in callbacks
    prgStmt.widget.id = idx;
    idbadge.id = idx;
    text.id = idx;
    badge.id = idx;

    // set color of right badge
    if (prgStmt.statement.error_code != null && 
        prgStmt.statement.error_code.val == Constants.ErrorCodes.SUCCESS) {
      badge.classList.remove("badge-success");
      badge.classList.add("badge-danger");
    } else {
      badge.classList.remove("badge-danger");
      badge.classList.add("badge-success");
    }

    // remove color of waypoints
    prgStmt.widget.classList.remove('list-group-item-success');
    badge.classList.remove('badge-secondary');
    badge.classList.remove('badge-primary');

    if (prgStmt.statement.type == Constants.Statement.STATEMENT_TYPE_WAIT) {
      prgStmt.widget.classList.add('list-group-item-light');
      badge.classList.add('badge-secondary');

      // even if a different radio button is on, display the other fields
      document.getElementById('waitConfirmationText').value = prgStmt.statement.comment;
      document.getElementById('secondsToWait').value = prgStmt.statement.kitkat.secs;

      document.getElementById('waitForSeconds').checked = false;
      document.getElementById('waitForConfirmation').checked = false;
      document.getElementById('dontWait').checked = false;
      if (prgStmt.statement.wait_type == Constants.Statement.WAIT_TYPE_WAIT)
        document.getElementById('waitForSeconds').checked = true;
      else
      if (prgStmt.statement.wait_type == Constants.Statement.WAIT_TYPE_CONFIRMATION)
        document.getElementById('waitForConfirmation').checked = true;
      else
      if (prgStmt.statement.wait_type == Constants.Statement.WAIT_TYPE_NOWAIT)
        document.getElementById('dontWait').checked = true;
    }

    if (prgStmt.statement.type == Constants.Statement.STATEMENT_TYPE_MOVEMENT) {
      badge.classList.add('badge-primary');
      prgStmt.widget.classList.add('list-group-item-success');
      document.getElementById('collisionCheck').checked = prgStmt.statement.collision_check;

      if (prgStmt.statement.path_strategy == Constants.Planning.PLAN_CARTESIC_STRATEGY)
        document.getElementById('cartesicPath').checked = true;
      if (prgStmt.statement.path_strategy == Constants.Planning.PLAN_SPLINE_STRATEGY)
        document.getElementById('splinePath').checked = true;
      if (prgStmt.statement.path_strategy == Constants.Planning.PLAN_SPACE_STRATEGY)
        document.getElementById('spacePath').checked = true;
      var movementErrorWidget = document.getElementById('movementError');
      if (prgStmt.statement.error_code != null && prgStmt.statement.error_code.val != Constants.ErrorCodes.SUCCESS) {
        movementErrorWidget.style.display = 'block';
        movementErrorWidget.innerHTML = "Planning error " + prgStmt.statement.error_code.val;
      } else {
        document.getElementById('waypointError').style.display = 'none';
        movementErrorWidget.innerHTML = "";
      }
    }
    if (prgStmt.statement.type == Constants.Statement.STATEMENT_TYPE_WAYPOINT) {
      badge.classList.add('badge-primary');
      prgStmt.widget.classList.add('list-group-item-success');
      var waypointErrorWidget = document.getElementById('waypointError');
      if (prgStmt.statement.error_code != null && prgStmt.statement.error_code.val != Constants.ErrorCodes.SUCCESS) {
        waypointErrorWidget.style.display = 'block';
        waypointErrorWidget.innerHTML = "Planning error " + prgStmt.statement.error_code.val;
      } else {
        document.getElementById('waypointError').style.display = 'none';
        waypointErrorWidget.innerHTML = "";
      }
      document.getElementById('blendWaypoint').checked = prgStmt.statement.blend;
    }

    if (prgStmt.statement.type == Constants.Statement.STATEMENT_TYPE_COMMENT) {
      badge.classList.add('badge-secondary');
      document.getElementById('commentStatement').value = prgStmt.statement.comment;
    }
  }


  var updateWidgets = function() {
    for (var idx = 0; idx < getProgrammeLength(); idx++) {
      updateWidget(idx);
      // set an id for proper identification of the widget in callbacks
      programmeItems[idx].widget.id = idx;
      programmeItems[idx].widget.childNodes[0].id = idx;
      programmeItems[idx].widget.childNodes[1].id = idx;
      programmeItems[idx].widget.childNodes[2].id = idx;
    }
    if (getProgrammeLength() == 0) {
        // last element? then hide the detail view
        document.getElementById('detailsMovementStatement').style.display = 'none';
        document.getElementById('detailsWaypointStatement').style.display = 'none';
        document.getElementById('detailsWaitStatement').style.display = 'none';
        document.getElementById('detailsCommentStatement').style.display = 'none';
    }
    document.getElementById('visualizeLocalPlan').checked = settingsPanel.getVisualizationLocalPlan()
    document.getElementById('visualizeGlobalPlan').checked = settingsPanel.getVisualizationGlobalPlan()
  }

  var getStatementIDByUID = function(uid) {
    var result = 0;
    for (var idx = 0; idx < getProgrammeLength(); idx++) {
      if (programmeItems[idx].statement.uid == uid)
        return idx;
    }
    return -1;
  }

  // returns the id of the statement that refers to a given pose ID  
  var getStatementIDByPoseUID = function(uid) {
    var result = 0;
    for (var idx = 0; idx < getProgrammeLength(); idx++) {
      if (programmeItems[idx].statement.pose_uid == uid)
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

  var newStatement = function() {
    var prgItem = new Object();

    // compute a new UID
    var max = programmeItems.length;
    if (max == 0)
      max = 1;
    for (var idx = 0; idx < programmeItems.length; idx++) {
      var currUID = programmeItems[idx].statement.uid;
      if (currUID >= max)
        max = currUID + 1;
    }

    // this methods ends with an empty statement
    prgItem.statement = new Object();
    prgItem.statement.uid = max;
    prgItem.statement.name = null;
    prgItem.statement.type = Constants.Statement.STATEMENT_TYPE_NONE;
    prgItem.statement.error_code = new Object();
    prgItem.statement.error_code.val = Constants.ErrorCodes.SUCCESS;
    prgItem.statement.kitkat = Object();
    prgItem.statement.kitkat.secs = 0;
    prgItem.statement.kitkat.nsecs = 0;

    var li = document.createElement('LI');
    li.setAttribute('class', 'list-group-item py-1 list-group-item-action justify-content-center align-self-center p-1');
    li.onclick = callbackClick;
    li.ondblclick = renameStatementCallback;


    var leftSpan = document.createElement("SPAN");
    leftSpan.setAttribute('class', 'badge badge-light float-left mr-1  justify-content-center align-self-center');

    var rightSpan = document.createElement("A");
    rightSpan.setAttribute('href', '#');
    rightSpan.setAttribute('class', 'badge badge-success badge-pill mt-1 mr-0 float-right justify-content-center align-self-center');
    rightSpan.innerHTML = '';
    rightSpan.onclick = activatePose;
    var text = document.createTextNode('');

    li.appendChild(leftSpan);
    li.appendChild(text);
    li.appendChild(rightSpan);
    programmeListWidget.appendChild(li);
    prgItem.widget = li;

    // add the item to the end of the list
    programmeItems[programmeItems.length] = prgItem;

    return prgItem;
  }


  var updateStatement = function(uid, name, type) {
    var id = getStatementIDByUID(uid);
    if (id >= 0) {
      programmeItems[id].statement.uid = uid;
      programmeItems[id].statement.name = name;
      programmeItems[id].statement.type = type;

      return programmeItems[id].statement;
    }
    return null;
  }

  var updateWait = function(uid, name, waitType, waitForSeconds, waitForComment) {
    var statement = updateStatement(uid, name, Constants.Statement.STATEMENT_TYPE_WAIT);
    statement.wait_type = waitType;
    if (waitForSeconds == "") 
      statement.kitkat.secs = 0;
    else
      statement.kitkat.secs = parseFloat(waitForSeconds);
    statement.comment = waitForComment;
    return statement;
  }

  var updateComment = function(uid, name, comment) {
    var statement = updateStatement(uid, name, Constants.Statement.STATEMENT_TYPE_COMMENT);
    statement.comment = comment;
    return statement;
  }

  var updateMovement = function(uid, name, pose_uid, path_strategy, collisionCheck) {
    var statement = updateStatement(uid, name, Constants.Statement.STATEMENT_TYPE_MOVEMENT);
    statement.pose_uid = pose_uid;
    statement.path_strategy = path_strategy;
    statement.collision_check = collisionCheck;
    // in case the statement gets converted to a waypoint as some point
    statement.blend = false;
  }

  var updateWaypoint = function(uid, name, pose_uid, blend) {
    var statement = updateStatement(uid, name, Constants.Statement.STATEMENT_TYPE_WAYPOINT);
    statement.pose_uid = pose_uid;
    statement.blend = blend;
  }

  var convertType = function() {
    var id = getActiveId();
    if (programmeItems[id].statement.type == Constants.Statement.STATEMENT_TYPE_MOVEMENT)
      programmeItems[id].statement.type = Constants.Statement.STATEMENT_TYPE_WAYPOINT;
    else
      programmeItems[id].statement.type = Constants.Statement.STATEMENT_TYPE_MOVEMENT;

    // activate the same statement to show the right details
    activateStatement(id);

    updateWidgets();
    storeInDatabase(false);
  }

  // move a statement up or down
  var moveStatement = function(uid, newId) {
    var id = getStatementIDByUID(uid);
    if (id >= 0) {
      var saveStatement = programmeItems[newId];
      programmeItems[newId] = programmeItems[id];
      programmeItems[id] = saveStatement;
    }
  }

  var getStatement = function(idx) {
    return programmeItems[idx];
  }

  var getProgrammeLength = function(idx) {
    return programmeItems.length;
  }


  // return a short string out of a pose that is used in the badges 
  var getBadgeString = function(statement) {
    if (statement.type == Constants.Statement.STATEMENT_TYPE_NONE)
      return null;
    if (statement.type == Constants.Statement.STATEMENT_TYPE_MOVEMENT) {
      var s = "Move " + statement.pose_uid;
      var minibotState = poseStorePanel.getMinibotStateByUID(statement.pose_uid);
      if (minibotState != null)
        s += ":" + minibotState.name;
      return s;
    }
    if (statement.type == Constants.Statement.STATEMENT_TYPE_WAYPOINT) {
      var s = "Point " + statement.pose_uid;
      var minibotState = poseStorePanel.getMinibotStateByUID(statement.pose_uid);
      if (minibotState != null)
        s += ":" + minibotState.name;
      return s;
    }

    if (statement.type == Constants.Statement.STATEMENT_TYPE_COMMENT) {
      return statement.comment.substring(0, 6);
    }
    if (statement.type == Constants.Statement.STATEMENT_TYPE_WAIT) {
      if (statement.wait_type == Constants.Statement.WAIT_TYPE_WAIT)
        return statement.kitkat.secs.toString() + "s";
      else
      if (statement.wait_type == Constants.Statement.WAIT_TYPE_CONFIRMATION)
        return statement.comment.substring(0, 6);
      else
        return "NIL";
    }
    return null;
  }

  var createWaypoint = function(name, pose_uid, blend) {
    var prgItem = newStatement();
    updateWaypoint(prgItem.statement.uid, name, pose_uid,blend);
    updateWidgets();
    return prgItem;
  }

  var createMovement = function(name, pose_uid, path_strategy, collisionCheck) {
    var prgItem = newStatement();
    updateMovement(prgItem.statement.uid, name, pose_uid, path_strategy, collisionCheck);
    updateWidgets();
    return prgItem;
  }

  var createComment = function(name, comment) {
    var prgItem = newStatement();
    updateComment(prgItem.statement.uid, name, comment);
    updateWidgets();
    return prgItem;
  }

  var createWait = function(name, waitType, waitForSeconds, waitForComment) {
    var prgItem = newStatement();
    updateWait(prgItem.statement.uid, name, waitType, waitForSeconds, waitForComment);
    updateWidgets();
    return prgItem;
  }

  var callbackKeyDown = function(event) {
    var li = event.target.parentNode;
    var id = parseInt(li.getAttribute('id'));
    if (event.key == 'Escape') {
      cancelEditMode();
    }
    if (event.key == 'Enter') {
      var prgStmt = getStatement(id);
      prgStmt.statement.name = event.target.value
      cancelEditMode();
      updateWidgets();
      storeInDatabase(false);
    }
  }

  // in case an item is in edit mode, turn it off
  var callbackFocusOut = function(event) {
    cancelEditMode();
  }

  var callbackClick = function(event) {
    var id = event.target.id;
    activateStatement(id);
    poseStorePanel.activateByUID(programmeItems[id].statement.pose_uid);

    // display the trajectory to the next Item
    // planningAction();
  }

  // activate the statement by the passed statement id
  var activateStatement = function(id) {
    id = parseFloat(id); // passed id might be a string
    if (id >= 0) {
      for (var idx = 0; idx < getProgrammeLength(); idx++) {
        var prgItem  = programmeItems[idx];
        if (idx == id) {
          prgItem.widget.classList.add('active');
          if (prgItem.statement.type == Constants.Statement.STATEMENT_TYPE_MOVEMENT)
            document.getElementById('detailsMovementStatement').style.display = 'block';
          else
            document.getElementById('detailsMovementStatement').style.display = 'none';

          if (prgItem.statement.type == Constants.Statement.STATEMENT_TYPE_WAYPOINT)
            document.getElementById('detailsWaypointStatement').style.display = 'block';
          else
            document.getElementById('detailsWaypointStatement').style.display = 'none';

          if (prgItem.statement.type == Constants.Statement.STATEMENT_TYPE_WAIT)
            document.getElementById('detailsWaitStatement').style.display = 'block';
          else
            document.getElementById('detailsWaitStatement').style.display = 'none';

          if (prgItem.statement.type == Constants.Statement.STATEMENT_TYPE_COMMENT)
            document.getElementById('detailsCommentStatement').style.display = 'block';
          else
            document.getElementById('detailsCommentStatement').style.display = 'none';

          updateWidget(idx);
        } else {
          prgItem.widget.classList.remove('active');
        }
      }
    }
  }

  var activatePoseInPosePanel  = function(id) {
    var id = parseInt(event.target.parentNode.getAttribute('id'));
    poseStorePanel.setPoseByUID(programmeItems[id].statement.pose_uid);
  }

  var activatePose = function(event) {
    var id = parseInt(event.target.parentNode.getAttribute('id'));
    poseStorePanel.setPoseByUID(programmeItems[id].statement.pose_uid);
  }

  // called from a double click in a pose
  var renameStatementCallback = function(event) {
    var id = parseInt(event.target.parentNode.getAttribute('id'));
    activateStatement(id);

    renameStatement();
  }

  // return id of active list item
  var getActiveId = function(event) {
    for (var idx = 0; idx < getProgrammeLength(); idx++) {
      if (programmeItems[idx].widget.getAttribute('class').includes('active'))
        return idx;
    }
    return null;
  }


  // rename the currently selected pose
  function renameStatement() {
    var id = getActiveId();
    if (id != null) {
      var programmeItem = getStatement(id);
      var text = programmeItem.widget.childNodes[1];

      // if editmode is one, stop it
      cancelEditMode();

      // change the label to an text input to allow changing the name
      var inputWidget = document.createElement('input');
      inputWidget.id = 'statementPanelInputField'
      inputWidget.onkeydown = callbackKeyDown;
      inputWidget.onfocusout = callbackFocusOut;
      inputWidget.value = programmeItem.statement.name;
      inputWidget.type = 'text';
      // inputWidget.setAttribute('class', 'form-control');
      programmeItem.widget.insertBefore(inputWidget, text);
      programmeItem.widget.removeChild(text);
      inputWidget.focus();
    } else
      displayErr('select a statement first')
  }

  function deleteStatement() {
    var id = getActiveId();
    if (id != null) {
      var li = getStatement(id).widget;

      // if editmode is on, stop it
      cancelEditMode();

      // delete dom node 
      programmeListWidget.removeChild(li);
      for (var idx = id; idx < programmeItems.length - 1; idx++)
        programmeItems[idx] = programmeItems[idx + 1];
      programmeItems.pop();

      // deleting an item changes the IDs
      updateWidgets();

      // activate the next statement
      if (id < programmeItems.length)
        activateStatement(id);
      else
      if (id > 0)
        activateStatement(id - 1);

      storeInDatabase(false);
    } else
      displayErr('select a statement first')
  }

  function newMovement() {
    var pose_uid = poseStorePanel.getCurrentPoseUID();

    if (pose_uid != null) {
      // if editmode is on, stop it
      cancelEditMode();

      // create new element without a name yet
      var prgItem = createMovement('', pose_uid, Constants.Statement.PLAN_SPACE_STRATEGY, false  /* collision_check */);
      var id = getStatementIDByUID(prgItem.statement.uid);

      // scroll to new element, dont activate it yet, the new element needs to be stored in the database first
      // and we cannot start the planning yet
      prgItem.widget.scrollIntoView();

      // immediately store in database
      storeInDatabase(false);

    } else
      displayErr('select a pose first')
  }

  function newWaypoint() {
    var pose_uid = poseStorePanel.getCurrentPoseUID();

    if (pose_uid != null) {
      // if editmode is on, stop it
      cancelEditMode();

      // create new element without a name yet
      var prgItem = createWaypoint('', pose_uid, false  /* blend */);
      var id = getStatementIDByUID(prgItem.statement.uid);

      // scroll to new element, dont activate it yet, the new element needs to be stored in the database first
      // and we cannot start the planning yet
      prgItem.widget.scrollIntoView();

      // immediately store in database
      storeInDatabase(false);

    } else
      displayErr('select a pose first')
  }

  function newComment() {
    // if editmode is on, stop it
    cancelEditMode();

    // create new element without a name yet
    var prgItem = createComment('', '');
    var id = getStatementIDByUID(prgItem.statement.uid);
    // scroll to new element and activate it
    prgItem.widget.scrollIntoView();
    activateStatement(id);

    // immediately store in database
    storeInDatabase(false);
  }

  function newWait() {
    // if editmode is on, stop it
    cancelEditMode();

    // create new element without a name yet
    var prgItem = createWait('', Constants.Statement.WAIT_TYPE_NOWAIT, 0, '');
    var id = getStatementIDByUID(prgItem.statement.uid);
    prgItem.statement.comment = '';
    prgItem.statement.kitkat = new Object();
    prgItem.statement.kitkat.secs = 0;
    prgItem.statement.kitkat.nsecs = 0;
    // scroll to new element and activate it
    prgItem.widget.scrollIntoView();
    activateStatement(id);

    // immediately store in database
    storeInDatabase(false);
  }


  function up() {
    var id = getActiveId();

    if (id > 0) {
      var uid = programmeItems[id].uid;
      var prgItem = programmeItems[id];
      var prevPrgItem = programmeItems[id - 1];

      programmeListWidget.removeChild(prgItem.widget);
      moveStatement(uid, id - 1);


      programmeListWidget.insertBefore(prgItem.widget, prevPrgItem.widget);

      prgItem.widget.scrollIntoView();
      updateWidgets();

      // immediately store in database
      storeInDatabase(false);
    } else
      displayErr('selected statement is first already')
  }

  function down() {
    var id = getActiveId();

    if (id < programmeItems.length - 1) {
      var uid = programmeItems[id].uid;
      var prgItem = programmeItems[id];
      programmeListWidget.removeChild(prgItem.widget);
      moveStatement(uid, id + 1);
      if (id == programmeItems.length - 2)
        programmeListWidget.append(prgItem.widget);
      else
        programmeListWidget.insertBefore(prgItem.widget, programmeItems[id + 2].widget);

      prgItem.widget.scrollIntoView();
      updateWidgets();

      // immediately store in database
      storeInDatabase(false);
    } else
      displayErr('selected statement is last already')
  }

  function setPathStrategy(event) {
    var id = getActiveId();
    if (id >= 0) {
      var prgItem = programmeItems[id];
      if (event.target.id == "spacePath")
        prgItem.statement.path_strategy = Constants.Statement.PLAN_SPACE_STRATEGY;
      if (event.target.id == "cartesicPath")
        prgItem.statement.path_strategy = Constants.Statement.PLAN_CARTESIC_STRATEGY;
      if (event.target.id == "splinePath")
        prgItem.statement.path_strategy = Constants.Statement.PLAN_SPLINE_STRATEGY;
      updateWidget(id);

      // immediately store in database
      storeInDatabase(false);
    }
  }

  function setBlendWaypoint(event) {
    var id = getActiveId();
    if (id >= 0) {
      var prgItem = programmeItems[id];
      prgItem.statement.bend = event.target.checked;
      updateWidget(id);

      // immediately store in database
      storeInDatabase(false);
    }
  }


  function setCollisionCheck(event) {
    var id = getActiveId();
    if (id >= 0) {
      var prgItem = programmeItems[id];
      prgItem.statement.collision_check = event.target.checked;
      updateWidget(id);

      // immediately store in database
      storeInDatabase(false);
    }
  }

  function setWaitType(waitType) {
    var id = getActiveId();
    if (id >= 0) {
      var prgItem = programmeItems[id];
      prgItem.statement.wait_type = waitType;
      updateWidget(id);
      storeInDatabase(false);
    }
  }

  function setWaitTypeNoWait(event) {
    setWaitType(Constants.Statement.WAIT_TYPE_NOWAIT);
    storeInDatabase(false);
  }

  function setWaitTypeWaitForConfirmation(event) {
    setWaitType(Constants.Statement.WAIT_TYPE_CONFIRMATION);
    storeInDatabase(false);
  }

  function setWaitTypeWaitForSeconds(event) {
    setWaitType(Constants.Statement.WAIT_TYPE_WAIT);
    storeInDatabase(false);
  }


  function setWaitingSeconds(event) {
    var id = getActiveId();
    if (id >= 0) {
      var prgItem = programmeItems[id];
      prgItem.statement.kitkat.secs = parseFloat(event.target.value);

      // activate the wait type
      prgItem.statement.wait_type = Constants.Statement.WAIT_TYPE_WAIT;

      updateWidget(id);
      storeInDatabase(false);
    }
  }

  function setConfirmationText(event) {
    var id = getActiveId();
    if (id >= 0) {
      var prgItem = programmeItems[id];
      prgItem.statement.comment = event.target.value;

      // activate the wait type
      prgItem.statement.wait_type = Constants.Statement.WAIT_TYPE_CONFIRMATION;

      updateWidget(id);
      storeInDatabase(false);
    }
  }

  function setComment(event) {
    var id = getActiveId();
    if (id >= 0) {
      var prgItem = programmeItems[id];
      prgItem.statement.comment = event.target.value;
      updateWidget(id);
      storeInDatabase(false);
    }
  }


  // in case a list item is in edit mode, turn back to normal
  var cancelEditMode = function() {
    inputWidget = document.getElementById('statementPanelInputField');
    if (inputWidget != null) {
      var id = inputWidget.parentNode.id;
      var prgItem = programmeItems[id];
      var li = prgItem.widget;
      var text = document.createTextNode(prgItem.statement.name);
      li.removeChild(inputWidget);
      li.insertBefore(text, li.childNodes[1]); // insert between the two spans
    }
  }

  var callbackNameKeyDown = function(event) {
    var li = event.target.parentNode;
    var id = parseInt(li.getAttribute('id'));
    if (event.key == 'Escape') {
      cancelEditMode();
    }
    if (event.key == 'Enter') {
      var prgItem = programmeItems[id];
      prgItem.statement.name = event.target.value;
      cancelEditMode();

      // immediately store in database
      storeInDatabase(false);

      updateWidgets;
    }
  }

  // in case an item is in edit mode, turn it off
  var callbackNameFocusOut = function(event) {
    cancelEditMode();
  }

  // called from outside when something changed that could influence the 
  //programmepanel, does a complete refresh of all widgets
  var refresh = function() {
    updateWidgets();
  }

  // take the pose from the posePanel and assign it to the current waypoint 
  var assignActivePose = function() {
    var uid = poseStorePanel.getCurrentPoseUID();
    if (uid != null) {
      var id = getActiveId();
      if (id != null) {
        var prgItem = programmeItems[id];
        Utils.assert(
          (prgItem.statement.type == Constants.Statement.Statement.STATEMENT_TYPE_MOVEMENT) || 
          (prgItem.statement.type == Constants.Statement.Statement.STATEMENT_TYPE_WAYPOINT), "pose can be assigned to waypoint or move");
        prgItem.statement.pose_uid = uid;
        updateWidget(id);
      } else
        displayErr("select a statement first");
    } else
      displayErr("select a pose first");
  }

  // return the id of the last continuous waypoint starting from the active statement
  var getNumberOfWaypoints = function() {
    var id = getActiveId();
    if (id != null && id >= 0 && 
      ((programmeItems[id].type == Constants.Statement.Statement.STATEMENT_TYPE_WAYPOINT) ||
       (programmeItems[id].type == Constants.Statement.Statement.STATEMENT_TYPE_MOVEMENT))) {
      var endID = -1;
      for (var idx = id + 1; idx < programmeItems.length; idx++) {
        if (programmeItems[idx].type == Constants.Statement.Statement.STATEMENT_TYPE_WAYPOINT) {
          endID = idx;
        } else
          break;
      }
      if (endID >= 0) {
        return endID;
      }
    }
    return -1;
  }

  // called when a pose is modified and we need to replan, if the pose is used in a statement
  var modifyPose = function(poseUID) {
    var statementID = getStatementIDByPoseUID (poseUID)
    if (statementID != null) {
      updateWidgets();
      storeInDatabase(false);
    }
  }

  // save  the programm to the server
  var planningAction = function() {
    // anything active?
    var startID = getActiveId();
    if (startID == null || startID < 0) {
      displayErr("select a statement first");
      return;
    }

    var request = new ROSLIB.ServiceRequest({
        type: Constants.Planning.SELECT_LOCAL_PLAN,
        startStatementUID: programmeItems[startID].statement.uid
    });

    var planningAction = new ROSLIB.Service({
      ros: ros,
      name: '/planning_action',
      serviceType: 'minibot/PlanningAction'
    });

    planningAction.callService(request,
      function(response) {
        var id = getActiveId();
        if (response.error_code.val == Constants.ErrorCodes.SUCCESS)
          programmeItems[id].statement.error_code.val = response.error_code.val;
        else {
          programmeItems[id].statement.error_code.val = response.error_code.val;
        }
      },
      function(response) {
        var id = getActiveId();
        programmeItems[id].statement.error_code.val = Constants.ErrorCodes.FAILURE;
      });
  }


  var simulatePlan = function() {
    var startID = getActiveId();
    if (startID == null || startID < 0 || 
       ((programmeItems[startID].statement.type != Constants.Statement.STATEMENT_TYPE_WAYPOINT) || 
        (programmeItems[startID].statement.type != Constants.Statement.STATEMENT_TYPE_MOVEMENT))) {
      displayErr("select a statement first");
      return;
    }

    var endID = getNumberOfWaypoints();
    if (endID == null || endID < 0) {
      displayErr("select a move sequence first");
      return;
    }

    // activate the pose of that statement as starting point
    poseStorePanel.setPoseByUID(programmeItems[startID].statement.pose_uid);

    var request = new ROSLIB.ServiceRequest({
      type: Constants.PlannigAction.SIMULATE_PLAN,
      startStatementUID: programmeItems[startID].statement.uid,
      endStatementUID: programmeItems[endID].statement.uid
    });

    var planningAction = new ROSLIB.Service({
      ros: ros,
      name: '/planning_action',
      serviceType: 'minibot/PlanningAction'
    });

    planningAction.callService(request,
      function(response) {
        var id = getActiveId();
        if (response.error_code.val == Constants.ErrorCodes.SUCCESS)
          programmeItems[id].statement.error_code.val = response.error_code.val;
        else {
          programmeItems[id].statement.error_code.val = response.error_code.val;
        }
      },
      function(response) {
        var id = getActiveId();
        programmeItems[id].statement.error_code.val = Constants.ErrorCodes.FAILURE;
      });
  }



  var visualizePlan = function(type, ok) {
    var request = new ROSLIB.ServiceRequest({
      type: type,
      jfdi : ok
    });

    var planningAction = new ROSLIB.Service({
      ros: ros,
      name: '/planning_action',
      serviceType: 'minibot/PlanningAction'
    });

    planningAction.callService(request,
      function(response) {
        if (response.error_code.val != Constants.ErrorCodes.SUCCESS)
          displayErr("could not change plan visualization({0}".format(response.error_code.val));
      },
      function(response) {
          displayErr("could not change plan visualization({0}".format(response.error_code.val));
      });
  }

  var visualizeGlobalPlan = function(event) {
    visualizePlan(Constants.PlanningAction.VIS_GLOBAL_PLAN, event.target.checked);
    settingsPanel.setVisualizationGlobalPlan(event.target.checked);
  }

  var visualizeLocalPlan = function(event) {
    visualizePlan(Constants.PlanningAction.VIS_LOCAL_PLAN, event.target.checked);
    settingsPanel.setVisualizationLocalPlan(event.target.checked);
  }

  var forward = function(event) {
    var startID = getActiveId();
    if (startID == null) {
      displayErr("select a statement first");
      return;
    }

    if (startID >= programmeItems.length-1) {
      displayErr("last statement, cannot go forward");
      return;
    }

    if (programmeItems[startID].type == Constants.Statement.STATEMENT_TYPE_WAYPOINT) {

      // activate the pose of that statement as starting point
      poseStorePanel.setPoseByUID(programmeItems[startID].statement.pose_uid);

      var request = new ROSLIB.ServiceRequest({
        type: Constants.Planning.ACTION_STEP_FORWARD,
        startStatementUID: programmeItems[startID].statement.uid
      });

      var planningAction = new ROSLIB.Service({
        ros: ros,
        name: '/planning_action',
        serviceType: 'minibot/PlanningAction'
      });

      planningAction.callService(request,
        function(response) {
          var id = getActiveId();
          if (response.error_code.val == ErrorCode.ErrorCodes.SUCCESS)
            programmeItems[id].statement.error_code.val = response.error_code.val;
          if (programmeItems.length > startID+1)
            activateStatement(startID + 1);
          else {
            programmeItems[id].statement.error_code.val = response.error_code.val;
          }
        },
        function(response) {
          var id = getActiveId();
          programmeItems[id].statement.error_code.val = ErrorCode.ErrorCodes.FAILURE;
        });
    } 
    else {
        activateStatement(startID + 1);
    }
  }

  var run = function(event) {
    simulatePlan();
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
    displayAlert(t, document.getElementById('programmeAlertHeadline'), document.getElementById('programmeAlertSuccess'));
  }

  function displayErr(t) {
    displayAlert(t, document.getElementById('programmeAlertHeadline'), document.getElementById('programmeAlertError'));
  }

  function displayWarn(t) {
    Alert(t, document.getElementById('programmeAlertHeadline'), document.getElementById('programmeAlertWarning'));
  }

  // exposed inner functions
  return {
    deleteStatement: deleteStatement,
    newWaypoint: newWaypoint,
    newWait: newWait,
    newComment: newComment,
    newMovement : newMovement,
    renameStatement: renameStatement,
    up: up,
    down: down,

    getStatementIDByPoseUID: getStatementIDByPoseUID,
    activateStatement: activateStatement,

    setPathStrategy: setPathStrategy,
    setCollisionCheck: setCollisionCheck,
    assignActivePose: assignActivePose,
    setBlendWaypoint : setBlendWaypoint,

    setWaitingSeconds: setWaitingSeconds,
    setConfirmationText: setConfirmationText,
    setComment: setComment,

    setWaitTypeWaitForSeconds: setWaitTypeWaitForSeconds,
    setWaitTypeNoWait: setWaitTypeNoWait,
    setWaitTypeWaitForConfirmation: setWaitTypeWaitForConfirmation,

    visualizeLocalPlan : visualizeLocalPlan,
    visualizeGlobalPlan : visualizeGlobalPlan,

    modifyPose : modifyPose,                          // called when a pose has been modified, updates the database and the plan
    convertType : convertType,
    forward: forward,
    run: run,
    refresh: refresh,
    initialize : initialize
  };
};