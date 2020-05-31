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


  // types of statements
  const StatementType = {
    NoStatement: 1,
    WayPoint: 2,
    Comment: 3,
    Wait: 4
  };

  const WaitType = {
    NoWait: 1,
    WaitForSeconds: 2,
    WaitForConfirmation: 3
  };

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
        if (result.error_code.val == ErrorCode.MOVEIT.SUCCESS) {
          programmeItems = [];
          for (var idx = 0; idx < result.programme_store.statements.length; idx++) {
            var statementDB = result.programme_store.statements[idx];
            var stmt = newStatement();
            stmt.uid = statementDB.uid;
            stmt.name = statementDB.name;
            if (statementDB.type == StatementType.WayPoint) {
              stmt.type = StatementType.WayPoint;
              stmt.poseUID = statementDB.pose_uid;
              stmt.cartesicPath = statementDB.cartesic_path;
              stmt.collisionCheck = statementDB.collision_check;
              stmt.improvedPath = statementDB.improved_path;
            }
            if (statementDB.type == StatementType.Wait) {
              stmt.type = StatementType.Wait;
              stmt.waitForSeconds = Math.floor((statementDB.kitkat.secs + statementDB.kitkat.nsecs / 1000000000) * 1000) / 1000;
              stmt.waitType = statementDB.waitType;
            }
            if (statementDB.type == StatementType.Comment) {
              stmt.type = StatementType.Comment;
              stmt.comment = statementDB.comment;
            }
          }

          // update DOM 
          updateWidgets();

          // and select the first statement if it is there
          if (programmeItems.length > 0) {
            activateStatement(1);
          }
        } else {
          displayErr("reading programme from database failed " + result.error_code.val);
        }
      },
      function(result) {
        displayErr("reading programme from database failed " + result);
      })

      var listenToClientActions   = new ROSLIB.Topic({
        ros : ros,
        name : '/client_action',
        messageType : 'minibot/ClientAction'
      });

      listenToClientActions.subscribe(function(clientAction) {
        if (clientAction.type == Constants.Planning.ACTIVATE_STATEMENT) {
          var block = clientAction.statement_block_no;
          var blockCounter = 0;
          var inblock = false; 
          for (var idx = 0; idx < programmeItems.length; idx++) {
            if (!inblock) {
              if (programmeItems[idx].type == StatementType.WayPoint) {
                  blockCounter = blockCounter +1;
                  inblock = true;
                  if (blockCounter == block) {
                    activateStatement(idx);
                    break;
                  }
              } 
            } else {
              if (programmeItems[idx].type != StatementType.WayPoint) {
                  inblock = false;
              }
            } 
          }
       }
      });
  };


  var storeInDatabase = function(force) {
    if (force)
      Utils.callDelay("storedatabase", 0, function() { 
        displayInfo("store & replan...")
        rawStoreInDatabase() 
        displayInfo("store & replan... done")
      }); 
    else
      Utils.callDelay("storedatabase", settingsPanel.getSaveAfterSeconds(), function() { 
        displayInfo("store & replan...")
        rawStoreInDatabase() 
        displayInfo("store & replan... done")
      }); 

  }

  var rawStoreInDatabase = function() {

    var statementsDB = [];
    for (var idx = 0; idx < programmeItems.length; idx++) {
      var statement = programmeItems[idx];
      var statementDB = new Object();
      statementDB.uid = statement.uid;
      statementDB.name = statement.name;

      if (statement.type == StatementType.WayPoint) {
        statementDB.type = StatementType.WayPoint;
        statementDB.pose_uid = statement.poseUID;
        statementDB.cartesic_path = statement.cartesicPath;
        statementDB.collision_check = statement.collisionCheck;
        statementDB.improved_path = statement.improvedPath;
      }
      if (statement.type == StatementType.Wait) {
        statementDB.type = StatementType.Wait;
        statementDB.kitkat = new Object();
        statementDB.kitkat.secs = Math.floor(statement.waitForSeconds);
        statementDB.kitkat.nsecs = (statement.waitForSeconds - statementDB.kitkat.secs) * 1000000000;
        statementDB.waitType = statement.waitType;
      }
      if (statement.type == StatementType.Comment) {
        statementDB.type = StatementType.Comment;
        statementDB.comment = statement.comment;
      }

      statementsDB[idx] = statementDB;
    }

    // read the poses from the database:q
    var request = new ROSLIB.ServiceRequest({
      type: Constants.Database.WRITE_PROGRAMME,
      programme_store: {
        statements: statementsDB
      }
    });
    var databaseAction = new ROSLIB.Service({
      ros: ros,
      name: '/database',
      serviceType: 'minibot/Database'
    });

    databaseAction.callService(request,
      function(result) {
        if (result.error_code.val == ErrorCode.MOVEIT.SUCCESS) {
          displayInfo("saved");
        } else {
          displayErr("storing in database failed " + result.error_code.val);
        }
      },
      function(result) {
        displayErr("storing in database failed " + result.toString());
      });
  }


  var updateWidget = function(idx) {
    var statement = programmeItems[idx];
    var id = idx;

    var idbadge = statement.widget.childNodes[0];
    var text = statement.widget.childNodes[1];
    var badge = statement.widget.childNodes[2];
    idbadge.innerHTML = (id + 1).toString() + '<br/>' + statement.uid.toString();
    text.textContent = statement.name;
    badge.innerHTML = getBadgeString(statement);

    // set an id for proper identification of the widget in callbacks
    statement.widget.id = idx;
    idbadge.id = idx;
    text.id = idx;
    badge.id = idx;

    // set color of right badge
    if (statement.error_code != null && statement.error_code != ErrorCode.PLANNING.SUCCESS) {
      badge.classList.remove("badge-success");
      badge.classList.add("badge-danger");
    } else {
      badge.classList.remove("badge-danger");
      badge.classList.add("badge-success");
    }

    // remove color of waypoints
    statement.widget.classList.remove('list-group-item-success');
    badge.classList.remove('badge-secondary');
    badge.classList.remove('badge-primary');

    if (statement.type == StatementType.Wait) {
      statement.widget.classList.add('list-group-item-light');
      badge.classList.add('badge-secondary');

      // even if a different radio button is on, display the other fields
      document.getElementById('waitConfirmationText').value = statement.comment;
      document.getElementById('secondsToWait').value = statement.waitForSeconds;

      document.getElementById('waitForSeconds').checked = false;
      document.getElementById('waitForConfirmation').checked = false;
      document.getElementById('dontWait').checked = false;
      if (statement.waitType == WaitType.WaitForSeconds)
        document.getElementById('waitForSeconds').checked = true;
      else
      if (statement.waitType == WaitType.WaitForConfirmation)
        document.getElementById('waitForConfirmation').checked = true;
      else
      if (statement.waitType == WaitType.NoWait)
        document.getElementById('dontWait').checked = true;
    }
    if (statement.type == StatementType.WayPoint) {
      badge.classList.add('badge-primary');
      statement.widget.classList.add('list-group-item-success');
      document.getElementById('cartesicPath').checked = statement.cartesicPath;
      document.getElementById('collisionCheck').checked = statement.collisionCheck;
      document.getElementById('improvedPath').checked = statement.improvedPath;
      var waypointErrorWidget = document.getElementById('waypointError');
      if (statement.error_code != null && statement.error_code != ErrorCode.PLANNING.SUCCESS) {
        waypointErrorWidget.style.display = 'block';
        waypointErrorWidget.innerHTML = "Planning error " + statement.error_code;
      } else {
        document.getElementById('waypointError').style.display = 'none';
        waypointErrorWidget.innerHTML = "";
      }
    }
    if (statement.type == StatementType.Comment) {
      badge.classList.add('badge-secondary');
      document.getElementById('commentStatement').value = statement.comment;
    }
  }


  var updateWidgets = function() {
    for (var idx = 0; idx < getProgrammeLength(); idx++)
      updateWidget(idx);
    if (getProgrammeLength() == 0) {
      // last element? then hide the detail view
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
      if (programmeItems[idx].uid == uid)
        return idx;
    }
    return -1;
  }

  // returns the id of the statement that refers to a given pose ID  
  var getStatementIDByPoseUID = function(uid) {
    var result = 0;
    for (var idx = 0; idx < getProgrammeLength(); idx++) {
      if (programmeItems[idx].poseUID == uid)
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
    var statement = new Object();

    // compute a new UID
    var max = programmeItems.length;
    if (max == 0)
      max = 1;
    for (var idx = 0; idx < programmeItems.length; idx++) {
      var currUID = programmeItems[idx].uid;
      if (currUID >= max)
        max = currUID + 1;
    }

    // this methods ends with an empty statement
    statement.uid = max;
    statement.name = null;
    statement.type = StatementType.NoStatement;

    var li = document.createElement('LI');
    li.setAttribute('class', 'list-group-item py-1 list-group-item-action justify-content-center align-self-center p-1');

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
    statement.widget = li;
    li.onclick = callbackClick;

    // add the item to the end of the list
    programmeItems[programmeItems.length] = statement;

    return statement;
  }


  var updateStatement = function(uid, name, type) {
    var id = getStatementIDByUID(uid);
    if (id >= 0) {
      programmeItems[id].name = name;
      programmeItems[id].type = type;

      return programmeItems[id];
    }
    return null;
  }

  var updateWait = function(uid, name, waitType, waitForSeconds, waitForComment) {
    var statement = updateStatement(uid, name, StatementType.Wait);
    statement.waitType = waitType;
    if (waitForSeconds == "")
      statement.waitForSeconds = 0;
    else
      statement.waitForSeconds = parseFloat(waitForSeconds);
    statement.comment = waitForComment;
    return statement;
  }

  var updateComment = function(uid, name, comment) {
    var statement = updateStatement(uid, name, StatementType.Comment);
    statement.comment = comment;
    return statement;
  }

  var updateWaypoint = function(uid, name, poseUID, cartesicPath, collisionCheck, improvedPath) {
    var statement = updateStatement(uid, name, StatementType.WayPoint);
    statement.poseUID = poseUID;
    statement.cartesicPath = cartesicPath;
    statement.collisionCheck = collisionCheck;
    statement.improvedPath = improvedPath;
  }

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
    if (statement.type == StatementType.NoStatement)
      return null;
    if (statement.type == StatementType.WayPoint) {
      var s = "Pose " + statement.poseUID;
      var name = poseStorePanel.getNameByUID(statement.poseUID);
      if (name != null)
        s += ":" + name;
      return s;
    }
    if (statement.type == StatementType.Comment) {
      return statement.comment.substring(0, 16);
    }
    if (statement.type == StatementType.Wait) {
      if (statement.waitType == WaitType.WaitForSeconds)
        return statement.waitForSeconds.toString() + "s";
      else
      if (statement.waitType == WaitType.WaitForConfirmation)
        return statement.comment;
      else
        return "NIL";
    }
    return null;
  }

  var createWaypoint = function(name, poseUID, cartesicPath, collisionCheck, improvedPath) {
    var statement = newStatement();
    updateWaypoint(statement.uid, name, poseUID, cartesicPath, collisionCheck, improvedPath);
    updateWidgets();
    return statement;
  }

  var createComment = function(name, comment) {
    var statement = newStatement();
    updateComment(statement.uid, name, comment);
    updateWidgets();
    return statement;
  }

  var createWait = function(name, wait, waitType, waitForSeconds, waitForComment) {
    var statement = newStatement();
    updateWait(statement.uid, name, wait, waitType, waitForSeconds, waitForComment);
    updateWidgets();
    return statement;
  }


  var callbackKeyDown = function(event) {
    var li = event.target.parentNode;
    var id = parseInt(li.getAttribute('id'));
    if (event.key == 'Escape') {
      cancelEditMode();
    }
    if (event.key == 'Enter') {
      var poseItem = getStatement(id);
      programmeItem.name = event.target.value
      cancelEditMode();
      updateWidgets;
    }
  }

  // in case an item is in edit mode, turn it off
  var callbackFocusOut = function(event) {
    cancelEditMode();
  }

  var callbackClick = function(event) {
    var id = event.target.id;
    activateStatement(id);
    poseStorePanel.activateByUID(programmeItems[id].poseUID);

    // display the trajectory to the next Item
    planningAction();
  }

  // activate the statement by the passed statement id
  var activateStatement = function(id) {
    id = parseFloat(id); // passed id might be a string
    if (id >= 0) {
      for (var idx = 0; idx < getProgrammeLength(); idx++) {
        if (idx == id) {
          var statement = programmeItems[idx];
          statement.widget.classList.add('active');
          if (statement.type == StatementType.WayPoint)
            document.getElementById('detailsWaypointStatement').style.display = 'block';
          else
            document.getElementById('detailsWaypointStatement').style.display = 'none';

          if (statement.type == StatementType.Wait)
            document.getElementById('detailsWaitStatement').style.display = 'block';
          else
            document.getElementById('detailsWaitStatement').style.display = 'none';

          if (statement.type == StatementType.Comment)
            document.getElementById('detailsCommentStatement').style.display = 'block';
          else
            document.getElementById('detailsCommentStatement').style.display = 'none';
          updateWidget(idx);
        } else {
          programmeItems[idx].widget.classList.remove('active');
        }
      }
    }
  }

  var activatePoseInPosePanel  = function(id) {
    var id = parseInt(event.target.parentNode.getAttribute('id'));
    poseStorePanel.setPoseByUID(programmeItems[id].poseUID);
  }

  var activatePose = function(event) {
    var id = parseInt(event.target.parentNode.getAttribute('id'));
    poseStorePanel.setPoseByUID(programmeItems[id].poseUID);
  }

  // return id of active list item
  var getActiveId = function(event) {
    for (var idx = 0; idx < getProgrammeLength(); idx++) {
      if (programmeItems[idx].widget.getAttribute('class').includes('active'))
        return idx;
    }
    return null;
  }

  // in case a list item is in edit mode, turn back to normal
  var cancelEditMode = function() {
    inputWidget = document.getElementById('statementPanelInputField');
    if (inputWidget != null) {
      var id = inputWidget.parentNode.id;
      var poseItem = getStatement(id);
      var li = getStatement(id).widget;
      var text = document.createTextNode(programmeItem.name);
      li.removeChild(inputWidget);
      li.insertBefore(text, li.childNodes[1]); // insert between the two spans
    }
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
      inputWidget.value = programmeItem.name;
      inputWidget.type = 'text';
      inputWidget.setAttribute('class', 'form-control');
      programmeItem.widget.appendChild(inputWidget);
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

  function newWaypoint() {
    var poseUID = poseStorePanel.getCurrentPoseUID();

    if (poseUID != null) {
      // if editmode is on, stop it
      cancelEditMode();

      // create new element without a name yet
      var statement = createWaypoint('', poseUID, false, false, false);
      var id = getStatementIDByUID(statement.uid);

      // scroll to new element, dont activate it yet, the new element needs to be stored in the database first
      // and we cannot start the planning yet
      statement.widget.scrollIntoView();

      // immediately store in database
      storeInDatabase(false);

    } else
      displayErr('select a pose first')
  }

  function newComment() {
    // if editmode is on, stop it
    cancelEditMode();

    // create new element without a name yet
    var statement = createComment('', '');
    var id = getStatementIDByUID(statement.uid);
    // scroll to new element and activate it
    statement.widget.scrollIntoView();
    activateStatement(id);

    // immediately store in database
    storeInDatabase(false);
  }

  function newWait() {
    // if editmode is on, stop it
    cancelEditMode();

    // create new element without a name yet
    var statement = createWait('', WaitType.NoWait, '');
    var id = getStatementIDByUID(statement.uid);
    statement.comment = '';
    statement.waitForSeconds = 0;
    // scroll to new element and activate it
    statement.widget.scrollIntoView();
    activateStatement(id);

    // immediately store in database
    storeInDatabase(false);
  }


  function up() {
    var id = getActiveId();

    if (id > 0) {
      var uid = programmeItems[id].uid;
      var statement = programmeItems[id];
      var prevStatement = programmeItems[id - 1];

      programmeListWidget.removeChild(statement.widget);
      moveStatement(uid, id - 1);


      programmeListWidget.insertBefore(statement.widget, prevStatement.widget);

      statement.widget.scrollIntoView();
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
      var statement = programmeItems[id];
      programmeListWidget.removeChild(statement.widget);
      moveStatement(uid, id + 1);
      if (id == programmeItems.length - 2)
        programmeListWidget.append(statement.widget);
      else
        programmeListWidget.insertBefore(statement.widget, programmeItems[id + 2].widget);

      statement.widget.scrollIntoView();
      updateWidgets();

      // immediately store in database
      storeInDatabase(false);
    } else
      displayErr('selected statement is last already')
  }

  function setCartesicPath(event) {
    var id = getActiveId();
    if (id >= 0) {
      var statement = programmeItems[id];
      if (event.target.checked)
        statement.cartesicPath = true;
      else
        statement.cartesicPath = false;
      updateWidget(id);

      // immediately store in database
      storeInDatabase(false);
    }
  }

  function setCollisionCheck(event) {
    var id = getActiveId();
    if (id >= 0) {
      var statement = programmeItems[id];
      if (event.target.checked)
        statement.collisionCheck = true;
      else
        statement.collisionCheck = false;
      updateWidget(id);

      // immediately store in database
      storeInDatabase(false);
    }
  }

  function setImprovedPath(event) {
    var id = getActiveId();
    if (id >= 0) {
      var statement = programmeItems[id];
      if (event.target.checked)
        statement.improvedPath = true;
      else
        statement.improvedPath = false;
      updateWidget(id);

      // immediately store in database
      storeInDatabase(false);
    }
  }

  function setWaitType(waitType) {
    var id = getActiveId();
    if (id >= 0) {
      var statement = programmeItems[id];
      statement.waitType = waitType;
      updateWidget(id);
      storeInDatabase(false);
    }
  }

  function setWaitTypeNoWait(event) {
    setWaitType(WaitType.NoWait);
    storeInDatabase(false);
  }

  function setWaitTypeWaitForConfirmation(event) {
    setWaitType(WaitType.WaitForConfirmation);
    storeInDatabase(false);
  }

  function setWaitTypeWaitForSeconds(event) {
    setWaitType(WaitType.WaitForSeconds);
    storeInDatabase(false);
  }


  function setWaitingSeconds(event) {
    var id = getActiveId();
    if (id >= 0) {
      var statement = programmeItems[id];
      statement.waitForSeconds = Utils.makeFloatString(event.target.value);
      updateWidget(id);
      storeInDatabase(false);
    }
  }

  function setConfirmationText(event) {
    var id = getActiveId();
    if (id >= 0) {
      var statement = programmeItems[id];
      statement.comment = event.target.value;
      updateWidget(id);
      storeInDatabase(false);
    }
  }

  function setComment(event) {
    var id = getActiveId();
    if (id >= 0) {
      var statement = programmeItems[id];
      statement.comment = event.target.value;
      updateWidget(id);
      storeInDatabase(false);
    }
  }


  // in case a list item is in edit mode, turn back to normal
  var cancelEditMode = function() {
    inputWidget = document.getElementById('programmeTextInputField');
    if (inputWidget != null) {
      var id = inputWidget.parentNode.id;
      var statement = programmeItems[id];
      var li = statement.widget;
      var text = document.createTextNode(statement.name);
      li.removeChild(inputWidget);
      li.insertBefore(text, li.childNodes[1]); // insert between the two spans
    }
  }

  // rename the currently selected pose
  function rename() {
    var id = getActiveId();
    if (id != null) {
      var statement = programmeItems[id];
      var text = statement.widget.childNodes[1];

      // if editmode is one, stop it
      cancelEditMode();

      // change the label to an text input to allow changing the name
      var inputWidget = document.createElement('input');
      inputWidget.id = 'programmeTextInputField'
      inputWidget.onkeydown = callbackNameKeyDown;
      inputWidget.onfocusout = callbackNameFocusOut;
      inputWidget.value = statement.name;
      inputWidget.type = 'text';
      inputWidget.setAttribute('class', 'form-control');
      statement.widget.appendChild(inputWidget);
      statement.widget.removeChild(text);
      inputWidget.focus();
    } else
      displayErr('select a statement first')
  }

  var callbackNameKeyDown = function(event) {
    var li = event.target.parentNode;
    var id = parseInt(li.getAttribute('id'));
    if (event.key == 'Escape') {
      cancelEditMode();
    }
    if (event.key == 'Enter') {
      var statement = programmeItems[id];
      statement.name = event.target.value;
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
        var statement = programmeItems[id];
        Utils.assert(statement.type == StatementType.WayPoint, "cant assin pose to non-waypoint");
        statement.poseUID = uid;
        updateWidget(id);
      } else
        displayErr("select a statement first");
    } else
      displayErr("select a pose first");
  }

  // return the id of the last continuous waypoint starting from the active statement
  var getNumberOfWaypoints = function() {
    var id = getActiveId();
    if (id != null && id >= 0 && programmeItems[id].type == StatementType.WayPoint) {
      var endID = -1;
      for (var idx = id + 1; idx < programmeItems.length; idx++) {
        if (programmeItems[idx].type == StatementType.WayPoint) {
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
        type: Constants.Planning.ACTION_SELECT_LOCAL_PLAN,
        startStatementUID: programmeItems[startID].uid
    });

    var planningAction = new ROSLIB.Service({
      ros: ros,
      name: '/planning_action',
      serviceType: 'minibot/PlanningAction'
    });

    planningAction.callService(request,
      function(response) {
        var id = getActiveId();
        if (response.error_code.val == ErrorCode.PLANNING.SUCCESS)
          programmeItems[id].error_code = response.error_code.val;
        else {
          programmeItems[id].error_code = response.error_code.val;
        }
      },
      function(response) {
        var id = getActiveId();
        programmeItems[id].error_code = ErrorCode.PLANNING.FAILURE;
      });
  }


  var simulatePlan = function() {
    var startID = getActiveId();
    if (startID == null || startID < 0 || programmeItems[startID].type != StatementType.WayPoint) {
      displayErr("select a statement first");
      return;
    }

    var endID = getNumberOfWaypoints();
    if (endID == null || endID < 0) {
      displayErr("select a waypoint sequence first");
      return;
    }

    // activate the pose of that statement as starting point
    poseStorePanel.setPoseByUID(programmeItems[startID].poseUID);

    var request = new ROSLIB.ServiceRequest({
      type: Constants.Planning.ACTION_SIMULATE_PLAN,
      startStatementUID: programmeItems[startID].uid,
      endStatementUID: programmeItems[endID].uid
    });

    var planningAction = new ROSLIB.Service({
      ros: ros,
      name: '/planning_action',
      serviceType: 'minibot/PlanningAction'
    });

    planningAction.callService(request,
      function(response) {
        var id = getActiveId();
        if (response.error_code.val == ErrorCode.PLANNING.SUCCESS)
          programmeItems[id].error_code = response.error_code.val;
        else {
          programmeItems[id].error_code = response.error_code.val;
        }
      },
      function(response) {
        var id = getActiveId();
        programmeItems[id].error_code = ErrorCode.PLANNING.FAILURE;
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
        if (response.error_code.val != ErrorCode.PLANNING.SUCCESS)
          displayErr("could not change plan visualization({0}".format(response.error_code.val));
      },
      function(response) {
          displayErr("could not change plan visualization({0}".format(response.error_code.val));
      });
  }

  var visualizeGlobalPlan = function(event) {
    visualizePlan(Constants.Planning.ACTION_VIS_GLOBAL_PLAN, event.target.checked);
    settingsPanel.setVisualizationGlobalPlan(event.target.checked);
  }

  var visualizeLocalPlan = function(event) {
    visualizePlan(Constants.Planning.ACTION_VIS_LOCAL_PLAN, event.target.checked);
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

    if (programmeItems[startID].type == StatementType.WayPoint) {

      // activate the pose of that statement as starting point
      poseStorePanel.setPoseByUID(programmeItems[startID].poseUID);

      var request = new ROSLIB.ServiceRequest({
        type: Constants.Planning.ACTION_STEP_FORWARD,
        startStatementUID: programmeItems[startID].uid
      });

      var planningAction = new ROSLIB.Service({
        ros: ros,
        name: '/planning_action',
        serviceType: 'minibot/PlanningAction'
      });

      planningAction.callService(request,
        function(response) {
          var id = getActiveId();
          if (response.error_code.val == ErrorCode.PLANNING.SUCCESS)
            programmeItems[id].error_code = response.error_code.val;
          if (programmeItems.length > startID+1)
            activateStatement(startID + 1);
          else {
            programmeItems[id].error_code = response.error_code.val;
          }
        },
        function(response) {
          var id = getActiveId();
          programmeItems[id].error_code = ErrorCode.PLANNING.FAILURE;
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

  // call init in contructor
  initialize();

  // exposed inner functions
  return {
    deleteStatement: deleteStatement,
    newWaypoint: newWaypoint,
    newWait: newWait,
    newComment: newComment,
    rename: rename,
    up: up,
    down: down,

    getStatementIDByPoseUID: getStatementIDByPoseUID,
    activateStatement: activateStatement,

    setCartesicPath: setCartesicPath,
    setCollisionCheck: setCollisionCheck,
    setImprovedPath: setImprovedPath,
    assignActivePose: assignActivePose,

    setWaitingSeconds: setWaitingSeconds,
    setConfirmationText: setConfirmationText,
    setComment: setComment,

    setWaitTypeWaitForSeconds: setWaitTypeWaitForSeconds,
    setWaitTypeNoWait: setWaitTypeNoWait,
    setWaitTypeWaitForConfirmation: setWaitTypeWaitForConfirmation,

    visualizeLocalPlan : visualizeLocalPlan,
    visualizeGlobalPlan : visualizeGlobalPlan,

    modifyPose : modifyPose,                          // called when a pose has been modified, updates the database and the plan

    forward: forward,
    run: run,
    refresh: refresh
  };
};