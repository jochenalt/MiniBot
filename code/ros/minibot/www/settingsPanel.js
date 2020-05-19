/**
 * @author Jochen Alt
 */


var SettingsPanel = SettingsPanel || {
};


SettingsPanel.Init = function(options) {
  options = options || {};
  var ros = options.ros;
  var configuration;


  var readConfiguration = function(callbackSuccess, callbackFailure) {
    var request = new ROSLIB.ServiceRequest({
     	type : Constants.Database.READ_CONFIGURATIION
    });

    var readConfiguration = new ROSLIB.Service({
        ros : ros,
        name : '/database'
    });

    readConfiguration.callService(request, 
      function(result) {
        if (result.error_code.val == ErrorCode.MOVEIT.SUCCESS) {
        	configuration = result.configuration;
			callbackSuccess(result.configuration);
        }
        else {
			callbackFailure(result.configuration);
        }
      }, 
      function (result) {
			callbackFailure(result);
      });
  };

  var writeConfiguration = function(config, callbackSuccess, callbackFailure) {
    var request = new ROSLIB.ServiceRequest({
     type : Constants.Database.WRITE_CONFIGURATIION,
     configuration : config
    });

    var readConfiguration = new ROSLIB.Service({
        ros : ros,
        name : '/database'
    });

    readConfiguration.callService(request, 
      function(result) {
        if (result.error_code.val == ErrorCode.MOVEIT.SUCCESS) {
        	configuration = config;
			callbackSuccess(result);
        }
        else {
			callbackFailure(result);
        }
      }, 
      function (result) {
			callbackFailure(result);
      });
  };


  var loadSettings = function() {

  }
  var saveSettings = function() {

  }

  var apply = function() {
  	setTheme(configuration.theme);
  } 

  var factoryReset = function() {
  	if (configuration == null) {
  		displayErr("Did not read configuration");
  		return;
  	}
  	configuration.theme = "cyborgTheme";
  	apply();
  }

  var save = function() {
  	writeConfiguration(configuration, function(result) {
  		displayInfo("Settings have been saved");
  	},
  	function(error) {
 		displayErr("Settings have not been saved due to " + error);
  	})	
  }

  var setTheme = function(theme) {
  	if (configuration == null) {
  		displayErr("Did not read configuration");
  		return;
  	}
  	configuration.theme = theme;
  	document.getElementById(theme).checked = true;
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
    displayAlert(t, document.getElementById('settingsAlertHeadline'), document.getElementById('settingsAlertSuccess'));
  }

  function displayErr(t) {
    displayAlert(t, document.getElementById('settingsAlertHeadline'), document.getElementById('settingsAlertError'));
  }

  function displayWarn(t) {
    displayAlert(t, document.getElementById('settingsAlertHeadline'), document.getElementById('settingsAlertWarning'));
  }

  // exposed inner functions
  return {
  	readConfiguration : readConfiguration,
  	writeConfiguration : writeConfiguration,
  	factoryReset : factoryReset,
  	apply : apply,
  	save : save,
  	setTheme : setTheme
  };
};
