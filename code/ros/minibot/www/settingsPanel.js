                 // delete the old robotview (unless it is the first call)
/**
 * @author Jochen Alt
 */
var SettingsPanel = SettingsPanel || {};


SettingsPanel.Init = function(options) {
	options = options || {};
	var ros = options.ros;
	var configuration;
	var kinematicsPanel;
	var poseStorePanel;
	var programmePanel;
	var setGlobalThemeFunc;

	var initialize = function() {
		// patch all the themes into the DOM
		document.getElementById("themeSlider").setAttribute("max",Constants.Themes.Name.length-1);
		for (var idx = 0;idx < Constants.Themes.Name.length;idx++) {
			var name = Constants.Themes.Name[idx];
			var uppperName = name[0].toUpperCase() + name.substring(1);
			var themeScale = document.getElementById("themeScale");
			var label = document.createElement("DIV");
    		label.setAttribute('class', 'my-3 m-0 p-0');
    		label.innerHTML = uppperName;
    		label.style.transform = 'rotate(80deg)';
    		label.style.width =100/Constants.Themes.Name.length + '%';

    		// label.style.width="100%"; // Math.floor(w/Constants.Themes.Name.length) + "px";
    		themeScale.appendChild(label);
		}
	}

	var readConfiguration = function(callbackSuccess, callbackFailure) {
		var request = new ROSLIB.ServiceRequest({
			type: Constants.Database.READ_CONFIGURATIION
		});

		var readConfiguration = new ROSLIB.Service({
			ros: ros,
			name: '/database'
		});

		readConfiguration.callService(request,
			function(result) {
				if (result.error_code.val == Constants.ErrorCodes.SUCCESS) {
					configuration = result.configuration;
					callbackSuccess(result.configuration);
				} else {
					callbackFailure(result.configuration);
				}
			},
			function(result) {
				callbackFailure(result);
			});
	};

	var writeConfiguration = function(callbackSuccess, callbackFailure) {
		var request = new ROSLIB.ServiceRequest({
			type: Constants.Database.WRITE_CONFIGURATIION,
			configuration: configuration
		});

		var readConfiguration = new ROSLIB.Service({
			ros: ros,
			name: '/database'
		});

		readConfiguration.callService(request,
			function(result) {
				if (result.error_code.val == Constants.ErrorCodes.SUCCESS) {
					callbackSuccess(result);
				} else {
					callbackFailure(result);
				}
			},
			function(result) {
				callbackFailure(result);
			});
	};

	var setRadUnit = function() {
		configuration.angle_unit = Constants.Database.ANGLE_UNIT_RAD;
		Utils.switchAngleUnit(false);
		kinematicsPanel.switchAngleUnit(Utils.currentAngleUnitIsGrad());
	}

	var setGradUnit = function() {
		configuration.angle_unit = Constants.Database.ANGLE_UNIT_GRAD;
		Utils.switchAngleUnit(true);
		kinematicsPanel.switchAngleUnit(Utils.currentAngleUnitIsGrad());
	}

	var apply = function() {
		setTheme(configuration.theme);
	}

	

	var factoryReset = function() {
		if (configuration == null) {
			displayErr("Did not read configuration");
			return;
		}
		configuration.theme = "cyborg";
		configuration.angle_unit = Constants.Database.ANGLE_UNIT_RAD;
		apply();
	}

	var save = function() {
		writeConfiguration(function(result) {
				displayInfo("Settings have been saved");
			},
			function(error) {
				displayErr("Settings have not been saved due to " + error);
			})
	}

	var delayedSave = function() {
		Utils.callDelay ("configuration", 5000, function() {
			save();
		})
	}

	var setKinematicsPanel = function(panel) {
		kinematicsPanel = panel;
	}

	var setPoseStorePanel = function(panel) {
		poseStorePanel = panel;
	}

	var setProgrammePanel = function(panel) {
		programmePanel = panel;
	}

	var setThemeWidget = function() {
		var foundit = false;
		for (var idx = 0;idx < Constants.Themes.Name.length;idx++) {
			var name = Constants.Themes.Name[idx];
			var uppperName = name[0].toUpperCase() + name.substring(1);
			if (Constants.Themes.Name[idx] == configuration.theme) {
				document.getElementById("themeDescription").innerHTML = uppperName+ "<br><small>" + Constants.Themes.Description[idx];
				document.getElementById("themeSlider").value = idx;
				configuration.theme = Constants.Themes.Name[idx];
				foundit = true;
			}
		}
		if (!foundit)
			displayErr("could not find theme " + configuration.theme);
	}

	var setWidgets = function() {
		if (configuration.angle_unit == Constants.Database.ANGLE_UNIT_RAD) {
			document.getElementById("radUnit").checked = true;
			setRadUnit();
		} else {
			document.getElementById("gradUnit").checked = true;
			setGradUnit();
		}
		document.getElementById("saveAfterSeconds").value = parseInt(configuration.save_after.secs);
		setThemeWidget();
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

	var setGlobalThemeFunction = function(func) {
		setGlobalThemeFunc = func;
	}

	var callSetGlobalTheme = function () {
		setGlobalThemeFunc(configuration.theme);
	}

	var changeTheme = function(event) {
		var themeIdx = parseInt(event.target.value);
		configuration.theme =Constants.Themes.Name[themeIdx];
		setThemeWidget();
		Utils.callDelay ("theme", 1000, function() {
			callSetGlobalTheme();
			delayedSave();
		})
	}

	var getVisualizationLocalPlan = function() {
		return configuration.vis_local_plan;
	} 

	var setVisualizationLocalPlan = function(jfdi) {
		configuration.vis_local_plan = jfdi;
		save();
	} 

	var getVisualizationGlobalPlan = function() {
		return configuration.vis_global_plan;
	} 

	var setVisualizationGlobalPlan = function(jfdi) {
		configuration.vis_global_plan = jfdi;
		save();
	} 

	var setSaveAfterSeconds = function (event) {
		configuration.save_after.secs = parseInt(Utils.makeFloatString(event.target.value));
		configuration.save_after.nsecs = 0;
		delayedSave();
	}

	var getSaveAfterSeconds = function () {
		return configuration.save_after.secs;
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
		readConfiguration: readConfiguration,
		writeConfiguration: writeConfiguration,
		factoryReset: factoryReset,
		apply: apply,
		save: save,

		setKinematicsPanel: setKinematicsPanel,
		setPoseStorePanel: setPoseStorePanel,
		setProgrammePanel: setProgrammePanel,

		setRadUnit: setRadUnit,
		setGradUnit: setGradUnit,

		setWidgets: setWidgets,
		initialize : initialize,

		setGlobalThemeFunction : setGlobalThemeFunction,
		changeTheme : changeTheme,
		callSetGlobalTheme : callSetGlobalTheme,
		
		getVisualizationGlobalPlan : getVisualizationGlobalPlan,
		getVisualizationLocalPlan : getVisualizationLocalPlan,
		setVisualizationGlobalPlan : setVisualizationGlobalPlan,
		setVisualizationLocalPlan : setVisualizationLocalPlan,

		setSaveAfterSeconds :setSaveAfterSeconds,
		getSaveAfterSeconds : getSaveAfterSeconds
	};
};