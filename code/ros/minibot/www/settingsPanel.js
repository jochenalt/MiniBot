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
				if (result.error_code.val == ErrorCode.MOVEIT.SUCCESS) {
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

	var writeConfiguration = function(config, callbackSuccess, callbackFailure) {
		var request = new ROSLIB.ServiceRequest({
			type: Constants.Database.WRITE_CONFIGURATIION,
			configuration: config
		});

		var readConfiguration = new ROSLIB.Service({
			ros: ros,
			name: '/database'
		});

		readConfiguration.callService(request,
			function(result) {
				if (result.error_code.val == ErrorCode.MOVEIT.SUCCESS) {
					configuration = config;
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

	var initialize = function() {
		setThemeWidget();
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
		writeConfiguration(configuration, function(result) {
				displayInfo("Settings have been saved");
			},
			function(error) {
				displayErr("Settings have not been saved due to " + error);
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
		for (var idx = 0;idx < Constants.Themes.Name.length;idx++) {
			if (Constants.Themes.Name[idx] == configuration.theme) {
				var name = Constants.Themes.Name[idx];
				var uppperName = name[0].toUpperCase() + name.substring(1);
				document.getElementById("themeDescription").innerHTML = uppperName+ "<br><small>" + Constants.Themes.Description[idx];
				document.getElementById("themeSlider").value = idx;
				configuration.theme = Constants.Themes.Name[idx];
				return;
			}
		}
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
		})
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
		changeTheme : changeTheme
	};
};