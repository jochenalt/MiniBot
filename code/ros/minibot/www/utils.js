/**
 * @author Jochen Alt
 */


var Utils =  {};

var angleViewUnitIsGrad = 0; // 0 == rad, 1 = grad

Utils.assert = function(cond, msg) {
	if (!cond)
		alert(msg)
}

// throttle invokations of a function to a given frequency. 
// Invokations before the call is due are supressed. 
var  timerMeasurementDict = new Object();
Utils.measureTime =function (name, func) {
	var timeMeasurementObject = timerMeasurementDict[name];
	if (timeMeasurementObject == null) {
		timeMeasurementObject  = new Object();
		timerMeasurementDict[name] = timeMeasurementObject;
		timeMeasurementObject.avr = 0;
		timeMeasurementObject.dev = 0;
		timeMeasurementObject.lastLog = 0;
	} 
	var start = Date.now();
	func();
	var d = Date.now() - start;	
	timeMeasurementObject.avr = timeMeasurementObject.avr*0.5 + d*0.5;
	timeMeasurementObject.dev = timeMeasurementObject.dev*0.5 + 0.5*Math.abs(d - timeMeasurementObject.avr);
	if (start - timeMeasurementObject.lastLog > 1000) {
		console.log("time of " + name + " avr=" + timeMeasurementObject.avr + "ms dev=" + timeMeasurementObject.dev + "ms");
		timeMeasurementObject.lastLog = start;
	}
}

// throttle invokations of a function to a given frequency. 
// Invokations before the call is due are supressed. 
var  callThrottleDict = new Object();
Utils.callThrottler =function (name, rate, func, params) {
	var callFrequencyObject = callThrottleDict[name];
	if (callFrequencyObject == null) {
		callFrequencyObject  = new Object();
		callThrottleDict[name] = callFrequencyObject;
	} 
	var now = Date.now();
	var lastCall = callFrequencyObject.lastCall;

	// if the last call is long ago, invoke immediately
	var timeSinceLastCall = now - lastCall;
	var timeUntilNextCall = 1000/rate - timeSinceLastCall ;
	if (callFrequencyObject.lastCall == null || (timeUntilNextCall <= 1)) {
		// last call is a while ago, we can call the function immediately
		callFrequencyObject.lastCall = now;
		callFrequencyObject.setup = false;
		func(params);
	} else {
		// if timeout hasnt been set yet, do it
		callFrequencyObject.params = params;
		callFrequencyObject.func = func;
		if (!callFrequencyObject.setup) {
			callFrequencyObject.setup = true;
			setTimeout (function() {		
				callFrequencyObject.lastCall = Date.now();
				callThrottleDict.supressed = false;
				callFrequencyObject.func(callFrequencyObject.params);
				callFrequencyObject.setup = false;
			}, timeUntilNextCall);
		}
	}
}

var callDelayDict = new Object();
Utils.callDelay = function (name, duration, func) {
	var callDelayObject = callDelayDict[name];
	if (callDelayObject == null) {
		callDelayObject  = new Object();
		callDelayDict[name] = callDelayObject;
		callDelayObject.timeoutId = null;
	} 

    if (callDelayObject.timeoutId != null) {
      clearTimeout(callDelayObject.timeoutId );
    }
    callDelayObject.timeoutId = setTimeout(function() {
        callDelayObject.timeoutId  = null;      
        func();
    },duration);
}


var semaphorDict = new Object();
Utils.getMutex = function(name) {
	var mutex = semaphorDict[name];
	if (mutex == null) {
		mutex = new Object();
		semaphorDict[name] = mutex;
		mutex.lastDataObject = null;
		mutex.status = 0;
	}
	return mutex;
}	
Utils.stopMutex = function(name) {
	var mutex = Utils.getMutex(name);
	mutex.status = 1;
}

Utils.releaseMutex = function(name, func) {
	var mutex = Utils.getMutex(name);
 	if (mutex.status == 1) && (mutex.lastDataObject != null) {
        func (mutex.lastDataObject);
        mutex.status  = 0;
    } 
}

Utils.queueAtMutex = function (name,  func, data) {
	var mutex = Utils.getMutex(name);
    if (mutex.status == 0) {
    	mutex.func = func;
      	func(data);
      	mutex.lastDataObject = null;
    }
    else {
      mutex.lastDataObject = data;
    }
}


Utils.Sleep = function(milliseconds) {
   return new Promise(resolve => setTimeout(resolve, milliseconds));
}

Utils.rad2View = function(value) {
	if (angleViewUnitIsGrad)
		return Math.round(value*(180/Math.PI*100))/100;
	else
		return Math.round(value*1000)/1000;
}

Utils.view2Rad = function(value) {
	if (angleViewUnitIsGrad)
		return value*(Math.PI/180);
	else
		return value;
}

Utils.angleSteps = function() {
	if (angleViewUnitIsGrad)
		return 0.01;
	else
		return 0.001;
}

Utils.angleRound = function(value) {
	if (angleViewUnitIsGrad)
		return Math.round(value*100)/100;
	else
		return Math.round(value*1000)/1000;
}

Utils.distanceRound = function(value) {
	return Math.round(value*10)/10;
}


Utils.switchAngleUnit = function(toGrad) {
	angleViewUnitIsGrad = toGrad;
}

Utils.currentAngleUnitIsGrad = function() {
	return angleViewUnitIsGrad;
}
/** convert meters in rounded mm
 * @param value - object with following keys:
 *   * x, y, z - ros position in [m]
 * returns array of positions in [mm]
 */
Utils.position2View = function(value) {
	return  { x:Utils.distance2View(value.x), y:Utils.distance2View(value.y), z:Utils.distance2View(value.z)};
}

Utils.distance2View = function(value) {
	return Math.round(value*10000)/10;
}

Utils.view2Distance  = function(value) {
	return value/1000;
}


Utils.distanceSteps = function(value) {
	return 0.1;
}
/** convert quaternions into euler angels in rad/grad
 * and does a converson from base_link to a frame that's origin is towards the screen
 * @param value - object with following keys:
 *   * x, y, z, w - ros quaternion orientation 
 * returns array of euler orientation in [rad/grad]
 */

Utils.quaternion2Euler  = function(q) {

	var botFrame2ViewFrame = new THREE.Quaternion();
	botFrame2ViewFrame.setFromAxisAngle(new THREE.Vector3( 0, 1, 0 ), -Math.PI/2);
	var q1 = new THREE.Quaternion(q.x, q.y, q.z, q.w);
	q1.multiply(botFrame2ViewFrame);
	var euler = new THREE.Euler().setFromQuaternion( q1, "YXZ" ); 
	return { 	x:euler._x, 
				y:euler._y, 
				z:euler._z};
}

/**
 * the reverse
 */
Utils.euler2Quaternion = function(e) {
	var botFrame2ViewFrame = new THREE.Quaternion();
	botFrame2ViewFrame.setFromAxisAngle(new THREE.Vector3( 0, 1, 0 ), Math.PI/2);
	var e1 = new THREE.Euler(e.x, e.y, e.z, "YXZ");
	var q1 = new THREE.Quaternion().setFromEuler( e1 ); 
	q1.multiply(botFrame2ViewFrame);
	return { 	x:q1.x, 
		  		y:q1.y, 
		  		z:q1.z, 
		  		w:q1.w};
}

Utils.makeFloatString =function (str) {
	return str.replace(/[^\d.-]/g, ''); 
}


