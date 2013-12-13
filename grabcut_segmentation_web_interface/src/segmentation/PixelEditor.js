/**
 * @author Sarah Osentoski - sarah.osentoski@us.bosch.com
 */

/**
 * Manage drawing the image and drawing of the selection rectangle on the canvas
 */


/*global $:false */

GRABCUTSEGMENTATIONLIB.PixelEditor = function(options){
    // needed for passing `this` into nested functions
    var that = this;

    var divID = options.divID;
    this.width = options.width;
    this.height = options.height;
    this.host = options.host;
    this.port = options.port || 8080;
    this.quality = options.quality;
    var topic = options.topic;
    var overlay = options.overlay;

    // create no image initially
    this.image = new Image();

    // used if there was an error loading the stream
    var errorIcon = new MJPEGCANVAS.ErrorIcon();

    // create the canvas to render to
    this.canvas = document.createElement('canvas');
    this.canvas.width = this.width;
    this.canvas.height = this.height;

    document.getElementById(divID).appendChild(this.canvas);

    this.stage = new createjs.Stage(this.canvas);
    //console.log(this.canvas);
    //console.log(this.stage);
    var context = this.canvas.getContext('2d');

    this.canvas.style.background = '#aaaaaa';

    // use requestAnimationFrame if it exists
    var requestAnimationFrame = window.requestAnimationFrame || window.webkitRequestAnimationFrame
        || window.mozRequestAnimationFrame || window.oRequestAnimationFrame
        || window.msRequestAnimationFrame || function(callback) {
            setInterval(callback, 100);
        };

    // grab the initial stream
    this.changeStream(topic);

    // vars used by the mouseEventHandler
    // TODO: I'm not sure when to use this./that. vs just 'var'...
    var rightMouseDown = false;
    var leftMouseDown = false;
    var firstClick = null;
    var position = null;
    var positonVec3 = null;
    var rect = null;

    // left drag FG, right drag BG
    // TODO: how to handle arrays?
    this.foreground = [];
    this.background = [];

    // and, the display objects to show FG/BG.
    // add them to stage here so can always remove in callback.
    this.fgLine = new createjs.Shape();
    this.fgLine.graphics.beginStroke("#F00").setStrokeStyle(3,"round");
    this.stage.addChild(this.fgLine);
    this.bgLine = new createjs.Shape();
    this.bgLine.graphics.beginStroke("#00F").setStrokeStyle(3,"round");
    this.stage.addChild(this.bgLine);
    // example from:
    // https://github.com/CreateJS/EaselJS/blob/master/examples/CurveTo.html
    var oldPt;
    var oldMidPt;
    /** 
     * Updates and redraws the current bounding-box for every mouse event.
     * Modifies that.bounds to save the coordinates
     */
    var mouseEventHandler = function (event, mouseEvent){
	// any click resets the current list of points that needs to be drawn
	// releasing the mouse button appends the list to the correct FG/BG
	// TODO; it's ugly how this is in parallel for left/right clicks
	// TODO: I liked the pattern that used the down click to create a callback
	// for mousemove, and the mouseup to remove tha tcallback
	if( mouseEvent === 'leftdown') {
	    console.log('left down');
	    rightMouseDown = false;
	    leftMouseDown = true;
            oldPt = new createjs.Point(event.stageX, event.stageY);
            oldMidPt = oldPt;
	} else if (mouseEvent === 'rightdown') {
            //if mouse is pushed down get the position and save it
	    console.log(mouseEvent);
	    rightMouseDown = true;
	    leftMouseDown = false;
            oldPt = new createjs.Point(event.stageX, event.stageY);
            oldMidPt = oldPt;
	} else if ((mouseEvent === 'leftmove') && (leftMouseDown === true)) {
	    var currentClick={x: event.stageX, y: event.stageY};
            // TODO: these vars are currently unused. 
            // Sarah says that this magically maps coordinates to ROS coordinates =)
	    var currentPos = that.stage.globalToRos(event.stageX, event.stageY);
	    currentPosVec3 = new ROSLIB.Vector3(currentPos);
		
	    that.foreground.push(currentClick);

	    // TODO; figure out how to draw a list of points!
            var midPt = new createjs.Point(oldPt.x + event.stageX>>1, oldPt.y+event.stageY>>1);

	    // TODO: this is probably overkill. just do lineTo?
            that.fgLine.graphics.moveTo(midPt.x, midPt.y).curveTo(oldPt.x, oldPt.y, oldMidPt.x, oldMidPt.y);

            oldPt.x = event.stageX;
            oldPt.y = event.stageY;

            oldMidPt.x = midPt.x;
            oldMidPt.y = midPt.y;

            that.stage.update();

	} else if((mouseEvent === 'rightmove') && (rightMouseDown === true)) {
	    console.log('right mouse drag NYI');
        } else if(mouseEvent === 'leftup' && leftMouseDown) {
	    leftMouseDown = false;
	    console.log(that.foreground);
	} else if(mouseEvent === 'rightup' && rightMouseDown) {
	    rightMouseDown = false;
	    console.log(that.background);
        } else {
	    // (or mouseup) incompatible combination of event and mouse state', mouseEvent, leftMouseDown, rightMouseDown
	}
    }; // end of mouseEventHandler


    //set up callbacks for the canvas
    this.stage.addEventListener('stagemousedown', function(event) {
	if(event.nativeEvent.button === 2) {
            mouseEventHandler(event,'rightdown');
	} else {
	    mouseEventHandler(event,'leftdown');
	}	
    });

    this.stage.addEventListener('stagemousemove', function(event) {
        mouseEventHandler(event,'move');
	if(event.nativeEvent.button === 2) {
            mouseEventHandler(event,'rightmove');
	} else {
	    mouseEventHandler(event,'leftmove');
	}
    });

    this.stage.addEventListener('stagemouseup', function(event) {
	if(event.nativeEvent.button === 2) {
            mouseEventHandler(event,'rightup');
	} else {
            mouseEventHandler(event,'leftup');
	}
    });
};


GRABCUTSEGMENTATIONLIB.PixelEditor.prototype.getbounds = function() {
    return this.bounds;
}

GRABCUTSEGMENTATIONLIB.PixelEditor.prototype.__proto__ = EventEmitter2.prototype;

/**
 * Change the stream of this canvas to the given topic.
 *
 * @param topic - the topic to stream, like '/wide_stereo/left/image_color'
 */
GRABCUTSEGMENTATIONLIB.PixelEditor.prototype.changeStream = function(topic) {
    this.image = new Image();
    // create the image to hold the stream
    var src = 'http://' + this.host + ':' + this.port + '/snapshot?topic=' + topic;
    // add various options
    src += '?width=' + this.width;
    src += '?height=' + this.height;
    if (this.quality > 0) {
        src += '?quality=' + this.quality;
    }
    this.image.src = src;
    // emit an event for the change
    this.emit('change', topic);

    //trying out bitmap easel thing
    var imagebitmap = new createjs.Bitmap(this.image);
    console.log('bitmap');
    this.stage.addChild(imagebitmap);
};