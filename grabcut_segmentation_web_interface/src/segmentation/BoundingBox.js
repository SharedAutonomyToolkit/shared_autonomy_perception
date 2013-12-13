/**
 * @author Sarah Osentoski - sarah.osentoski@us.bosch.com
 */

/**
 * Manage drawing the image and drawing of the selection rectangle on the canvas
 */


/*global $:false */

GRABCUTSEGMENTATIONLIB.BoundingBox = function(options){
    // needed for passing `this` into nested functions
    var that = this;

    var divID = options.divID;
    var canvasID = options.canvasID;
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

    //get the canvas
    this.canvas = document.getElementById(canvasID);

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
    var mouseDown = false;
    var firstClick = null;
    var position = null;
    var positonVec3 = null;
    var rect = null;
    this.bounds = null;

    /** 
     * Updates and redraws the current bounding-box for every mouse event.
     * Modifies that.bounds to save the coordinates
     */
    var mouseEventHandler = function (event, mouseState){
	if( mouseState == 'down'){
            //if mouse is pushed down get the position and save it
	    console.log('mouse down');
	    mouseDown = true;

	    firstClick = { x: event.stageX, y:  event.stageY};

            // TODO: these aren't used?
	    position = that.stage.globalToRos(event.stageX, event.stageY);
            positionVec3 = new ROSLIB.Vector3(position);
            
	    //remove previous rectangle
            if (rect){
                console.log('removing rect')
		that.stage.removeChild(rect);
		rect = null;
                that.stage.update();
                that.bounds = null;
	    }
	}
        else if (mouseState === 'move') {
            if (mouseDown === true) {
	        //if mouse button is being held down:
	        //get the current mouse position
	        //calculate distance from start position
	        
	        var currentClick={x: event.stageX, y: event.stageY};

                // TODO: these vars are currently unused. 
                // Sarah says that this magically maps coordinates to ROS coordinates =)
	        var currentPos = that.stage.globalToRos(event.stageX, event.stageY);
	        currentPosVec3 = new ROSLIB.Vector3(currentPos);
		
	        //calculate positions and rectangle information
	        var squareStart = {x: firstClick.x, y: firstClick.y};
	        if(firstClick.x > currentClick.x) {
		    squareStart.x = currentClick.x;
	        }
	        if(firstClick.y > currentClick.y) {
    		    squareStart.y = currentClick.y;
	        }
	        var distancex = Math.abs(firstClick.x - currentClick.x);
	        var distancey = Math.abs(firstClick.y - currentClick.y);

                that.bounds = {x:squareStart.x, y:squareStart.y, dx:distancex, dy:distancey};

	        //remove old rec so we can draw a new one
		// TODO: is this required? or if I leave rect as var, can I just update it??
	        if(rect) {
		    that.stage.removeChild(rect);
		    rect = null;
	        }
                // and, draw new rectangle
	        rect = new createjs.Shape();
	        rect.graphics.beginStroke("#F00");
	        rect.graphics.drawRect(squareStart.x, squareStart.y, distancex, distancey);
	        that.stage.addChild(rect);
	        that.stage.update();
	    }
        }
        else { //mouseState === 'up'
	    //if mouse button is up, stop updating square on mouse move
	    mouseDown = false;
        }
    }; // end of mouseEventHandler


    //set up callbacks for the canvas
    this.stage.addEventListener('stagemousedown', function(event) {
        mouseEventHandler(event,'down');
    });

    this.stage.addEventListener('stagemousemove', function(event) {
        mouseEventHandler(event,'move');
    });

    this.stage.addEventListener('stagemouseup', function(event) {
        mouseEventHandler(event,'up');
    });
};


GRABCUTSEGMENTATIONLIB.BoundingBox.prototype.getbounds = function() {
    return this.bounds;
}

GRABCUTSEGMENTATIONLIB.BoundingBox.prototype.__proto__ = EventEmitter2.prototype;

/**
 * Change the stream of this canvas to the given topic.
 *
 * @param topic - the topic to stream, like '/wide_stereo/left/image_color'
 */
GRABCUTSEGMENTATIONLIB.BoundingBox.prototype.changeStream = function(topic) {
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