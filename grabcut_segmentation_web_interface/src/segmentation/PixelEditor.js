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

    // left drag FG, right drag BG
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

    this.label = 'none'
    // example from:
    // https://github.com/CreateJS/EaselJS/blob/master/examples/CurveTo.html
    // TODO: the curveTois probably overkill. just lineTo, maybe?
    var oldPt;
    var oldMidPt;
    function mouseMoveEventHandler(event) {
        console.log('mouse moved!');
	    var currentClick={u: Math.round(event.stageX), v: Math.round(event.stageY)};
        // TODO: these vars are currently unused. 
        // Sarah says that this magically maps coordinates to ROS coordinates =)
	    var currentPos = that.stage.globalToRos(event.stageX, event.stageY);
	    currentPosVec3 = new ROSLIB.Vector3(currentPos);
		
        var midPt = new createjs.Point(oldPt.x + event.stageX>>1, oldPt.y+event.stageY>>1);
        
        if(that.label === 'foreground') {
	        that.foreground.push(currentClick);
            that.fgLine.graphics.moveTo(midPt.x, midPt.y).curveTo(oldPt.x, oldPt.y, oldMidPt.x, oldMidPt.y);
        } else if(that.label === 'background') {
	        that.background.push(currentClick);
            that.bgLine.graphics.moveTo(midPt.x, midPt.y).curveTo(oldPt.x, oldPt.y, oldMidPt.x, oldMidPt.y);
        }
        oldPt.x = event.stageX;
        oldPt.y = event.stageY;
        
        oldMidPt.x = midPt.x;
        oldMidPt.y = midPt.y;
        
        that.stage.update();
    }

    function mouseDownEventHandler(event) {
	    console.log('down');
	    mouseDown = true;
        oldPt = new createjs.Point(event.stageX, event.stageY);
        oldMidPt = oldPt;
        that.stage.addEventListener('stagemousemove', mouseMoveEventHandler);
    }

    function  mouseUpEventHandler(event) {
        console.log('up');
	    mouseDown = false;
        that.stage.removeEventListener('stagemousemove', mouseMoveEventHandler);
    }

    //set up callbacks for the canvas
    this.stage.addEventListener('stagemousedown', mouseDownEventHandler);
    this.stage.addEventListener('stagemouseup', mouseUpEventHandler);
};


GRABCUTSEGMENTATIONLIB.PixelEditor.prototype.getlabels = function() {
    var result = {fg : this.foreground, bg : this.background};
    // reset state
    this.foreground = [];
    this.background = [];
    this.label = 'none';
    // removing the lines we've added, but leaving ready to draw again
    this.fgLine.graphics.clear();
    this.fgLine.graphics.beginStroke("#F00").setStrokeStyle(3,"round");
    this.bgLine.graphics.clear();
    this.bgLine.graphics.beginStroke("#00F").setStrokeStyle(3,"round");
    this.stage.update();
    
    return result;
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
    // TODO; what listens to this?
    this.emit('change', topic);

    //trying out bitmap easel thing
    var imagebitmap = new createjs.Bitmap(this.image);
    console.log('bitmap');
    this.stage.addChild(imagebitmap);

};

GRABCUTSEGMENTATIONLIB.PixelEditor.prototype.setForeground = function() {
    this.label = 'foreground';
}
GRABCUTSEGMENTATIONLIB.PixelEditor.prototype.setBackground = function() {
    this.label = 'background';
}