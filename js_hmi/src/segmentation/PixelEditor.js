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

    this.stage = options.stage;
    this.width = options.width;
    this.height = options.height;

    this.bitmap = null;

    // use requestAnimationFrame if it exists
    var requestAnimationFrame = window.requestAnimationFrame || window.webkitRequestAnimationFrame
        || window.mozRequestAnimationFrame || window.oRequestAnimationFrame
        || window.msRequestAnimationFrame || function(callback) {
            setInterval(callback, 100);
        };

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

	    var currentClick={u: Math.round(event.stageX), v: Math.round(event.stageY)};
        // TODO: these vars are currently unused. 
        // Sarah says that this magically maps coordinates to ROS coordinates =)
	    var currentPos = that.stage.globalToRos(event.stageX, event.stageY);
	    currentPosVec3 = new ROSLIB.Vector3(currentPos);
		
        var midPt = new createjs.Point(oldPt.x + event.stageX>>1, oldPt.y+event.stageY>>1);
        
        if(that.label === 'foreground') {
            console.log('mouse moved! - foreground');
	        that.foreground.push(currentClick);
            that.fgLine.graphics.moveTo(midPt.x, midPt.y).curveTo(oldPt.x, oldPt.y, oldMidPt.x, oldMidPt.y);
        } else if(that.label === 'background') {
            console.log('mouse moved! - foreground');
	        that.background.push(currentClick);
            that.bgLine.graphics.moveTo(midPt.x, midPt.y).curveTo(oldPt.x, oldPt.y, oldMidPt.x, oldMidPt.y);
        } else {
            console.log('mouse moved! - no label');
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
        console.log('stage children: ', that.stage.getNumChildren());
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

GRABCUTSEGMENTATIONLIB.PixelEditor.prototype.displayMask = function(mask) {
    var backgroundAlpha = 180;
    var foregroundAlpha = 0;

    var maskData = window.atob(mask.data);

    //create a canvas to make a bitmap
    var tempCanvas = document.createElement("canvas");
    var tempCanvasContext = tempCanvas.getContext("2d");
    tempCanvas.width = this.width;
    tempCanvas.height = this.height;
    tempData = tempCanvasContext.getImageData(0,0,this.width, this.height);
    tempPixels = tempData.data;

    imageIndex = 0;

    // create mask that's black, and  entirely transparent in the FG, and adds a half-grey to BG pixels
    for (var maskIndex = 0; maskIndex < maskData.length; maskIndex++) {
            // make image black
	    for(var k = 0; k<3; k++) {
	        tempPixels[imageIndex+k]=0;
	    }
        // foreground
	    if(maskData.charCodeAt(maskIndex)==1 || maskData.charCodeAt(maskIndex)==3) {
		    alpha = foregroundAlpha;
	    }
	    else {
		    alpha = backgroundAlpha;
	    }
	    tempPixels[imageIndex+3]=alpha; 
	    imageIndex += 4;
    }

    console.log(tempPixels);
    //now copy the images. recall that tempPixels= tempData.data
    tempCanvasContext.putImageData(tempData,0,0);
    if(this.bitmap != null) {
        this.stage.removeChild(this.bitmap);
    }
    this.bitmap = new createjs.Bitmap(tempCanvas);
    // needs to be in front of the main image, behind everything else.
    this.stage.addChildAt(this.bitmap,1);
    this.stage.update();
}

GRABCUTSEGMENTATIONLIB.PixelEditor.prototype.__proto__ = EventEmitter2.prototype;

GRABCUTSEGMENTATIONLIB.PixelEditor.prototype.setForeground = function() {
    this.label = 'foreground';
}
GRABCUTSEGMENTATIONLIB.PixelEditor.prototype.setBackground = function() {
    this.label = 'background';
}