/**
 * @author Sarah Osentoski - sarah.osentoski@us.bosch.com
 */

/**
 * Manage drawing of current segmentation of code and handle the selection of background and foreground points
 * 
 *
 * @constructor 
 * @param options - object with the following keys:
 *   * stage - an easeljs stage that helps manage the canvas: www.createjs.com   
 *   * width - width of the canvas                                               
 *   * height - height of the canvas 
 *
 */


GRABCUTSEGMENTATIONLIB.PixelEditor = function(options){
    var that = this;

    this.stage = options.stage;
    this.width = options.width;
    this.height = options.height;

    this.imageViewer = new GRABCUTSEGMENTATIONLIB.ImageViewer({
        stage : this.stage
    });

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
    
    //handle mouse events
    function mouseMoveEventHandler(event) {

	var currentClick={u: Math.round(event.stageX), v: Math.round(event.stageY)};
     	
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
	mouseDown = true;
        oldPt = new createjs.Point(event.stageX, event.stageY);
        oldMidPt = oldPt;
        that.stage.addEventListener('stagemousemove', mouseMoveEventHandler);
    }

    function  mouseUpEventHandler(event) {
	mouseDown = false;
        that.stage.removeEventListener('stagemousemove', mouseMoveEventHandler);
    }

    //set up callbacks for the canvas
    this.stage.addEventListener('stagemousedown', mouseDownEventHandler);
    this.stage.addEventListener('stagemouseup', mouseUpEventHandler);
};


/**                                                                         
 * Returns currently labeled pixels and removes the current points
 *
 */

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

/**
 * Overlay the given mask to the image
 *
 * @param mask - the sensor_msgs/Image mask to display (assumes mono8 encoding)
 */
GRABCUTSEGMENTATIONLIB.PixelEditor.prototype.displayMask = function(mask) {
    var backgroundAlpha = 180;
    var foregroundAlpha = 0;
    
    if (mask.encoding != 'mono8') {
	console.log('Mask has unrecognized image encoding!');
    }

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


/**
 * Sets the current goal of the PixelEditor viewer, updates display
 */
GRABCUTSEGMENTATIONLIB.PixelEditor.prototype.setGoal = function(goalMessage) {
    this.imageViewer.updateDisplay(goalMessage.image);
    this.displayMask(goalMessage.mask);
}


/**                                                                         
 * Sets pixel labels to foreground
 *
 */
GRABCUTSEGMENTATIONLIB.PixelEditor.prototype.setForeground = function() {
    this.label = 'foreground';
}

/**                                                                         
 * Sets pixel labels to background
 *
 */

GRABCUTSEGMENTATIONLIB.PixelEditor.prototype.setBackground = function() {
    this.label = 'background';
}