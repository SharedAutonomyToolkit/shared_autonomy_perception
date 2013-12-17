var GRABCUTSEGMENTATIONLIB = GRABCUTSEGMENTATIONLIB || {
  REVISION : '1'
};

/**
 * @author Sarah Osentoski - sarah.osentoski@us.bosch.com
 */

/**
 * Based on code from Trevor Jay in the original rosbridge
 *
 * Takes raw ROS Image from rosbridge and places it it on a given canvas
 * Note this is extremely inefficient for streaming images and is only meant to transfer one image
 * This should NEVER be used for streaming video.  Use the mjpeg_server for this.  
 */


/*global $:false */

GRABCUTSEGMENTATIONLIB.ImageViewer = function(options){
    // needed for passing `this` into nested functions
    var that = this;
    console.log(options);
    options = options || {};

    this.stage = options.stage;
    console.log(this.stage);

    this.bitmap = null;

    this.src = null;
    this.srcContext = null;
    this.srcWidth = 0;
    this.srcHeight = 0;
    this.srcData = null;
    this.srcPixels = null;

};


GRABCUTSEGMENTATIONLIB.ImageViewer.prototype.__proto__ = EventEmitter2.prototype;

/**
 * Update the canvas with the most recent ROS Image 
 *
 * @param img - a sensor_msg/Image message.  Currently only accepts rgb8 and bgr8 encodings
 */
GRABCUTSEGMENTATIONLIB.ImageViewer.prototype.updateDisplay = function(img) {
    console.log("update display");

    if(this.bitmap != null) {
        this.stage.removeChild(this.bitmap);
    }

    if (img.encoding != 'rgb8' && img.encoding != 'bgr8' ) {
        console.log('unrecognized image encoding!');
        console.log(img.encoding);
        return;
    }
    
    //get the image data
    var imgWidth = img.width;
    var imgHeight = img.height;
    var imgPixels = window.atob(img.data);
    
    var imgLen = imgPixels.length;

    //Set up the canvas first; we'll copy to a never-seen div before updating all at once?
    if (this.src == null ||
	    this.srcWidth != imgWidth ||
	    this.srcHeight != imgHeight) {

	    var div = document.createElement('div');
	    div.innerHTML = '<canvas width="'+ imgWidth + '" height="' + imgHeight + '"></canvas>';

	    this.src = div.firstChild;
	    this.srcContext = this.src.getContext('2d');
	    this.srcWidth = imgWidth;
	    this.srcHeight = imgHeight;
	    this.srcData = this.srcContext.getImageData(0,0,this.srcWidth,this.srcHeight);
	    this.srcPixels = this.srcData.data; 
	    console.log('created new canvas');
    }

    //Set up the images
    var i = 0; // index into ROS image
    var j = 0; // index into JS image (rgba?)
    if( img.encoding == 'rgb8'){
	    while(i < imgLen) {
	        for (var k = 0; k < 3; k++) {
		        this.srcPixels[j+k] = imgPixels.charCodeAt(i+k);
	        }
	        this.srcPixels[j+3] = 255;
	        
	        i += 3;
	        j += 4;
	    }
    }
    else { // bgr8?
	    while( i < imgLen) {

	        this.srcPixels[j+0] = imgPixels.charCodeAt(i+2);
	        this.srcPixels[j+1] = imgPixels.charCodeAt(i+1);
	        this.srcPixels[j+2] = imgPixels.charCodeAt(i+0);

	        this.srcPixels[j+3] = 255;
	        
	        i += 3;
	        j += 4;
        }
	}

    //Copy

    this.srcContext.putImageData(this.srcData,0,0);
    this.bitmap = new createjs.Bitmap(this.src);
    this.stage.addChildAt(this.bitmap, 0);
    this.stage.update();
};
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

    this.stage = options.stage;
    this.width = options.width;
    this.height = options.height;

    // use requestAnimationFrame if it exists
    var requestAnimationFrame = window.requestAnimationFrame || window.webkitRequestAnimationFrame
        || window.mozRequestAnimationFrame || window.oRequestAnimationFrame
        || window.msRequestAnimationFrame || function(callback) {
            setInterval(callback, 100);
        };

    // vars used by the mouseEventHandler
    // TODO: I'm not sure when to use this./that. vs just 'var'...
    var mouseDown = false;
    var firstClick = null;
    var position = null;
    var positonVec3 = null;
    this.bounds = null;
    this.rect = new createjs.Shape();
    this.stage.addChild(this.rect);

    /** 
     * Updates and redraws the current bounding-box for every mouse event.
     * Modifies that.bounds to save the coordinates
     */
    var mouseEventHandler = function (event, mouseState){
	    if( mouseState == 'down'){
            //if mouse is pushed down get the position and save it
	        console.log('mouse down');
            console.log(that.stage);
	        mouseDown = true;

	        firstClick = { x: event.stageX, y:  event.stageY};

            // TODO: these aren't used?
	        position = that.stage.globalToRos(event.stageX, event.stageY);
            positionVec3 = new ROSLIB.Vector3(position);

            // remove previous rectangle
            that.rect.graphics.clear();
            that.stage.update();
            that.bounds = null;
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

	            that.rect.graphics.clear().beginStroke("#F00").drawRect(squareStart.x, squareStart.y, distancex, distancey);
	            that.stage.update();
	        }
        }
        else { //mouseState === 'up'
	        //if mouse button is up, stop updating square on mouse move
	        mouseDown = false;
        }
    }; // end of mouseEventHandler


    //set up callbacks for the stage
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
    this.rect.graphics.clear();
    return this.bounds;
}

GRABCUTSEGMENTATIONLIB.BoundingBox.prototype.__proto__ = EventEmitter2.prototype;

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
/**
 * @author Sarah Osentoski - sarah.osentoski@us.bosch.com
 */

/**
 * Manage drawing the interactive segmentation input and interaction with ROS
 */


/*global $:false */


/* This sets up two different SimpleActionServers, one that has the user
 * input a bounding box, and one that get individual pixel labels 
 */
GRABCUTSEGMENTATIONLIB.Segmenter = function(options){
    var that = this;

    options = options || {};
    var ros = options.ros;
    var host = options.host || 'localhost';
    var bboxTopic = options.bboxTopic;
    var bboxService = options.bboxService;
    var editTopic = options.editTopic;
    var editService = options.editService;


    this.bboxDiv = $('#' + options.bboxDiv);
    this.editDiv = $('#' + options.editDiv);
    var canvasWidth = options.canvasWidth;
    var canvasHeight = options.canvasHeight;

    // TODO: These names kinda suck.
    // needs to match the divs declared in interactive_segmentation_interface.html
    var bboxDivID = 'grabcut-bbox-canvas';
    var editDivID = 'grabcut-edit-canvas';

    var bboxCanvasID = 'grabcut-bbox-canvas-display';
    var editCanvasID = 'grabcut-edit-canvas-display';

    //add canvas and buttons to the window
    this.bboxDiv.dialog({
	    autoOpen : false,
	    width : canvasWidth,
	    height : canvasHeight
    });
    
    this.editDiv.dialog({
	    autoOpen : false,
	    width : canvasWidth,
	    height : canvasHeight
    });

    this.bboxDiv.html('<div id="' + bboxDivID + '"><canvas id ="' + bboxCanvasID + '" width = "' + canvasWidth +'" height="' + canvasHeight + '"> </canvas><\/div> <br> <br><button id="grabcut-bbox">Segment</button> <button id="grabcut-reset">Reset</button>');
    this.editDiv.html('<div id="' + editDivID + '"><canvas id ="' + editCanvasID + '" width = "' + canvasWidth +'" height="' + canvasHeight + '"> </canvas><\/div> <br> <br><button id="grabcut-edit">Segment</button> <button id="edit-foreground">Edit Foreground</button> <button id="edit-background">EditBackground</button>');

    // TODO: OK, I'm officially confused by "var" vs "this." vs "" for vars... AND WHAT'S THIS "NEW"?
    var bboxCanvas = document.getElementById(bboxCanvasID);
    bboxStage = new createjs.Stage(bboxCanvas);

    var bboxViewer = new GRABCUTSEGMENTATIONLIB.BoundingBox({
        stage : bboxStage,
    	width : canvasWidth,
    	height : canvasHeight
    });

    var bboxImageViewer= new GRABCUTSEGMENTATIONLIB.ImageViewer({
        stage : bboxStage
    });

    var editCanvas = document.getElementById(editCanvasID);
    editStage = new createjs.Stage(editCanvas);

    
    var editViewer = new GRABCUTSEGMENTATIONLIB.PixelEditor({
        stage : editStage, 
    	width : canvasWidth,
    	height : canvasHeight
    });

    var editImageViewer = new GRABCUTSEGMENTATIONLIB.ImageViewer({
        stage : editStage
    });

    this.bboxServer = new ROSLIB.SimpleActionServer({
	    ros : ros,
	    serverName : bboxService,
	    actionName : 'shared_autonomy_msgs/BoundingBoxAction'
    });

    this.editServer = new ROSLIB.SimpleActionServer({
	    ros : ros,
	    serverName : editService,
	    actionName : 'shared_autonomy_msgs/EditPixelAction'
    });

    // TODo; eventually, we hope to be able to pass image from this into 
    // the BoundingBox, but for now, we have to assume that it'll have
    // received an image as well ...
    this.bboxServer.on('goal', function(goalMessage) {
	    console.log('bbox service call');
        console.log(goalMessage);
        bboxImageViewer.updateDisplay(goalMessage.image);
	    that.bboxDiv.dialog("open");
    });
    this.editServer.on('goal', function(goalMessage) {
	    console.log('edit service call');
        console.log(goalMessage);
	console.log(editStage);
        editImageViewer.updateDisplay(goalMessage.image);
        editViewer.displayMask(goalMessage.mask);
	    that.editDiv.dialog("open");
    });
    
    //setup bbox button callbacks
    $('#grabcut-bbox')
	    .button()
	    .click(function(event){
	        console.log("clicked segmentation button - testing");
	        // TODO: Do I need logic that makes sure that we have
	        // valid bounds? what should happen if they're bad?
            var bounds = bboxViewer.getbounds();

            var result = {
                min_row : {data : Math.round(bounds.y)},
                max_row : {data : Math.round(bounds.y + bounds.dy)},
		        min_col : {data : Math.round(bounds.x)},
                max_col : {data : Math.round(bounds.x + bounds.dx)}
            };

	        that.bboxServer.setSucceeded(result);
            console.log("... set succeeded with: ");
            console.log(result);
	        that.bboxDiv.dialog("close");
            // TODO: need to remove the stage's 0-th child
	    });


    //setup edit button callbacks
    $('#grabcut-edit')
	    .button()
	    .click(function(event){
	        console.log("clicked segmentation button - testing");
	        // TODO: Do I need logic that makes sure that we have
	        // valid bounds? what should happen if they're bad?
            var result = editViewer.getlabels();

	        that.editServer.setSucceeded(result);
            console.log("... set succeeded with: ");
            console.log(result);
	        that.editDiv.dialog("close");
	    });

    $('#edit-foreground')
        .button()
        .click(function(event) {
            console.log("Now editing FG pixels");
            editViewer.setForeground();
        });
    $('#edit-background')
        .button()
        .click(function(event) {
            console.log("Now editing BG pixels");
            editViewer.setBackground();
        });

};
