var GRABCUTSEGMENTATIONLIB = GRABCUTSEGMENTATIONLIB || {
  REVISION : '1'
};


/**
 * @author Sarah Osentoski - sarah.osentoski@us.bosch.com
 */

/**
 * Manage drawing the interactive segmentation input and interaction with ROS
 */


/*global $:false */

GRABCUTSEGMENTATIONLIB.Segmenter = function(options){
	var that = this;
	options = options || {};
	var ros = options.ros;
        var host = options.host || 'localhost';
	var cameraTopic = options.cameraTopic;
	var editTopic = options.editTopic;
	this.segmentationDiv = $('#' + options.segmentationDiv);
	this.editDiv = $('#' + options.editDiv);
	var canvasWidth = options.canvasWidth;
	var canvasHeight = options.canvasHeight;
	this.widgetState = 'segement';
	
	this.Lock = false;

	var segmentationCanvasId = 'grabcut-segmentation-canvas';
	var editCanvasId = 'grabcut-edit-canvas';

	//add canvas and buttons to the window

	this.segmentationDiv.dialog({
		autoOpen : false,
		width : canvasWidth,
		height : canvasHeight

	});
	
	this.editDiv.dialog({
		autoOpen : false
	});

	this.segmentationDiv.html('<div id="' + segmentationCanvasId + '"><\/div> <br> <br><button id="grabcut-segmentation">Segment</button> <button id="grabcut-reset">Reset</button>'); 	
	this.editDiv.html( '<div id="' + editCanvasId +  '"><\/div> <br><br><button id="grabcut-edit-segmentation">Send</button> <button id="grabcut-edit-reset">Reset</button>');
//	this.segmentationDiv.html('<canvas id="' + segmentationCanvasId + '" width="' + canvasWidth + '" height="' + canvasHeight + '"><\/canvas> <br> <br><button id="grabcut-segmentation">Segment</button> <button id="grabcut-reset">Reset</button>'); 
//	this.editDiv.html( '<canvas id="' + editCanvasId + '" width="' + canvasWidth + '" height="' + canvasHeight + '"><\/canvas> <br><br><button id="grabcut-edit-segmentation">Send</button> <button id="grabcut-edit-reset">Reset</button>');

	var segmentationViewer = new GRABCUTSEGMENTATIONLIB.Selector({
    	divID : segmentationCanvasId,
    	host : host,
    	width : canvasWidth,
    	height : canvasHeight,
    	topic : cameraTopic
  	});

    //todo change this to edit object
	var editViewer = new MJPEGCANVAS.Viewer({
    	divID : editCanvasId,
   		host : host,
    	width : canvasWidth,
    	height : canvasHeight,
    	topic : editTopic
  	});

	
	//set up  camera topic subscriber
	var cameraListener = new ROSLIB.Topic({
		ros : ros, 
		name : cameraTopic, 
		messageType : 'sensor_msgs/Image'
	});

	console.log(cameraListener);
	
	cameraListener.subscribe(function(message){
		that.widgetState = 'segment';
		console.log('cameramessage');

		//get canvas

//		var stage = new createjs.Stage();
		that.segmentationDiv.dialog("open");

	});


	//setup camera button callbacks

	$('#grabcut-segmentation')
		.button()
		.click(function(event){
			console.log("clicked segmentation button");
		});

	$('#grabcut-reset')
		.button()
		.click(function(even){
			console.log("clicked reset button");
		});

	//set up subscriber to edit topic
	var editListener = new ROSLIB.Topic({
		ros : ros, 
		name : editTopic, 
		messageType : 'sensor_msgs/Image' 
	});

	editListener.subscribe(function(message){
		that.widgetState = 'edit';
		that.editDiv.dialog();
	});

	$('#grabcut-edit-segmentation')
		.button()
		.click(function(even){
			console.log("clicked edit segmentation button");
		});

	$('#grabcut-edit-reset')
		.button()
		.click(function(even){
			console.log("clicked edit reset button");
		});


	

};

/**
 * @author Sarah Osentoski - sarah.osentoski@us.bosch.com
 */

/**
 * Manage drawing the image and drawing of the selection rectangle on the canvas
 */


/*global $:false */

GRABCUTSEGMENTATIONLIB.Selector = function(options){
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
//  this.canvas.style.background = '#aaaaaa';
  document.getElementById(divID).appendChild(this.canvas);


  this.stage = new createjs.Stage(this.canvas);
  console.log(this.canvas);
  console.log(this.stage);
  var context = this.canvas.getContext('2d');

  this.canvas.style.background = '#aaaaaa';

// use requestAnimationFrame if it exists
  var requestAnimationFrame = window.requestAnimationFrame || window.webkitRequestAnimationFrame
      || window.mozRequestAnimationFrame || window.oRequestAnimationFrame
      || window.msRequestAnimationFrame || function(callback) {
        setInterval(callback, 100);
      };

  /**
   * A function to draw the image onto the canvas.
   */
  function draw() {
    // clear the canvas
    that.canvas.width = that.canvas.width;

    // check if we have a valid image
    if (that.image.width * that.image.height > 0) {
      context.drawImage(that.image, 0, 0, that.width, that.height);
    } else {
      // center the error icon
      context.drawImage(errorIcon.image, (that.width - (that.width / 2)) / 2,
          (that.height - (that.height / 2)) / 2, that.width / 2, that.height / 2);
      that.emit('warning', 'Invalid stream.');
    }

    // check for an overlay
    if (overlay) {
      context.drawImage(overlay, 0, 0);
    }
    requestAnimationFrame(draw);
  }

  // grab the initial stream
  this.changeStream(topic);
//  draw();
  console.log(this.image.src);


  var mouseDown = false;
    var clickPosition = null;
  var position = null;
  var positonVec3 = null;
  var rect = null;  

  var mouseEventHandler = function (event, mouseState){
		if( mouseState == 'down'){
			console.log('mouse down');
      //if mouse is pushed down get the position and save it

			//get position where mouse button is pressed down
		    clickPosition = { x: event.stageX, y:  event.stageY};
		    
		    
			position = that.stage.globalToRos(event.stageX, event.stageY);
        	        positionVec3 = new ROSLIB.Vector3(position);
        	    
		    //remove previous rectangle
      if (rect){
        console.log('removing recct')
			 that.stage.removeChild(rect);
			 rect = null;
       that.stage.update();
		  }

		  mouseDown = true;

		}

    else if (mouseState === 'move') {
      if (mouseDown === true) {
	     //if mouse button is held down:
	      //get the current mouse position
	      //calculate distance from start position
	      
	      var currentClick={x: event.stageX, y: event.stageY};
	      var currentPos = that.stage.globalToRos(event.stageX, event.stageY);
	      currentPosVec3 = new ROSLIB.Vector3(currentPos);

	      var squareStart = {x: clickPosition.x, y: clickPosition.y};
	      
	      //calculate positions and rectangle information
	      if(clickPosition.x > currentClick.x)
	      {
		      squareStart.x = currentClick.x;
	      }
	      if(clickPosition.y > currentClick.y)
	      {
    		  squareStart.y = currentClick.y;
	      }

	      var distancex = Math.abs(clickPosition.x - currentClick.x);
	      var distancey = Math.abs(clickPosition.y - currentClick.y);
	      

	      console.log(currentClick);
	      //remove old rec so we can draw a new one
	      if(rect) {
		      that.stage.removeChild(rect);
		      rect = null;
	      }

	      rect = new createjs.Shape();
	      rect.graphics.beginStroke("#F00");
	      rect.graphics.drawRect(squareStart.x, squareStart.y, distancex, distancey);

	      that.stage.addChild(rect);
	      that.stage.update();

	  }
      }
      else { //mouseState === 'up'
	      //if mouse button is released
	      //stop updating square

	      mouseDown = false;
	  
      }

  };


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


GRABCUTSEGMENTATIONLIB.Selector.prototype.__proto__ = EventEmitter2.prototype;

/**
 * Change the stream of this canvas to the given topic.
 *
 * @param topic - the topic to stream, like '/wide_stereo/left/image_color'
 */
GRABCUTSEGMENTATIONLIB.Selector.prototype.changeStream = function(topic) {
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
  console.log(imagebitmap);
  this.stage.addChild(imagebitmap);
};