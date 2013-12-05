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

    var boundsPublisher = new ROSLIB.Topic({
        ros : ros,
        name : "/bbox",
        messageType : "shared_autonomy_msgs/BoundingBox"
    });

    //console.log(cameraListener);
	
	cameraListener.subscribe(function(message){
		that.widgetState = 'segment';
		console.log('cameramessage');
        // TODO: seems like this should only happen on the first? 
        // or do subsequent calls update the image?
		that.segmentationDiv.dialog("open");
	});

	//setup camera button callbacks
	$('#grabcut-segmentation')
		.button()
		.click(function(event){
			console.log("clicked segmentation button - testing");
            console.log("testing...");
            var bounds = segmentationViewer.getbounds();
            console.log(bounds);

            var msg = new ROSLIB.Message({
                min_row : Math.round(bounds.y),
                max_row : Math.round(bounds.y + bounds.dy),
                min_col : Math.round(bounds.x),
                max_col : Math.round(bounds.x + bounds.dx)
            });
            boundsPublisher.publish(msg);
            console.log("just published: ");
            console.log(msg);
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
