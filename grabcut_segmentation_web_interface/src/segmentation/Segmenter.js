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

    // need 3 states - bbox, edit, idle
    this.widgetState = 'segement';
    this.Lock = false;

    // needs to match the divs declared in interactive_segmentation_interface.html
    var bboxCanvasId = 'grabcut-bbox-canvas';
    var editCanvasId = 'grabcut-edit-canvas';

    //add canvas and buttons to the window
    this.bboxDiv.dialog({
	autoOpen : false,
	width : canvasWidth,
	height : canvasHeight
    });
    

    this.bboxDiv.html('<div id="' + bboxCanvasId + '"><\/div> <br> <br><button id="grabcut-bbox">Segment</button> <button id="grabcut-reset">Reset</button>'); 	

    var bboxViewer = new GRABCUTSEGMENTATIONLIB.Selector({
    	divID : bboxCanvasId,
    	host : host,
    	width : canvasWidth,
    	height : canvasHeight,
    	topic : bboxTopic
    });

    //set up bbox topic subscriber
    var bboxListener = new ROSLIB.Topic({
	ros : ros, 
	name : bboxTopic, 
	messageType : 'sensor_msgs/Image'
    });

    var boundsPublisher = new ROSLIB.Topic({
        ros : ros,
        name : "/bbox",
        messageType : "shared_autonomy_msgs/BoundingBox"
    });

    //console.log(cameraListener);
    
    bboxListener.subscribe(function(message){
	that.widgetState = 'segment';
	console.log('bbox message');
        // TODO: seems like this should only happen on the first? 
        // or do subsequent calls update the image?
	that.bboxDiv.dialog("open");
    });

    //setup bbox button callbacks
    $('#grabcut-segmentation')
	.button()
	.click(function(event){
	    console.log("clicked segmentation button - testing");
            console.log("testing...");
            var bounds = bboxViewer.getbounds();
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
};
