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


    this.bboxServer = new ROSLIB.SimpleActionServer({
	ros : ros,
	serverName : bboxService,
	actionName : 'shared_autonomy_msgs/BoundingBoxAction'
    });
    console.log(this.bboxServer);
    //console.log(cameraListener);

    // TODo; eventually, we hope to be able to pass image from this into 
    // the Selector, but for now, we have to assume that it'll have
    // received an image as well ...
    this.bboxServer.on('goal', function(goalMessage) {
	console.log('bbox service call')
	that.bboxDiv.dialog("open");
    });
    
    // OH! Unlike with the IM stuff, there's no reason that we can't 
 // have both bbox and edit windows open at once... but only one of each.
// this really needs to be the SAS callback ... 
    bboxListener.subscribe(function(message){
	console.log('bbox message');
        // TODO: seems like this should only happen on the first? 
        // or do subsequent calls update the image?
//	that.bboxDiv.dialog("open");
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
