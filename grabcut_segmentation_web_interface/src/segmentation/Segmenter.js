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

    var bboxCanvas = 'grabcut-bbox-canvas-display';
    var editCanvas = 'grabcut-edit-canvas-display';

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
    

    this.bboxDiv.html('<div id="' + bboxCanvasId + '"><canvas id ="' + bboxCanvas + '" width = "' + canvasWidth +'" height="' + canvasHeight + '"> </canvas><\/div> <br> <br><button id="grabcut-bbox">Segment</button> <button id="grabcut-reset">Reset</button>'); 	
    this.editDiv.html('<div id="' + editCanvasId + '"><\/div> <br> <br><button id="grabcut-edit">Segment</button>');



    var bboxViewer = new GRABCUTSEGMENTATIONLIB.BoundingBox({
    	divID : bboxCanvasId,
        canvasID : bboxCanvas,
    	host : host,
    	width : canvasWidth,
    	height : canvasHeight,
    	topic : bboxTopic
    });

    var bboxImageViewer= new GRABCUTSEGMENTATIONLIB.ImageViewer({
        ros : ros,
        canvasID : bboxCanvas,
        width : canvasWidth,
        height : canvasHeight

    });

    var editViewer = new GRABCUTSEGMENTATIONLIB.PixelEditor({
    	divID : editCanvasId,
    	host : host,
    	width : canvasWidth,
    	height : canvasHeight,
    	topic : editTopic
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
	console.log('bbox service call')
    //first display the new image so it's there when the dialog opens
    console.log('argh');
    console.log(goalMessage);
    bboxImageViewer.updateDisplay(goalMessage.goal.image);

    //open the dialog box
	that.bboxDiv.dialog("open");
    });
    this.editServer.on('goal', function(goalMessage) {
	console.log('edit service call')
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
	    // TODO: somehow need to clear the bbox so it doesn't pop on the next request. (however, that made the window show up immediately, so maybe we should have a tiny one by default?)
	});


    //setup edit button callbacks
    $('#grabcut-edit')
	.button()
	.click(function(event){
	    console.log("clicked segmentation button - testing");
	    // TODO: Do I need logic that makes sure that we have
	    // valid bounds? what should happen if they're bad?
            var bounds = editViewer.getbounds();

            var result = {
                min_row : {data : Math.round(bounds.y)},
                max_row : {data : Math.round(bounds.y + bounds.dy)},
		min_col : {data : Math.round(bounds.x)},
                max_col : {data : Math.round(bounds.x + bounds.dx)}
            };

	    that.editServer.setSucceeded(result);
            console.log("... set succeeded with: ");
            console.log(result);
	    that.editDiv.dialog("close");
	});

};
