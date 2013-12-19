/**
 * @author Sarah Osentoski - sarah.osentoski@us.bosch.com
 */

/**
 * Manage drawing the interactive segmentation input and interaction with ROS
 * This sets up two different SimpleActionServers, one that has the user
 * input a bounding box, and one that get individual pixel labels 
 * 
 * Depends upon easeljs and jquery and jquery-ui
 *
 * @constructor
 * @param options - object with the following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * bboxService - the boundingbox action service topic 
 *   * editService - the edit action service topic
 *   * canvasWidth -  desired height of the Width
 *   * canvasHeight - desired height of the canvas
 */

GRABCUTSEGMENTATIONLIB.Segmenter = function(options){
    var that = this;

    options = options || {};
    var ros = options.ros;
    var bboxService = options.bboxService;
    var editService = options.editService;


    this.bboxDiv = $('#' + options.bboxDiv);
    this.editDiv = $('#' + options.editDiv);
    var canvasWidth = options.canvasWidth;
    var canvasHeight = options.canvasHeight;

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

    //handle boundingbox action request
    this.bboxServer.on('goal', function(goalMessage) {
	console.log('bbox service call');
        bboxImageViewer.updateDisplay(goalMessage.image);
	that.bboxDiv.dialog("open");
    });
    
    //handle edit action request
    this.editServer.on('goal', function(goalMessage) {
	console.log('edit service call');
        editImageViewer.updateDisplay(goalMessage.image);
        editViewer.displayMask(goalMessage.mask);
	that.editDiv.dialog("open");
    });
    

    //setup bbox button callbacks
    $('#grabcut-bbox')
	.button()
	.click(function(event){
	    // TODO: Do I need logic that makes sure that we have
	    // valid bounds? what should happen if they're bad?
            var result = bboxViewer.getbounds();
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
