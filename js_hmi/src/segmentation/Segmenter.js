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
    // TODO: when is this required? we only do it in some function(options) definitions...
    options = options || {};

    var ros = options.ros;
    var bboxService = options.bboxService;
    var editService = options.editService;

    var bboxDiv = $('#' + options.bboxDiv);
    var editDiv = $('#' + options.editDiv);
    var canvasWidth = options.canvasWidth;
    var canvasHeight = options.canvasHeight;

    var bboxDivID = 'grabcut-bbox-canvas';
    var editDivID = 'grabcut-edit-canvas';

    var bboxCanvasID = 'grabcut-bbox-canvas-display';
    var editCanvasID = 'grabcut-edit-canvas-display';

    //add canvas and buttons to the window
    bboxDiv.dialog({
	autoOpen : false,
	width : canvasWidth,
	height : canvasHeight
    });
    
    editDiv.dialog({
	autoOpen : false,
	width : canvasWidth,
	height : canvasHeight
    });

    bboxDiv.html('<div id="' + bboxDivID + '"><canvas id ="' + bboxCanvasID + '" width = "' + canvasWidth +'" height="' + canvasHeight + '"> </canvas><\/div> <br> <br><button id="grabcut-bbox">Segment</button> <button id="grabcut-reset">Reset</button>');
    editDiv.html('<div id="' + editDivID + '"><canvas id ="' + editCanvasID + '" width = "' + canvasWidth +'" height="' + canvasHeight + '"> </canvas><\/div> <br> <br><button id="grabcut-edit">Segment</button> <button id="edit-foreground">Edit Foreground</button> <button id="edit-background">EditBackground</button>');

    var bboxCanvas = document.getElementById(bboxCanvasID);
    bboxStage = new createjs.Stage(bboxCanvas);

    var bboxViewer = new GRABCUTSEGMENTATIONLIB.BoundingBox({
        stage : bboxStage,
    });

    var editCanvas = document.getElementById(editCanvasID);
    editStage = new createjs.Stage(editCanvas);

    var editViewer = new GRABCUTSEGMENTATIONLIB.PixelEditor({
        stage : editStage, 
    	width : canvasWidth,
    	height : canvasHeight
    });

    var bboxServer = new ROSLIB.SimpleActionServer({
	ros : ros,
	serverName : bboxService,
	actionName : 'shared_autonomy_msgs/BoundingBoxAction'
    });

    var editServer = new ROSLIB.SimpleActionServer({
	ros : ros,
	serverName : editService,
	actionName : 'shared_autonomy_msgs/EditPixelAction'
    });

    //handle boundingbox action request
    bboxServer.on('goal', function(goalMessage) {
	bboxViewer.setGoal(goalMessage);
	bboxDiv.dialog("open");
    });
    
    //handle edit action request
    editServer.on('goal', function(goalMessage) {
        editViewer.setGoal(goalMessage);
	editDiv.dialog("open");
    });
    

    //setup bbox button callbacks
    $('#grabcut-bbox')
	.button()
	.click(function(event){
            var result = bboxViewer.getbounds();
	    bboxServer.setSucceeded(result);
	    bboxDiv.dialog("close");
	});


    //setup edit button callbacks
    $('#grabcut-edit')
	.button()
	.click(function(event){
            var result = editViewer.getlabels();
	    editServer.setSucceeded(result);
	    editDiv.dialog("close");
	});

    $('#edit-foreground')
        .button()
        .click(function(event) {
            editViewer.setForeground();
        });
    $('#edit-background')
        .button()
        .click(function(event) {
            editViewer.setBackground();
        });

};
