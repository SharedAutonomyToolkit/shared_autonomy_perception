/**
 * @author Sarah Osentoski - sarah.osentoski@us.bosch.com
 */

/**
 * Manage drawing the image and drawing of the selection rectangle on the canvas
 *
 *
 * @constructor
 * @param options - object with the following keys:
 *  * stage - an easeljs stage that helps manage the canvas: www.createjs.com
 */


GRABCUTSEGMENTATIONLIB.BoundingBox = function(options){
    var that = this;

    var stage = options.stage;

    this.imageViewer= new GRABCUTSEGMENTATIONLIB.ImageViewer({
        stage : stage
    });


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
    stage.addChild(this.rect);

    /** 
     * Updates and redraws the current bounding-box for every mouse event.
     * Modifies that.bounds to save the coordinates
     */
    var mouseEventHandler = function (event, mouseState){
	if( mouseState == 'down'){
            //if mouse is pushed down get the position and save it
	    
	    mouseDown = true;
	    
	    firstClick = { x: event.stageX, y:  event.stageY};
	    
            // remove previous rectangle
            that.rect.graphics.clear();
            stage.update();
            that.bounds = null;
	}
        else if (mouseState === 'move') {
            if (mouseDown === true) {
	        //if mouse button is being held down:
	        //get the current mouse position
	        //calculate distance from start position
	        
	        var currentClick={x: event.stageX, y: event.stageY};
		
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
	        stage.update();
	    }
        }
        else { //mouseState === 'up'
	    //if mouse button is up, stop updating square on mouse move
	    mouseDown = false;
        }
    }; // end of mouseEventHandler
    
    
    //set up callbacks for the stage
    stage.addEventListener('stagemousedown', function(event) {
        mouseEventHandler(event,'down');
    });
    
    stage.addEventListener('stagemousemove', function(event) {
        mouseEventHandler(event,'move');
    });

    stage.addEventListener('stagemouseup', function(event) {
        mouseEventHandler(event,'up');
    });
};


/**
 * Return the bounds of the current rectangle, in expected JSON message format
*/
GRABCUTSEGMENTATIONLIB.BoundingBox.prototype.getBounds = function() {
    // TODO: Do I need logic that makes sure that we have valid bounds? 
    // what should happen if they're bad/segment is clicked before rectangle is drawn?
    var result = {
        min_row : {data : Math.round(this.bounds.y)},
        max_row : {data : Math.round(this.bounds.y + this.bounds.dy)},
	min_col : {data : Math.round(this.bounds.x)},
        max_col : {data : Math.round(this.bounds.x + this.bounds.dx)}
    };
    return result;
}

GRABCUTSEGMENTATIONLIB.BoundingBox.prototype.clearBounds = function() {
    this.rect.graphics.clear();
    this.bounds = null;
}


/**
 * Sets the current goal of the BoundingBox viewer
 */
GRABCUTSEGMENTATIONLIB.BoundingBox.prototype.setGoal = function(goalMessage) {
    this.imageViewer.updateDisplay(goalMessage.image);
}

GRABCUTSEGMENTATIONLIB.BoundingBox.prototype.__proto__ = EventEmitter2.prototype;
