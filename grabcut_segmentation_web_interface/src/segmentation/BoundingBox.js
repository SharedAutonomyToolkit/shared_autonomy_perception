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
