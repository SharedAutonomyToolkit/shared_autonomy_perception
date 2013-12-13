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
    var canvasID = options.canvasID;
    this.ros = options.ros;
    this.width = options.width;
    this.height = options.height;
    
    console.log(canvasID);
    this.dest = document.getElementById(canvasID);
    this.dest.width = this.width;
    this.dest.height = this.height;
 
    this.destContext = this.dest.getContext('2d');
    
    this.src = null;
    this.srcContext = null;
    this.srcData = null;
    this.srcWidth = 0;
    this.srcHeight = 0;
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

    if (img.encoding != 'rgb8' && img.encoding != 'bgr8' ) return;
    
    //get the image data
    var imgWidth = img.width;
    var imgHeight = img.height;
    var imgPixels = window.atob(img.data);
    var imgLen = imgPixels.length;

    //Set up the canvas first
    if (this.src == null ||
	this.srcWidth != imgWidth ||
	this.srcHeight != imgHeight) {
	var div = document.createElement('div');
	div.innerHTML = '<canvas width="'+ imgWidth + '" height="' + imgHeight + '"></canvas>';
	this.src = div.firstChild;
	this.srcContex = this.src.getContext('2d');
	this.srcWidth = imgWidth;
	this.srcHeight = imgHeight;
	this.srcData = this.srcContex.getImageData(0,0,this.srcWidth,this.srcHeight);
	this.srcPixels = this.srcData.data; 
	console.log('created new canvas');
    }

    //Set up the images
    var i = 0;
    var j = 0;   
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
    else {
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
    this.srcContex.putImageData(this.srcData,0,0);
    this.destContext.save();
    this.destContext.scale(this.destWidth/this.srcWidth,this.destHeight/this.srcHeight);
    this.destContext.drawImage(this.src,0,0);
    this.destContext.restore();
    
};