#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup() {
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    kinect.setRegistration(true);
    kinect.init(true);

    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    

    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    nearThreshold = 230;
    farThreshold = 70;
    
    ofSetFrameRate(60);
    
    // zero the tilt on startup
    angle = 0;
    kinect.setCameraTiltAngle(angle);
    
    // start from the front
    
    guiSetup();
}

void ofApp::guiSetup(){
    
    gui.setup("Settings", "settings.xml");
    
    
        parametersKinect.setName("Kinect");
        parametersKinect.add(farThreshold.set("Far Threshold", 0,0, 255 ));
        parametersKinect.add(nearThreshold.set("Near Threshold", 0,0, 255 ));
        parametersKinect.add(numMaxBlobs.set("Num Max Blobs",10,0,100));
        parametersKinect.add(maxBlobSize.set("Max Blob Size",0,0,500));
        parametersKinect.add(minBlobSize.set("Min Blob Size",0,0,500));
        parametersKinect.add(angle.set("Kinect Angle",0,-30,30));
    
        angle.addListener(this,&ofApp::updateAngle);

        gui.add(parametersKinect);
        
        gui.loadFromFile("settings.xml");

        gui.setPosition(20,340);
}

//--------------------------------------------------------------
void ofApp::update() {
    
    ofBackground(100, 100, 100);
    
    kinect.update();
    
    // there is a new frame and we are connected
    if(kinect.isFrameNew()) {
        
        grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        
        grayThreshNear = grayImage;
        grayThreshFar = grayImage;
        grayThreshNear.threshold(nearThreshold, true);
        grayThreshFar.threshold(farThreshold);
        cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        
        // update the cv images
        grayImage.flagImageChanged();

        
        contourFinder.setMinAreaRadius(minBlobSize);
        contourFinder.setMaxAreaRadius(maxBlobSize);
        contourFinder.findContours(grayImage);
    }

}

//--------------------------------------------------------------
void ofApp::draw() {
    
    ofSetColor(255, 255, 255);
    // draw from the live kinect
    ofPushMatrix();
        ofScale(0.66,0.66);
        kinect.drawDepth(0, 0);
        grayImage.draw(640, 0);
        ofTranslate(640,0);
        contourFinder.draw();
    ofPopMatrix();
    
    // draw instructions
    ofSetColor(255, 255, 255);
    
    gui.draw();

    
}

//--------------------------------------------------------------
void ofApp::exit() {
    kinect.close();
}

void ofApp::updateAngle(int &angle){
    
    kinect.setCameraTiltAngle(angle);

}


//--------------------------------------------------------------
void ofApp::keyPressed (int key) {}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{}
