#include "ofApp.h"

using namespace cv;
using namespace ofxCv;

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
    
    contourFinder.setMinAreaRadius(10);
    contourFinder.setMaxAreaRadius(150);

    
    ofSetFrameRate(60);
    
    // zero the tilt on startup
    angle = 0;
    kinect.setCameraTiltAngle(angle);
    
    // start from the front
    
    guiSetup();
    
    sender.setup(HOST, S_PORT);

}

void ofApp::guiSetup(){
    
    gui.setup("Settings", "settings.xml");
    
    
        parametersKinect.setName("Kinect");
        parametersKinect.add(farThreshold.set("Far Threshold", 0,0, 255 ));
        parametersKinect.add(nearThreshold.set("Near Threshold", 0,0, 255 ));
        parametersKinect.add(numMaxBlobs.set("Num Max Blobs",10,0,1000));
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
        
        
        
        //evaluate tracker
        
        ofxOscBundle bundle;

        RectTracker& tracker = contourFinder.getTracker();
        int count = 0;
        
        vector<int> objectsDepth;
        
        unsigned char * depthPixels = grayImage.getPixels();

//        int numPixels = grayImage.getWidth() * grayImage.getHeight();
//        for(int i = 0; i < numPixels; i++) {
//            if(pix[i] < nearThreshold && pix[i] > farThreshold) {
//                pix[i] = 255;
//            } else {
//                pix[i] = 0;
//            }
//        }
        
        for(int i=0; i < contourFinder.size(); i++){
            
            unsigned int label = contourFinder.getLabel(i);
            
            if(tracker.existsPrevious(label)) {
                
                
                // ellipse that best fits the contour
                ofSetColor(magentaPrint);
                cv::RotatedRect ellipse = contourFinder.getFitEllipse(i);
                ofPushMatrix();
                
                    ofVec2f ellipseCenter = toOf(ellipse.center);
                    ofVec2f ellipseSize = toOf(ellipse.size);
                    ofTranslate(ellipseCenter.x, ellipseCenter.y);
                    ofRotate(ellipse.angle);
                    ofEllipse(0, 0, ellipseSize.x, ellipseSize.y);
                ofPopMatrix();
                
                ofVec2f center = toOf(contourFinder.getCenter(i));
                ofSetColor(yellowPrint);
                ofCircle(center, 1);
                
                ofxOscMessage mEllipse;
                ofxOscMessage mCenter;
                //label, x, y, width, ellipse, angle
                mEllipse.setAddress("trackingOSC/ellipse/" + ofToString(count));
                mEllipse.addIntArg(label);
                mEllipse.addFloatArg(ellipseCenter.x);
                mEllipse.addFloatArg(ellipseCenter.y);
                mEllipse.addFloatArg(ellipseSize.x);
                mEllipse.addFloatArg(ellipseSize.y);
                mEllipse.addFloatArg(ellipse.angle);
                //label, x, y, depth
                mCenter.setAddress("trackingOSC/center" + ofToString(count));
                mCenter.addIntArg(label);
                mCenter.addFloatArg(center.x);
                mCenter.addFloatArg(center.y);
                
                int depthValue =  depthPixels[int(center.y * grayImage.getWidth() + center.x)];
                
                objectsDepth.push_back(depthValue);
                
                mCenter.addIntArg(depthValue);
                
                bundle.addMessage(mEllipse);
                bundle.addMessage(mCenter);
        
                kinect.getDepthPixels();
                count++;
            }
        }
        
        
        ofxOscMessage mObjects;
        //
        mObjects.setAddress("trackingOSC/allObjects");
        mObjects.addIntArg(objectsDepth.size());
        
        //tentative sum all depths, maybe
        int totalDepth = 0;
        for(int e = 0; e < objectsDepth.size(); e ++){
            totalDepth += objectsDepth[e];
            
        }
        
        mObjects.addIntArg(totalDepth);
        bundle.addMessage(mObjects);

        

        sender.sendBundle(bundle);

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
