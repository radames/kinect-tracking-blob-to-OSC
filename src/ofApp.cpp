#include "ofApp.h"
#include "ofConstants.h"

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
    

    depthImage.allocate(kinect.width, kinect.height);
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
    
    sender.setup(HOST, oscPort);

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
        gui.add(oscPort.set("OSC Port", 12345,5000,20000));
        gui.loadFromFile("settings.xml");

        gui.setPosition(20,340);
}

//--------------------------------------------------------------
void ofApp::update() {
    
    ofBackground(100, 100, 100);
    
    kinect.update();
    
    // there is a new frame and we are connected
    if(kinect.isFrameNew()) {
        
        depthImage.setFromPixels(kinect.getDepthPixels());
        depthImage.mirror(0,1);
        grayImage.setFromPixels(kinect.getDepthPixels());
        grayImage.mirror(0,1);
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
        depthImage.draw(0, 0);
        grayImage.draw(640, 0);
        ofTranslate(640,0);
        contourFinder.draw();
    ofPopMatrix();
    
    // draw instructions
    ofSetColor(255, 255, 255);
    
    processContour();
    
    
    gui.draw();

    
}
    
void ofApp::processContour() {
    //evaluate tracker
    
    ofxOscBundle bundle;
    
    RectTracker& tracker = contourFinder.getTracker();
    int count = 0;
    
    vector<float> objectsDepth;
    
    auto depthPixels = grayImage.getPixels();
    
    ofPushMatrix();
        ofScale(0.66,0.66);
        ofTranslate(640,0);
        for(int i=0; i < contourFinder.size(); i++){
            
            unsigned int label = contourFinder.getLabel(i);
            
            if(tracker.existsPrevious(label)) {
                
                
                // ellipse that best fits the contour
                ofSetColor(magentaPrint);
                RotatedRect ellipse = contourFinder.getFitEllipse(i);
                ofPushMatrix();
        
                    ofVec2f ellipseCenter = toOf(ellipse.center);
                    ofVec2f ellipseSize = toOf(ellipse.size);
                    ofTranslate(ellipseCenter.x, ellipseCenter.y);
                    ofRotate(ellipse.angle);
                    ofNoFill();
                    ofSetColor(255,0,0);
                    ofDrawEllipse(0, 0, ellipseSize.x, ellipseSize.y);
                
                ofPopMatrix();
                
                ofVec2f center = toOf(contourFinder.getCenter(i));
                
                
                
                
                int px0 = ellipseCenter.x - ellipseSize.x/2;
                int px1 = ellipseCenter.x + ellipseSize.x/2;

                px0 = ofClamp(px0, 0, kinect.width);
                px1 = ofClamp(px1, 0, kinect.width);

                float py0 = ellipseCenter.y - ellipseSize.y/2;
                float py1 = ellipseCenter.y + ellipseSize.y/2;
            
                py0 = ofClamp(py0, 0, kinect.height);
                py1 = ofClamp(py1, 0, kinect.height);

                
                float distanceValue = 0;
                int countAve = 0;
                for(int pointX = px0; pointX < px1; pointX++){
                    for(int pointY = py0; pointY < py1; pointY++){
                        distanceValue +=depthPixels[pointY * grayImage.width + pointX];
                        countAve++;
                    }
                }
            
                distanceValue = 255-(distanceValue/countAve);
                
                objectsDepth.push_back(distanceValue);

                ofSetColor(0,0,255);
                ofFill();
                ofDrawCircle(center, distanceValue/10);
                
                // convex hull of the contour
                ofSetColor(yellowPrint);
                ofPolyline convexHull = toOf(contourFinder.getConvexHull(i));
                convexHull.draw();
                
                // defects of the convex hull
                vector<cv::Vec4i> defects = contourFinder.getConvexityDefects(i);
                for(int j = 0; j < defects.size(); j++) {
                    ofDrawLine(defects[j][0], defects[j][1], defects[j][2], defects[j][3]);
                }
                
                
                ofxOscMessage mEllipse;
                //label, x, y, width, ellipse, angle
                
                
                mEllipse.setAddress("trackingOSC/ellipse/" + ofToString(count));
                mEllipse.addIntArg(label);
                mEllipse.addFloatArg(ellipseCenter.x);
                mEllipse.addFloatArg(ellipseCenter.y);
                
                mEllipse.addFloatArg(ellipseSize.x);
                mEllipse.addFloatArg(ellipseSize.y);
                mEllipse.addFloatArg(distanceValue);

                mEllipse.addFloatArg(ellipse.angle);

                bundle.addMessage(mEllipse);
    
                kinect.getDepthPixels();
                count++;
            }
        }
    
    ofPopMatrix();
    
    ofxOscMessage mObjects;
    //
    mObjects.setAddress("trackingOSC/allObjects");
    mObjects.addIntArg(objectsDepth.size());
    
    //tentative sum all depths, maybe
    float totalDepth = 0;
    for(int e = 0; e < objectsDepth.size(); e ++){
        totalDepth += objectsDepth[e];
        
    }
    
    mObjects.addFloatArg(totalDepth);
    bundle.addMessage(mObjects);
    
    
    
    sender.sendBundle(bundle);
    
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
