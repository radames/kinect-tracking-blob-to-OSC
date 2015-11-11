#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxGui.h"
#include "ofxOsc.h"

#define HOST "localhost"


class ofApp : public ofBaseApp {
public:
    
    void setup();
    void update();
    void draw();
    void exit();
    
    void guiSetup();
    void processContour();
    void updateAngle(int& angle);
    
    void keyPressed(int key);
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    
    ofxKinect kinect;

    ofxCvGrayscaleImage depthImage; // grayscale depth image
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    ofxCv::ContourFinder contourFinder;
    

    
    //GUI
    ofxPanel gui; //
    
    ofParameterGroup parametersKinect;
    
    ofParameter<int> farThreshold;
    ofParameter<int> nearThreshold;
    ofParameter<int> angle;

    ofParameter<int> numMaxBlobs;
    ofParameter<int> minBlobSize;
    ofParameter<int> maxBlobSize;
    
    ofParameter<int> oscPort = 12345;
    
    //oscMessage Sender
    ofxOscSender sender;

};

