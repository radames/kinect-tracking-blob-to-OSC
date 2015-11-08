#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxGui.h"
#include "ofxOsc.h"
#include <tr1/unordered_map>

#define HOST "localhost"

#define S_PORT 12346


class ofApp : public ofBaseApp {
public:
    
    void setup();
    void update();
    void draw();
    void exit();
    
    void guiSetup();
    void updateAngle(int& angle);
    
    void keyPressed(int key);
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    
    ofxKinect kinect;

    ofxCvColorImage colorImg;
    
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
    
    //oscMessage Sender
    ofxOscSender sender;

};

